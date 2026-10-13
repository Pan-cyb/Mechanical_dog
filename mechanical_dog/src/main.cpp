#include <Arduino.h>
#include <Wire.h>
#include <cstdio>
#include <cstring>

#include "Board/ApplepyV1.h"
#include "Board/DebugWifiConfig.h"
#include "Ctrl/BenchSafety.h"
#include "Ctrl/ChannelCalibration.h"
#include "Ctrl/ServoController.h"
#include "Driver/HxyIcm42688p.h"
#include "Driver/Pca9685.h"
#include "Port/ArduinoClock.h"
#include "Port/ArduinoSerialConsole.h"
#include "Port/ArduinoWireBus.h"
#include "Port/DebugWebServer.h"
#include "Port/PreferencesCalibrationStore.h"
#include "Port/PreferencesMotionParameterStore.h"
#include "Port/PreferencesControlProfileStore.h"
#include "UserApp/RobotApp.h"

namespace {
dog::ArduinoClock clock_port;
dog::ArduinoWireBus bus_port(Wire);
dog::ArduinoSerialConsole console_port(Serial);
dog::Pca9685 pwm(bus_port, clock_port, dog::board::kPca9685Address,
                 dog::board::kServoFrequencyHz);
dog::HxyIcm42688p imu(bus_port, clock_port, dog::board::kHxyImuAddress);
dog::PreferencesCalibrationStore calibration_store(
    dog::board::kCalibrationFallbackChannel,
    dog::board::kCalibrationLegacyChannel);
dog::PreferencesMotionParameterStore motion_store;
dog::PreferencesControlProfileStore profile_store;
dog::ServoController servos(pwm, clock_port, dog::board::kBenchMinPulseUs,
                            dog::board::kBenchMaxPulseUs,
                            dog::board::kCommandTimeoutMs,
                            dog::board::kSelectedServoChannels, 8,
                            dog::board::kExtendedMinPulseUs,
                            dog::board::kExtendedMaxPulseUs);
dog::ChannelCalibration calibration(calibration_store, servos,
                                    dog::board::kSelectedServoChannels, 8,
                                    dog::board::kExtendedMinPulseUs,
                                    dog::board::kExtendedMaxPulseUs);
dog::RobotApp app(servos, calibration, imu, clock_port, console_port,
                  dog::board::kRelativeServoLimits, 8,
                  dog::board::kShankMapConfigs,
                  dog::board::kShankMapConfigCount,
                  &dog::board::kV10MotionConfig, &motion_store, &profile_store);
dog::DebugWebServer web(app);

char input_line[256] = {}; // Longest strict profile/drive command; drain remains 32 bytes/loop.
std::size_t input_used = 0;
bool discard_line = false;

void reportHxyDiagnostics() {
    dog::HxyDiagnostics d{};
    if (!imu.readDiagnostics(d)) {
        Serial.println("HXY_DIAG read failed (no registers changed)");
        return;
    }
    Serial.printf("HXY_DIAG WHO=%02X COM=%02X FILTER=%02X STAT=%02X ACC_CONF=%02X ACC_RANGE=%02X GYR_CONF=%02X GYR_RANGE=%02X PWR=%02X\r\n",
                  d.who_am_i, d.com_cfg, d.filter_cfg, d.data_stat,
                  d.acc_conf, d.acc_range, d.gyro_conf, d.gyro_range, d.power_ctrl);
    Serial.printf("HXY_DIAG ACC_ERR=%u GYR_ERR=%u DRDY_ACC=%u DRDY_GYR=%u RAW_ACC=%d,%d,%d RAW_GYR=%d,%d,%d\r\n",
                  (d.data_stat >> 4) & 1U, (d.data_stat >> 5) & 1U,
                  d.data_stat & 1U, (d.data_stat >> 1) & 1U,
                  d.accel_raw[0], d.accel_raw[1], d.accel_raw[2],
                  d.gyro_raw[0], d.gyro_raw[1], d.gyro_raw[2]);
    char bytes[3 * 12 + 1] = {};
    for (int i = 0; i < 12; ++i)
        std::snprintf(bytes + i * 3, sizeof(bytes) - i * 3, "%02X%s",
                      d.raw_bytes[i], i == 11 ? "" : " ");
    Serial.printf("HXY_DIAG BYTES=%s\r\n", bytes);
}

const char* rangeError(dog::HxyRangeTestError error) {
    using dog::HxyRangeTestError;
    switch (error) {
    case HxyRangeTestError::None: return "none";
    case HxyRangeTestError::NotReady: return "IMU not ready";
    case HxyRangeTestError::Identity: return "WHO read/mismatch";
    case HxyRangeTestError::InitialRangeRead: return "initial ACC_RANGE read";
    case HxyRangeTestError::UnexpectedInitialRange: return "unexpected initial ACC_RANGE";
    case HxyRangeTestError::ConfigRead: return "ACC_CONF/PWR_CTRL read";
    case HxyRangeTestError::UnexpectedConfig: return "unexpected ACC_CONF/PWR_CTRL";
    case HxyRangeTestError::RangeWrite: return "temporary ACC_RANGE write";
    case HxyRangeTestError::RangeRead: return "temporary ACC_RANGE read";
    case HxyRangeTestError::RangeMismatch: return "temporary ACC_RANGE mismatch";
    case HxyRangeTestError::StatusRead: return "DATA_STAT read";
    case HxyRangeTestError::DataNotReady: return "accelerometer data not ready";
    case HxyRangeTestError::SampleRead: return "raw sample read";
    case HxyRangeTestError::RestoreWrite: return "restore ACC_RANGE write";
    case HxyRangeTestError::RestoreRead: return "restore ACC_RANGE read";
    case HxyRangeTestError::RestoreMismatch: return "restore ACC_RANGE mismatch";
    }
    return "unknown";
}

void reportHxyRangeTest() {
    const dog::DebugSnapshot state = app.snapshot();
    if (state.preview_pending || !dog::canRunImuRangeTest(servos, state.imu_ok)) {
        Serial.println("HXY_RANGE REFUSED: require no preflight, PCA ready, IMU fresh, disarmed, ACTIVE=-1, no PWM fault; no IMU registers changed");
        return;
    }
    dog::HxyRangeTest test{};
    const bool ok = imu.runAccelRangeTest(test);
    Serial.printf("HXY_RANGE WHO=%02X original_known=%u ORIGINAL=%02X ACC_CONF=%02X PWR=%02X steps=%u\r\n",
                  test.identity, test.original_range_known ? 1U : 0U,
                  test.original_range, test.acc_conf, test.power_ctrl,
                  test.steps_started);
    for (unsigned stage = 0; stage < 4; ++stage) {
        const dog::HxyRangeStep& step = test.steps[stage];
        if (!step.started) continue;
        Serial.printf("HXY_RANGE STEP=%u requested=%02X read_ok=%u observed=%02X frames=%u\r\n",
                      stage, step.requested_range, step.range_read_ok ? 1U : 0U,
                      step.observed_range, step.frames_captured);
        for (unsigned frame = 0; frame < step.frames_captured; ++frame)
            Serial.printf("HXY_RANGE STEP=%u FRAME=%u STAT=%02X RAW_ACC=%d,%d,%d\r\n",
                          stage, frame, step.status[frame],
                          step.accel_raw[frame][0], step.accel_raw[frame][1],
                          step.accel_raw[frame][2]);
    }
    Serial.printf("HXY_RANGE RESTORE attempted=%u register_verified=%u readback=%02X post_frames=%u baseline_consistent=%u ERROR=%s RESULT=%s\r\n",
                  test.restoration_attempted ? 1U : 0U,
                  test.restoration_verified ? 1U : 0U,
                  test.restored_readback, test.steps[3].frames_captured,
                  test.data_returned_to_baseline ? 1U : 0U,
                  rangeError(test.error), ok ? "COMPLETE" : "FAIL");
    Serial.println("HXY_RANGE NOTE: register_verified is only 0x41 readback; baseline_consistent assumes the board stayed still, neither proves physical g scale");
    if (test.restoration_attempted && !test.restoration_verified)
        Serial.println("FAULT: ACC_RANGE restoration NOT verified. Do not ARM; reset board after checking I2C. Fault handler may send PCA all-off; disconnect servo power if connected.");
    else if (test.restoration_verified && !test.data_returned_to_baseline)
        Serial.println("HXY_RANGE WARNING: post-restore raw data not comparable to baseline (movement or sensor issue); do not use for closed-loop control");
}
}

void setup() {
    Serial.begin(115200); // CH340K UART0, not GPIO18/19 native USB.
    Wire.begin(dog::board::kI2cSdaPin, dog::board::kI2cSclPin,
               dog::board::kI2cFrequencyHz);
    Wire.setTimeOut(25);
    if (!calibration_store.begin())
        Serial.println("NVS unavailable: defaults loaded, SAVE will fail");
    if (!motion_store.begin())
        Serial.println("Motion NVS unavailable: built-in gait defaults loaded; motion_save will fail");
    if (!profile_store.begin())
        Serial.println("Profile NVS unavailable: built-in body/control defaults loaded; profile_save will fail");
    app.begin();
    if (web.begin()) {
        Serial.printf("Wi-Fi AP: %s / %s ; http://192.168.4.1/\r\n",
                      dog::board::kDebugApSsid, dog::board::kDebugApPassword);
    } else Serial.println("FAULT: Wi-Fi AP failed; use USB serial commands");
}

void loop() {
    app.tick();
    web.handleClient();
    app.tick(); // Keep polling IMU/fault handling after a possibly slow HTTP request.
    // A bounded nonblocking input drain keeps the 100 Hz IMU poll responsive.
    for (int count = 0; count < 32 && Serial.available() > 0; ++count) {
        const int value = Serial.read();
        if (value < 0) break;
        const char ch = static_cast<char>(value);
        if (ch == '\n' || ch == '\r') {
            if (discard_line) Serial.println("ERR: input line too long");
            else if (input_used > 0) {
                input_line[input_used] = '\0';
                if (std::strcmp(input_line, "imu_diag") == 0)
                    reportHxyDiagnostics();
                else if (std::strcmp(input_line, "imu_range_test") == 0)
                    reportHxyRangeTest();
                else {
                    app.onLine(input_line);
                    if (std::strcmp(input_line, "help") == 0)
                        Serial.println("imu_diag: read-only HXY dump | imu_range_test: bench-only temporary ACC_RANGE test with readback/restore (servo power OFF)");
                }
            }
            input_used = 0;
            discard_line = false;
        } else if (!discard_line) {
            if (input_used + 1 < sizeof(input_line)) input_line[input_used++] = ch;
            else discard_line = true;
        }
    }
    delay(1);
}
