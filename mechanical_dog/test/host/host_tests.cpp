#include <cassert>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "Board/ApplepyV1.h"
#include "Ctrl/AttitudeEstimator.h"
#include "Ctrl/BenchSafety.h"
#include "Ctrl/ChannelCalibration.h"
#include "Ctrl/Command.h"
#include "Ctrl/ServoController.h"
#include "Driver/HxyIcm42688p.h"
#include "Driver/Pca9685.h"
#include "UserApp/RobotApp.h"

namespace {
struct Clock : dog::IClock {
    std::uint32_t now = 0;
    std::uint32_t millis() const override { return now; }
    void delayMs(std::uint32_t ms) override { now += ms; }
};

struct Bus : dog::IRegisterBus {
    struct Write {
        std::uint8_t address;
        std::uint8_t reg;
        std::vector<std::uint8_t> bytes;
    };
    std::vector<Write> writes;
    std::array<bool, 16> channel_off{};
    bool fail_read = false;
    bool fail_write = false;
    std::uint8_t who = 0x6A;
    std::uint8_t accel_range = 0x01;
    std::uint8_t acc_conf = 0xA8;
    std::uint8_t power_ctrl = 0x0E;
    std::uint8_t data_stat = 0x03;
    int raw_reads = 0;
    int fail_raw_read_on = -1;
    int status_reads = 0;
    int fail_status_read_on = -1;
    int status_not_ready_from = -1;
    int range_reads = 0;
    int fail_range_read_on = -1;
    int fail_range_write_value = -1;
    int ignore_range_write_value = -1;
    bool range_sensitive_samples = false;
    std::uint8_t sample[12] = {};

    bool read(std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
              std::size_t len) override {
        if (fail_read) return false;
        if (address == 0x19 && reg == 0x41 && len == 1 &&
            ++range_reads == fail_range_read_on) return false;
        if (address == 0x19 && reg == 0x0C && len == 12 &&
            ++raw_reads == fail_raw_read_on) return false;
        if (address == 0x19 && reg == 0x0B && len == 1 &&
            ++status_reads == fail_status_read_on) return false;
        if (address == 0x19 && reg == 0x0B && len == 1 &&
            status_not_ready_from > 0 && status_reads >= status_not_ready_from) {
            data[0] = 0x02;
            return true;
        }
        if (address == 0x19 && reg == 0x01 && len == 1) data[0] = who;
        else if (address == 0x19 && len == 1 &&
                 (reg == 0x05 || reg == 0x08 || reg == 0x0B ||
                  (reg >= 0x40 && reg <= 0x43) || reg == 0x7D)) {
            switch (reg) {
            case 0x05: data[0] = 0x50; break;
            case 0x08: data[0] = 0x80; break;
            case 0x0B: data[0] = data_stat; break;
            case 0x40: data[0] = acc_conf; break;
            case 0x42: data[0] = 0xA8; break;
            case 0x41: data[0] = accel_range; break;
            case 0x43: data[0] = 0x01; break;
            default: data[0] = power_ctrl; break;
            }
        }
        else if (address == 0x19 && reg == 0x0C && len == 12) {
            std::memcpy(data, sample, len);
            if (range_sensitive_samples) {
                const std::int16_t y = accel_range == 0 ? -8192 :
                                       accel_range == 1 ? -4096 : -2048;
                data[2] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(y) >> 8);
                data[3] = static_cast<std::uint8_t>(y);
            }
        }
        else if (address == 0x40 && reg == 0 && len == 1) data[0] = 0;
        else return false;
        return true;
    }
    bool write(std::uint8_t address, std::uint8_t reg,
               const std::uint8_t* data, std::size_t len) override {
        if (fail_write) return false;
        writes.push_back({address, reg, std::vector<std::uint8_t>(data, data + len)});
        if (address == 0x19 && reg == 0x41 && len == 1) {
            if (data[0] != ignore_range_write_value) accel_range = data[0];
            // Model a bus error after the sensor has already accepted the write.
            if (data[0] == fail_range_write_value) return false;
        }
        if (address == 0x40 && reg == 0xFD && len == 1 && (data[0] & 0x10))
            channel_off.fill(true); // ALL_LED_OFF_H copies to all channel registers.
        if (address == 0x40 && reg >= 0x06 && reg <= 0x42 &&
            ((reg - 0x06) % 4 == 0) && len == 4)
            channel_off[(reg - 0x06) / 4] = (data[3] & 0x10) != 0;
        if (address == 0x40 && reg >= 0x09 && reg <= 0x45 &&
            ((reg - 0x09) % 4 == 0) && len == 1)
            channel_off[(reg - 0x09) / 4] = (data[0] & 0x10) != 0;
        return true;
    }
};

struct Pwm : dog::IServoPwm {
    int output_count = 0;
    int off_count = 0;
    int channel_off_count = 0;
    int last_off_channel = -1;
    bool fail_output = false;
    int fail_output_on_call = -1;
    bool fail_off = false;
    int fail_off_on_call = -1;
    bool fail_off_channel = false;
    std::array<bool, 16> active{};
    std::array<std::uint16_t, 16> pulse_us{};
    std::vector<int> events; // -1 = all-off, channel + 1 = successful pulse.
    bool begin() override { return true; }
    bool allOff() override {
        ++off_count;
        if (fail_off || off_count == fail_off_on_call) return false;
        active.fill(false);
        events.push_back(-1);
        return true;
    }
    bool offChannel(std::uint8_t channel) override {
        ++channel_off_count;
        if (fail_off_channel) return false;
        active[channel] = false;
        last_off_channel = channel;
        return true;
    }
    bool setPulseUs(std::uint8_t channel, std::uint16_t width) override {
        ++output_count;
        if (fail_output || output_count == fail_output_on_call) return false;
        active[channel] = true;
        pulse_us[channel] = width;
        events.push_back(static_cast<int>(channel) + 1);
        return true;
    }
};

struct CalibrationStore : dog::ICalibrationStore {
    std::array<bool, 16> present{};
    std::array<std::uint8_t, 16> value{};
    int load_count = 0;
    int save_count = 0;
    bool fail_save = false;
    int fail_save_on_call = -1;
    bool load(std::uint8_t channel, std::uint8_t& center_deg) override {
        ++load_count;
        if (channel >= value.size() || !present[channel]) return false;
        center_deg = value[channel];
        return true;
    }
    bool save(std::uint8_t channel, std::uint8_t center_deg) override {
        ++save_count;
        if (fail_save || save_count == fail_save_on_call ||
            channel >= value.size()) return false;
        present[channel] = true;
        value[channel] = center_deg;
        return true;
    }
};

struct Imu : dog::IImu {
    Clock& clock;
    bool fail = false;
    float accel[3] = {0.0f, 0.0f, 9.8f};
    float gyro[3] = {};
    explicit Imu(Clock& c) : clock(c) {}
    bool begin() override { return !fail; }
    bool read(dog::ImuSample& sample) override {
        if (fail) return false;
        sample.timestamp_ms = clock.now;
        for (int axis = 0; axis < 3; ++axis) {
            sample.accel_mps2[axis] = accel[axis];
            sample.gyro_radps[axis] = gyro[axis];
        }
        return true;
    }
};

struct Console : dog::IConsole {
    std::string last;
    void line(const char* text) override { last = text; }
};

bool hasWrite(const Bus& bus, std::uint8_t address, std::uint8_t reg,
              std::uint8_t value) {
    for (const auto& w : bus.writes)
        if (w.address == address && w.reg == reg && w.bytes.size() == 1 &&
            w.bytes[0] == value) return true;
    return false;
}
} // namespace

int main() {
    using namespace dog;
    const std::uint8_t expected_channels[8] = {0, 1, 2, 3, 12, 9, 10, 11};
    for (int i = 0; i < 8; ++i) {
        assert(board::kSelectedServoChannels[i] == expected_channels[i]);
        assert(board::kProposedJointChannels[i] == expected_channels[i]);
        assert(board::kRelativeServoLimits[i].channel == expected_channels[i]);
        assert(board::kV10MotionConfig.channels[i] == expected_channels[i]);
        for (int j = i + 1; j < 8; ++j)
            assert(board::kProposedJointChannels[i] !=
                   board::kProposedJointChannels[j]);
    }
    assert(std::strcmp(board::kRelativeServoLimits[4].joint, "L3H") == 0 &&
           board::kRelativeServoLimits[4].min_offset_deg == -180.0f &&
           board::kRelativeServoLimits[4].max_offset_deg == 180.0f &&
           !board::kRelativeServoLimits[4].shank);
    assert(board::kCalibrationFallbackChannel == 12 &&
           board::kCalibrationLegacyChannel == 8);
    assert(board::kCommandTimeoutMs == 0); // Manual hold on this physical board.

    // Six-axis attitude is sensor-frame only. Yaw is relative to first sample,
    // not a compass heading; invalid samples must never publish fake zeros.
    constexpr float g = 9.80665f;
    AttitudeEstimator estimator;
    AttitudeSample attitude{};
    ImuSample body{};
    body.accel_mps2[2] = g;
    body.gyro_radps[2] = 0.01f; // Static sensor bias to be measured, not yaw.
    for (int i = 0; i < 100; ++i) {
        body.timestamp_ms = 100 + i * 10;
        assert(estimator.update(body, attitude) == (i == 99));
    }
    assert(estimator.valid() && estimator.calibrated());
    assert(attitude.yaw_reference_ms == 1090);
    assert(std::fabs(attitude.roll_deg) < 0.01f &&
           std::fabs(attitude.pitch_deg) < 0.01f &&
           std::fabs(attitude.yaw_deg) < 0.01f);
    for (int i = 1; i <= 100; ++i) {
        body.timestamp_ms = 1090 + i * 10;
        assert(estimator.update(body, attitude));
    }
    assert(std::fabs(attitude.yaw_deg) < 0.1f); // Static Z bias is cancelled.
    body.gyro_radps[2] = 1.57079633f + 0.01f; // 90 degrees/s plus bias.
    for (int i = 1; i <= 100; ++i) {
        body.timestamp_ms = 2090 + i * 10;
        assert(estimator.update(body, attitude));
    }
    assert(std::fabs(attitude.yaw_deg - 90.0f) < 1.0f);
    assert(std::fabs(attitude.roll_deg) < 1.0f &&
           std::fabs(attitude.pitch_deg) < 1.0f);
    const AttitudeSample yaw_before_error = attitude;
    body.accel_mps2[2] = 0.0f;
    body.timestamp_ms += 10;
    assert(!estimator.update(body, attitude));
    assert(attitude.yaw_deg == yaw_before_error.yaw_deg);
    body.accel_mps2[2] = 0.0f;
    body.accel_mps2[1] = g;
    body.gyro_radps[2] = 0.0f;
    body.timestamp_ms += 200;
    assert(estimator.update(body, attitude)); // Gap resets yaw reference.
    assert(attitude.yaw_reference_ms == body.timestamp_ms);
    assert(std::fabs(attitude.roll_deg - 90.0f) < 1.0f);
    assert(std::fabs(attitude.yaw_deg) < 1.0f);
    AttitudeEstimator pitch_estimator;
    ImuSample pitched{};
    pitched.accel_mps2[0] = -g;
    for (int i = 0; i < 100; ++i) {
        pitched.timestamp_ms = 10 + i * 10;
        assert(pitch_estimator.update(pitched, attitude) == (i == 99));
    }
    assert(std::fabs(attitude.pitch_deg - 90.0f) < 1.0f);
    AttitudeEstimator moving_start;
    ImuSample still{};
    still.accel_mps2[2] = g;
    for (int i = 0; i < 50; ++i) {
        still.timestamp_ms = 10 + i * 10;
        assert(!moving_start.update(still, attitude));
    }
    still.timestamp_ms = 510;
    still.gyro_radps[2] = 0.1f; // Not stationary: discard the first 50 samples.
    assert(!moving_start.update(still, attitude));
    assert(moving_start.calibrationSamples() == 0);
    still.gyro_radps[2] = 0.0f;
    for (int i = 0; i < 100; ++i) {
        still.timestamp_ms = 520 + i * 10;
        assert(moving_start.update(still, attitude) == (i == 99));
    }
    const AttitudeSample before_duplicate = attitude;
    assert(!moving_start.update(still, attitude));
    assert(attitude.timestamp_ms == before_duplicate.timestamp_ms);

    AttitudeEstimator wrap_estimator;
    ImuSample wrapping{};
    wrapping.accel_mps2[2] = g;
    const std::uint32_t wrap_start = UINT32_MAX - 995U;
    for (std::uint32_t i = 0; i < 100; ++i) {
        wrapping.timestamp_ms = wrap_start + i * 10U;
        assert(wrap_estimator.update(wrapping, attitude) == (i == 99));
    }
    const std::uint32_t wrap_reference = attitude.yaw_reference_ms;
    wrapping.timestamp_ms = wrap_start + 1000U; // Rolls over, elapsed is 10ms.
    assert(wrap_estimator.update(wrapping, attitude));
    assert(attitude.yaw_reference_ms == wrap_reference);

    assert(parseCommand(" pwm 15 1500  ").type == CommandType::Pwm);
    assert(parseCommand("pwm 16 1500").type == CommandType::Invalid);
    assert(parseCommand("pwm 1 1500;evil").type == CommandType::Invalid);
    assert(parseCommand("pwm -1 1500").type == CommandType::Invalid);
    assert(parseCommand("arm garbage").type == CommandType::Invalid);
    assert(parseCommand("cal_start").type == CommandType::CalStart);
    assert(parseCommand("cal_start now").type == CommandType::Invalid);
    assert(parseCommand("cal_save_all").type == CommandType::CalSaveAll);
    assert(parseCommand("cal_save_all now").type == CommandType::Invalid);
    assert(parseCommand("off").type == CommandType::Off);
    const Command selected_off = parseCommand("off 8");
    assert(selected_off.type == CommandType::OffChannel && selected_off.channel == 8);
    assert(parseCommand("off -1").type == CommandType::Invalid);
    assert(parseCommand("off 16").type == CommandType::Invalid);
    assert(parseCommand("off 8 extra").type == CommandType::Invalid);
    assert(parseCommand("rpy").type == CommandType::Attitude);
    assert(parseCommand("attitude").type == CommandType::Attitude);
    const Command nudge = parseCommand("nudge 8 -1");
    assert(nudge.type == CommandType::Nudge && nudge.channel == 8 &&
           nudge.delta_deg == -1);
    assert(parseCommand("nudge 8 +2").type == CommandType::Invalid);
    assert(parseCommand("nudge 8 +1 extra").type == CommandType::Invalid);
    assert(parseCommand("save 11").type == CommandType::Save);
    assert(parseCommand("center 0").type == CommandType::Center);

    Clock clock;
    Bus bus;
    Pca9685 pca(bus, clock, 0x40, 50);
    assert(pca.begin());
    assert(hasWrite(bus, 0x40, 0xFE, 121)); // 25 MHz / (4096*50) - 1.
    assert(!bus.writes.empty());
    assert(bus.writes.back().reg == 0xFD && bus.writes.back().bytes[0] == 0x10);
    for (bool off : bus.channel_off) assert(off);
    assert(!pca.setPulseUs(16, 1500));
    assert(!pca.setPulseUs(0, 100));
    assert(pca.setPulseUs(15, 1500));
    assert(bus.writes.back().reg == 0x42); // LED15_ON_L.
    assert(bus.writes.back().bytes[2] == 0x33); // 307 ticks = 0x133.
    assert(bus.writes.back().bytes[3] == 0x01);
    for (int ch = 0; ch < 15; ++ch) assert(bus.channel_off[ch]);
    assert(!bus.channel_off[15]);
    const std::size_t before_invalid_off = bus.writes.size();
    assert(!pca.offChannel(16) && bus.writes.size() == before_invalid_off);
    assert(pca.offChannel(15));
    assert(bus.writes.back().reg == 0x45 && bus.writes.back().bytes.size() == 1 &&
           bus.writes.back().bytes[0] == 0x10);
    assert(bus.channel_off[15]);
    assert(pca.setPulseUs(8, 1500));
    assert(!bus.channel_off[8]);
    assert(pca.offChannel(0));
    assert(bus.writes.back().reg == 0x09 && !bus.channel_off[8]);
    assert(pca.offChannel(8));
    assert(bus.writes.back().reg == 0x29 && bus.channel_off[8]);
    assert(pca.allOff());
    for (bool off : bus.channel_off) assert(off);

    HxyIcm42688p hxy(bus, clock, 0x19);
    bus.who = 0x47; // TDK-like/wrong ID must not be configured as HXY.
    assert(!hxy.begin());
    bus.who = 0x6A;
    assert(hxy.begin());
    assert(hasWrite(bus, 0x19, 0x7D, 0x0E));
    assert(hasWrite(bus, 0x19, 0x41, 0x01));
    bus.sample[4] = 0x20; // accel z raw +8192 => +1 g at +/-4 g.
    bus.sample[5] = 0x00;
    bus.sample[6] = 0xF0; // gyro x raw -4096 => -125 dps.
    bus.sample[7] = 0x00;
    ImuSample sample{};
    assert(hxy.read(sample));
    assert(std::fabs(sample.accel_mps2[2] - 9.80665f) < 0.001f);
    assert(std::fabs(sample.gyro_radps[0] + 2.18166f) < 0.001f);
    const std::size_t prior_writes = bus.writes.size();
    HxyDiagnostics diagnostic{};
    assert(hxy.readDiagnostics(diagnostic));
    assert(diagnostic.who_am_i == 0x6A && diagnostic.acc_range == 0x01);
    assert(diagnostic.com_cfg == 0x50 && diagnostic.data_stat == 0x03);
    assert(diagnostic.accel_raw[2] == 8192 && diagnostic.gyro_raw[0] == -4096);
    assert(diagnostic.raw_bytes[4] == 0x20 && diagnostic.raw_bytes[6] == 0xF0);
    assert(bus.writes.size() == prior_writes); // Diagnostic must be read-only.
    const ImuSample before = sample;
    bus.fail_read = true;
    assert(!hxy.read(sample));
    assert(sample.timestamp_ms == before.timestamp_ms);
    assert(sample.accel_mps2[2] == before.accel_mps2[2]);
    const HxyDiagnostics diagnostic_before_failure = diagnostic;
    assert(!hxy.readDiagnostics(diagnostic));
    assert(diagnostic.acc_range == diagnostic_before_failure.acc_range);
    assert(diagnostic.raw_bytes[4] == diagnostic_before_failure.raw_bytes[4]);
    bus.fail_read = false;

    // A bench range experiment must write only HXY ACC_RANGE, return to the
    // original byte, and keep the ordinary 4g sample conversion unchanged.
    bus.range_sensitive_samples = true;
    bus.writes.clear();
    HxyRangeTest range_test{};
    assert(hxy.runAccelRangeTest(range_test));
    assert(range_test.identity == 0x6A && range_test.original_range_known);
    assert(range_test.original_range == 0x01 && range_test.acc_conf == 0xA8 &&
           range_test.power_ctrl == 0x0E && range_test.steps_started == 4);
    assert(range_test.restoration_attempted && range_test.restoration_verified &&
           range_test.restored_readback == 0x01);
    assert(range_test.steps[0].requested_range == 1 &&
           range_test.steps[1].requested_range == 0 &&
           range_test.steps[2].requested_range == 2 &&
           range_test.steps[3].requested_range == 1);
    for (const auto& step : range_test.steps)
        assert(step.range_read_ok && step.observed_range == step.requested_range &&
               step.frames_captured == 3 && (step.status[0] & 0x30) == 0);
    assert(range_test.steps[0].accel_raw[0][1] == -4096);
    assert(range_test.steps[1].accel_raw[0][1] == -8192);
    assert(range_test.steps[2].accel_raw[0][1] == -2048);
    assert(range_test.steps[3].accel_raw[0][1] == -4096);
    assert(range_test.data_returned_to_baseline);
    assert(bus.accel_range == 1 && clock.now >= 150);
    assert(bus.writes.size() == 3);
    assert(bus.writes[0].bytes[0] == 0 && bus.writes[1].bytes[0] == 2 &&
           bus.writes[2].bytes[0] == 1);
    for (const auto& write : bus.writes)
        assert(write.address == 0x19 && write.reg == 0x41 && write.bytes.size() == 1);

    bus.writes.clear();
    bus.fail_range_write_value = 0x02; // A NACK after the byte took effect.
    assert(!hxy.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::RangeWrite);
    assert(range_test.restoration_attempted && range_test.restoration_verified);
    assert(bus.accel_range == 1 && bus.writes.back().bytes[0] == 1);
    bus.fail_range_write_value = -1;
    bus.writes.clear();
    bus.fail_raw_read_on = bus.raw_reads + 4; // First +/-2g frame fails.
    assert(!hxy.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::SampleRead);
    assert(range_test.steps_started == 3 && range_test.steps[0].frames_captured == 3);
    assert(!range_test.steps[2].started && range_test.steps[3].started);
    assert(range_test.restoration_verified && range_test.steps[3].frames_captured == 3 &&
           range_test.data_returned_to_baseline && bus.accel_range == 1);
    bus.fail_raw_read_on = -1;

    bus.writes.clear();
    bus.fail_range_read_on = bus.range_reads + 3; // Readback after +/-2g write.
    assert(!hxy.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::RangeRead);
    assert(range_test.restoration_verified && bus.accel_range == 1);
    bus.fail_range_read_on = -1;

    bus.writes.clear();
    bus.fail_status_read_on = bus.status_reads + 4; // First +/-2g status.
    assert(!hxy.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::StatusRead);
    assert(range_test.restoration_verified && bus.accel_range == 1);
    bus.fail_status_read_on = -1;

    bus.writes.clear();
    bus.ignore_range_write_value = 0x02; // ACK without taking the new value.
    assert(!hxy.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::RangeMismatch);
    assert(range_test.restoration_verified && bus.accel_range == 1);
    bus.ignore_range_write_value = -1;

    bus.writes.clear();
    bus.fail_range_read_on = bus.range_reads + 5; // Restore readback fails.
    assert(!hxy.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::RestoreRead);
    assert(range_test.restoration_attempted && !range_test.restoration_verified);
    assert(bus.accel_range == 1 && bus.writes.back().bytes[0] == 1);
    assert(!hxy.read(sample)); // Unverified restoration locks out normal IMU.
    bus.fail_range_read_on = -1;
    assert(!hxy.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::NotReady);

    Bus unexpected_bus;
    HxyIcm42688p unexpected(unexpected_bus, clock, 0x19);
    assert(unexpected.begin());
    unexpected_bus.writes.clear();
    unexpected_bus.accel_range = 0x02;
    assert(!unexpected.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::UnexpectedInitialRange);
    assert(unexpected_bus.writes.empty());
    unexpected_bus.accel_range = 0x01;
    unexpected_bus.acc_conf = 0x00;
    assert(!unexpected.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::UnexpectedConfig);
    assert(unexpected_bus.writes.empty());
    unexpected_bus.acc_conf = 0xA8;
    unexpected_bus.data_stat = 0x02;
    assert(!unexpected.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::DataNotReady);
    assert(unexpected_bus.writes.empty()); // Baseline never became fresh.

    Bus delayed_bus;
    HxyIcm42688p delayed(delayed_bus, clock, 0x19);
    assert(delayed.begin());
    delayed_bus.writes.clear();
    delayed_bus.status_not_ready_from = delayed_bus.status_reads + 4;
    assert(!delayed.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::DataNotReady);
    assert(range_test.restoration_attempted && range_test.restoration_verified);
    assert(delayed_bus.accel_range == 1 && delayed_bus.writes.back().bytes[0] == 1);

    Bus restore_write_bus;
    HxyIcm42688p restore_write_imu(restore_write_bus, clock, 0x19);
    assert(restore_write_imu.begin());
    restore_write_bus.writes.clear();
    restore_write_bus.fail_range_write_value = 0x01;
    assert(!restore_write_imu.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::RestoreWrite);
    assert(range_test.restoration_attempted && !range_test.restoration_verified);
    assert(!restore_write_imu.read(sample));

    Bus restore_mismatch_bus;
    HxyIcm42688p restore_mismatch_imu(restore_mismatch_bus, clock, 0x19);
    assert(restore_mismatch_imu.begin());
    restore_mismatch_bus.writes.clear();
    restore_mismatch_bus.ignore_range_write_value = 0x01;
    assert(!restore_mismatch_imu.runAccelRangeTest(range_test));
    assert(range_test.error == HxyRangeTestError::RestoreMismatch);
    assert(range_test.restored_readback == 0x02 && !range_test.restoration_verified);
    assert(!restore_mismatch_imu.read(sample));

    Clock preflight_clock;
    Pwm preflight_pwm;
    ServoController preflight(preflight_pwm, preflight_clock, 1000, 2000, 3000);
    assert(!canRunImuRangeTest(preflight, true)); // PCA all-off not yet ACKed.
    assert(preflight.begin());
    assert(canRunImuRangeTest(preflight, true));
    assert(!canRunImuRangeTest(preflight, false));
    assert(preflight.arm() && !canRunImuRangeTest(preflight, true));
    assert(preflight.pulse(0, 1500) == ServoResult::Ok);
    assert(!canRunImuRangeTest(preflight, true));
    assert(preflight.off() && canRunImuRangeTest(preflight, true));
    assert(preflight.arm());
    preflight_pwm.fail_off = true;
    assert(!preflight.off() && !canRunImuRangeTest(preflight, true));

    Pwm fake;
    ServoController ctl(fake, clock, 1000, 2000, 3000);
    assert(ctl.begin() && fake.off_count == 1);
    assert(ctl.pulse(0, 1500) == ServoResult::NotArmed);
    assert(ctl.arm() && fake.output_count == 0);
    assert(ctl.pulse(0, 2500) == ServoResult::OutOfRange);
    assert(ctl.pulse(0, 1500) == ServoResult::Ok);
    clock.now += 2999;
    assert(!ctl.arm()); // Repeated ARM must not silently renew an active pulse.
    ctl.tick();
    assert(ctl.armed());
    clock.now += 1;
    ctl.tick();
    assert(!ctl.armed() && fake.off_count == 2);
    assert(ctl.pulse(0, 1500) == ServoResult::NotArmed);
    assert(ctl.arm());
    fake.fail_output = true;
    assert(ctl.pulse(0, 1500) == ServoResult::HardwareFault);
    assert(ctl.faulted() && !ctl.arm());

    Pwm failed_shutdown;
    ServoController timeout_ctl(failed_shutdown, clock, 1000, 2000, 3000);
    assert(timeout_ctl.begin() && timeout_ctl.arm());
    failed_shutdown.fail_off = true;
    clock.now += 3000;
    timeout_ctl.tick();
    assert(timeout_ctl.faulted() && !timeout_ctl.armed());

    // The UI's eight selectable silkscreen outputs are an enforced hardware
    // pool, not a joint/leg map. Selecting another output stops the old one.
    Clock pool_clock;
    Pwm pool_pwm;
    ServoController pool_ctl(pool_pwm, pool_clock, 1000, 2000,
                             board::kCommandTimeoutMs,
                             board::kSelectedServoChannels, 8);
    assert(pool_ctl.begin() && pool_ctl.arm());
    assert(pool_ctl.manualHold() && pool_ctl.remainingMs() == 0);
    pool_clock.now += 600000; // ARM alone does not expire or emit a pulse.
    pool_ctl.tick();
    assert(pool_ctl.armed() && pool_pwm.output_count == 0);
    assert(pool_ctl.allowed(0) && pool_ctl.allowed(3) &&
           pool_ctl.allowed(12) && pool_ctl.allowed(11));
    assert(!pool_ctl.allowed(4) && !pool_ctl.allowed(8));
    assert(pool_ctl.pulse(4, 1500) == ServoResult::OutOfRange);
    assert(pool_ctl.pulse(8, 1500) == ServoResult::OutOfRange);
    assert(pool_pwm.output_count == 0);
    assert(pool_ctl.pulse(0, 1500) == ServoResult::Ok);
    assert(pool_ctl.activeChannel() == 0 && pool_pwm.active[0]);
    pool_clock.now += 600000; // PWM remains on until OFF, not a timed pulse.
    pool_ctl.tick();
    assert(pool_ctl.armed() && pool_ctl.activeChannel() == 0 &&
           pool_pwm.active[0] && pool_pwm.off_count == 1);
    assert(pool_ctl.pulse(12, 1500) == ServoResult::Ok);
    assert(pool_ctl.activeChannel() == 12 && !pool_pwm.active[0] &&
           pool_pwm.active[12] && !pool_pwm.active[8]);
    assert((pool_pwm.events == std::vector<int>{-1, 1, -1, 13}));
    assert(pool_ctl.offChannel(4) == ServoResult::OutOfRange);
    assert(pool_ctl.offChannel(0) == ServoResult::NotActive);
    assert(pool_pwm.channel_off_count == 0 && pool_pwm.active[12]);
    const int all_off_before_selected = pool_pwm.off_count;
    assert(pool_ctl.offChannel(12) == ServoResult::Ok);
    assert(pool_pwm.channel_off_count == 1 && pool_pwm.last_off_channel == 12);
    assert(pool_pwm.off_count == all_off_before_selected && !pool_pwm.active[12]);
    assert(pool_ctl.armed() && pool_ctl.activeChannel() == -1 &&
           pool_ctl.activePulseUs() == 0);
    assert(pool_ctl.pulse(12, 1500) == ServoResult::Ok);
    const int previous_off_count = pool_pwm.off_count;
    assert(pool_ctl.pulse(12, 1600) == ServoResult::Ok);
    assert(pool_pwm.off_count == previous_off_count);
    assert(pool_ctl.off() && pool_ctl.activeChannel() == -1);
    for (bool channel_active : pool_pwm.active) assert(!channel_active);
    assert(!pool_ctl.armed());

    Pwm manual_failed_off_pwm;
    ServoController manual_failed_off(manual_failed_off_pwm, pool_clock,
                                      1000, 2000, board::kCommandTimeoutMs);
    assert(manual_failed_off.begin() && manual_failed_off.arm());
    assert(manual_failed_off.pulse(0, 1500) == ServoResult::Ok);
    manual_failed_off_pwm.fail_off = true;
    assert(!manual_failed_off.off());
    assert(manual_failed_off.faulted() && !manual_failed_off.armed() &&
           manual_failed_off_pwm.active[0]); // Physical PWM may persist!

    Pwm selected_off_failure_pwm;
    ServoController selected_off_failure(selected_off_failure_pwm, pool_clock,
                                         1000, 2000, board::kCommandTimeoutMs);
    assert(selected_off_failure.begin() && selected_off_failure.arm());
    assert(selected_off_failure.pulse(0, 1500) == ServoResult::Ok);
    selected_off_failure_pwm.fail_off_channel = true;
    selected_off_failure_pwm.fail_off = true; // Fallback all-off also NACKs.
    assert(selected_off_failure.offChannel(0) == ServoResult::HardwareFault);
    assert(selected_off_failure_pwm.channel_off_count == 1 &&
           selected_off_failure_pwm.off_count == 2);
    assert(selected_off_failure.faulted() && !selected_off_failure.armed() &&
           selected_off_failure_pwm.active[0]);

    // Explicit eight-channel center calibration is separate from single
    // bench PWM and starts only after a full preflight/all-off ACK.
    Clock eight_clock;
    Pwm eight_pwm;
    CalibrationStore eight_store;
    eight_store.present[1] = true;
    eight_store.value[1] = 92;
    ServoController eight_ctl(eight_pwm, eight_clock, 1000, 2000,
                              board::kCommandTimeoutMs,
                              board::kSelectedServoChannels, 8);
    ChannelCalibration eight_cal(eight_store, eight_ctl,
                                  board::kSelectedServoChannels, 8, 1000, 2000);
    assert(eight_ctl.begin());
    eight_cal.begin();
    assert(eight_pwm.output_count == 0 && eight_store.save_count == 0);
    assert(eight_cal.startAll() == ServoResult::Ok);
    assert(eight_ctl.armed() && eight_ctl.calibrating() &&
           eight_ctl.calibrationMask() == 0x1E0F && eight_ctl.activeChannel() == -1);
    assert(!canRunImuRangeTest(eight_ctl, true));
    assert(eight_pwm.output_count == 8 && eight_pwm.off_count == 2);
    for (std::uint8_t ch : board::kSelectedServoChannels) assert(eight_pwm.active[ch]);
    assert(eight_pwm.pulse_us[0] == 1500 && eight_pwm.pulse_us[1] == 1522);
    eight_clock.now += 600000;
    eight_ctl.tick();
    assert(eight_ctl.calibrating() && eight_pwm.active[11]);
    assert(eight_ctl.pulse(0, 1500) == ServoResult::ModeConflict);
    assert(eight_ctl.offChannel(0) == ServoResult::ModeConflict);
    assert(eight_pwm.output_count == 8 && eight_pwm.channel_off_count == 0);
    assert(eight_cal.nudge(1, +1) == CalibrationResult::Ok);
    assert(eight_cal.centerDeg(1) == 93 && eight_pwm.pulse_us[1] == 1533);
    assert(eight_pwm.output_count == 9 && eight_pwm.active[0] &&
           eight_pwm.active[12] && !eight_pwm.active[8] && eight_pwm.active[11]);
    assert(eight_cal.nudge(12, +1) == CalibrationResult::Ok);
    assert(eight_cal.centerDeg(12) == 91 && eight_pwm.pulse_us[12] == 1511 &&
           !eight_pwm.active[8]);
    assert(eight_cal.save(1) == CalibrationResult::DisarmBeforeSave);
    std::size_t saved_centers = 99;
    assert(eight_cal.saveAll(saved_centers) == CalibrationResult::DisarmBeforeSave &&
           saved_centers == 0);
    assert(eight_store.save_count == 0);
    assert(eight_ctl.off());
    assert(!eight_ctl.armed() && !eight_ctl.calibrating() &&
           eight_ctl.calibrationMask() == 0);
    for (bool active : eight_pwm.active) assert(!active);
    assert(eight_cal.save(1) == CalibrationResult::Ok &&
           eight_store.value[1] == 93);
    assert(eight_cal.saveAll(saved_centers) == CalibrationResult::Ok &&
           saved_centers == 8 && eight_store.value[1] == 93);
    assert(eight_store.present[12] && eight_store.value[12] == 91 &&
           !eight_store.present[8]);
    assert(eight_cal.startAll() == ServoResult::Ok);
    assert(eight_pwm.pulse_us[1] == 1533);
    eight_pwm.fail_output = true;
    assert(eight_cal.nudge(1, +1) == CalibrationResult::HardwareFault);
    assert(eight_cal.centerDeg(1) == 93 && eight_ctl.faulted() &&
           !eight_ctl.calibrating() && !eight_ctl.armed());
    for (bool active : eight_pwm.active) assert(!active);

    Pwm invalid_eight_pwm;
    ServoController invalid_eight(invalid_eight_pwm, eight_clock, 1000, 2000,
                                  board::kCommandTimeoutMs,
                                  board::kSelectedServoChannels, 8);
    assert(invalid_eight.begin());
    std::uint8_t duplicate[8] = {0, 1, 2, 3, 12, 9, 10, 10};
    std::uint16_t center_pulses[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    assert(invalid_eight.startCalibration(duplicate, center_pulses, 8) ==
           ServoResult::OutOfRange);
    center_pulses[7] = 2500;
    assert(invalid_eight.startCalibration(board::kSelectedServoChannels,
                                           center_pulses, 8) == ServoResult::OutOfRange);
    assert(invalid_eight_pwm.off_count == 1 && invalid_eight_pwm.output_count == 0);
    assert(invalid_eight.arm());
    center_pulses[7] = 1500;
    assert(invalid_eight.startCalibration(board::kSelectedServoChannels,
                                           center_pulses, 8) == ServoResult::ModeConflict);
    assert(invalid_eight_pwm.output_count == 0);
    assert(invalid_eight.off());

    Pwm no_preflight_pwm;
    ServoController no_preflight(no_preflight_pwm, eight_clock, 1000, 2000,
                                 board::kCommandTimeoutMs,
                                 board::kSelectedServoChannels, 8);
    assert(no_preflight.begin());
    no_preflight_pwm.fail_off_on_call = 2;
    assert(no_preflight.startCalibration(board::kSelectedServoChannels,
                                          center_pulses, 8) == ServoResult::HardwareFault);
    assert(no_preflight_pwm.output_count == 0 && no_preflight.faulted());

    Pwm partial_pwm;
    ServoController partial_ctl(partial_pwm, eight_clock, 1000, 2000,
                                board::kCommandTimeoutMs,
                                board::kSelectedServoChannels, 8);
    assert(partial_ctl.begin());
    center_pulses[7] = 1500;
    partial_pwm.fail_output_on_call = 4;
    assert(partial_ctl.startCalibration(board::kSelectedServoChannels,
                                         center_pulses, 8) == ServoResult::HardwareFault);
    assert(partial_pwm.output_count == 4 && partial_pwm.off_count == 3);
    assert(partial_ctl.faulted() && !partial_ctl.armed() &&
           !partial_ctl.calibrating());
    for (bool active : partial_pwm.active) assert(!active);

    Pwm last_write_pwm;
    ServoController last_write_ctl(last_write_pwm, eight_clock, 1000, 2000,
                                   board::kCommandTimeoutMs,
                                   board::kSelectedServoChannels, 8);
    assert(last_write_ctl.begin());
    last_write_pwm.fail_output_on_call = 8;
    assert(last_write_ctl.startCalibration(board::kSelectedServoChannels,
                                            center_pulses, 8) == ServoResult::HardwareFault);
    assert(last_write_pwm.output_count == 8 && last_write_pwm.off_count == 3 &&
           last_write_ctl.faulted());
    for (bool active : last_write_pwm.active) assert(!active);

    Pwm unknown_pwm;
    ServoController unknown_ctl(unknown_pwm, eight_clock, 1000, 2000,
                                board::kCommandTimeoutMs,
                                board::kSelectedServoChannels, 8);
    assert(unknown_ctl.begin());
    unknown_pwm.fail_output_on_call = 4;
    unknown_pwm.fail_off_on_call = 3; // Fallback fails after preflight ACK.
    assert(unknown_ctl.startCalibration(board::kSelectedServoChannels,
                                         center_pulses, 8) == ServoResult::HardwareFault);
    assert(unknown_ctl.faulted() && !unknown_ctl.calibrating() &&
           unknown_pwm.active[0] && unknown_pwm.active[2]);

    // Invalid or absent NVS values fall back to the reference 90 degrees.
    // +/-1 updates RAM and a single output only; flash is written on SAVE
    // after OFF, never on loading or each button press.
    CalibrationStore pool_store;
    pool_store.present[0] = true;
    pool_store.value[0] = 255;
    pool_store.present[1] = true;
    pool_store.value[1] = 20; // Nominal pulse falls below the bench limit.
    pool_store.present[12] = true;
    pool_store.value[12] = 94;
    ChannelCalibration cal(pool_store, pool_ctl, board::kSelectedServoChannels,
                           8, 1000, 2000);
    const int outputs_before_load = pool_pwm.output_count;
    cal.begin();
    assert(pool_store.load_count == 8 && pool_store.save_count == 0);
    assert(pool_pwm.output_count == outputs_before_load);
    assert(cal.centerDeg(0) == 90 && cal.centerUs(0) == 1500);
    assert(cal.centerDeg(1) == 90 && cal.centerDeg(12) == 94 &&
           cal.centerDeg(8) == 0);
    assert(cal.centerDeg(4) == 0 && cal.centerUs(4) == 0);
    assert(cal.nudge(4, +1) == CalibrationResult::InvalidChannel);
    assert(cal.nudge(0, +2) == CalibrationResult::InvalidStep);
    assert(cal.nudge(0, +1) == CalibrationResult::NotArmed);
    assert(cal.centerDeg(0) == 90 && pool_store.save_count == 0);
    assert(pool_ctl.arm());
    assert(pool_ctl.pulse(0, 1800) == ServoResult::Ok);
    const int raw_output_count = pool_pwm.output_count;
    assert(cal.nudge(0, +1) == CalibrationResult::ManualPulseActive);
    assert(pool_pwm.output_count == raw_output_count && cal.centerDeg(0) == 90);
    assert(pool_ctl.off() && pool_ctl.arm());
    assert(cal.nudge(0, +1) == CalibrationResult::Ok);
    assert(cal.centerDeg(0) == 91 && cal.centerUs(0) == 1511);
    assert(cal.nudge(0, -1) == CalibrationResult::Ok);
    assert(cal.centerDeg(0) == 90);
    assert(cal.nudge(0, +1) == CalibrationResult::Ok);
    assert(pool_store.save_count == 0);
    assert(cal.save(0) == CalibrationResult::DisarmBeforeSave);
    assert(pool_store.save_count == 0);
    assert(pool_ctl.off());
    assert(cal.save(0) == CalibrationResult::Ok);
    assert(pool_store.save_count == 1 && pool_store.value[0] == 91);
    ChannelCalibration reloaded(pool_store, pool_ctl,
                                board::kSelectedServoChannels, 8, 1000, 2000);
    reloaded.begin();
    assert(reloaded.centerDeg(0) == 91);
    pool_store.fail_save = true;
    assert(reloaded.save(0) == CalibrationResult::StorageFault);

    // A failed shutdown while switching channels must not command the new
    // output and latches the fault until reboot/physical power intervention.
    Pwm switching_pwm;
    ServoController switching_ctl(switching_pwm, pool_clock, 1000, 2000, 3000,
                                   board::kSelectedServoChannels, 8);
    assert(switching_ctl.begin() && switching_ctl.arm());
    assert(switching_ctl.pulse(0, 1500) == ServoResult::Ok);
    switching_pwm.fail_off = true;
    assert(switching_ctl.pulse(12, 1500) == ServoResult::HardwareFault);
    assert(switching_pwm.output_count == 1 && switching_ctl.faulted() &&
           !switching_ctl.armed() && !switching_ctl.arm());
    CalibrationStore fault_store;
    ChannelCalibration fault_cal(fault_store, switching_ctl,
                                 board::kSelectedServoChannels, 8, 1000, 2000);
    fault_cal.begin();
    assert(fault_cal.save(0) == CalibrationResult::HardwareFault);
    assert(fault_store.save_count == 0);

    Clock app_clock;
    Pwm app_pwm;
    Imu app_imu(app_clock);
    Console console;
    CalibrationStore app_store;
    ServoController app_servo(app_pwm, app_clock, 1000, 2000,
                             board::kCommandTimeoutMs,
                             board::kSelectedServoChannels, 8);
    ChannelCalibration app_cal(app_store, app_servo,
                               board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp app(app_servo, app_cal, app_imu, app_clock, console,
                 board::kRelativeServoLimits, 8);
    app.begin();
    assert(!app.snapshot().attitude_ok); // Nonblocking 1-second gyro calibration.
    for (int i = 1; i <= 100; ++i) {
        app_clock.now = i * 10;
        app.tick();
    }
    app.onLine("pwm 0 1500");
    assert(app_pwm.output_count == 0);
    assert(!app.execute(parseCommand("pwm 0 1500")).ok);
    assert(app.execute(parseCommand("arm")).ok); // Same path as Web POST.
    assert(app_servo.armed());
    app.onLine("pwm 0 1500");
    assert(app_pwm.output_count == 1);
    assert(app.execute(parseCommand("pwm 12 1500")).ok);
    assert(!app_pwm.active[0] && app_pwm.active[12] && !app_pwm.active[8]);
    assert(!app.execute(parseCommand("pwm 8 1500")).ok);
    assert(!app.execute(parseCommand("pwm 4 1500")).ok);
    assert(app_pwm.output_count == 2);
    const DebugSnapshot app_status = app.snapshot();
    assert(app_status.armed && app_status.imu_ok && app_status.active_channel == 12);
    assert(app_status.manual_hold && app_status.remaining_ms == 0);
    assert(app_status.attitude_ok && std::fabs(app_status.attitude.roll_deg) < 1.0f);
    app.onLine("rpy");
    assert(console.last.find("relative, drifts") != std::string::npos);
    assert(app_status.channel_count == 8 && app_status.channels[0].servo == 1 &&
           app_status.channels[4].servo == 13 &&
           app_status.channels[4].channel == 12 &&
           std::strcmp(app_status.channels[4].joint, "L3H") == 0);
    assert(app.execute(parseCommand("nudge 12 +1")).ok);
    assert(app_store.save_count == 0);
    assert(!app.execute(parseCommand("save 12")).ok);
    assert(app.execute(parseCommand("off")).ok);
    assert(app.execute(parseCommand("save 12")).ok && app_store.save_count == 1);
    assert(app_store.value[12] == 91 && !app_store.present[8]);
    assert(app.execute(parseCommand("arm")).ok);
    app_imu.fail = true;
    app_pwm.fail_off = true; // Shared I2C failure: shutdown cannot be assumed.
    app_clock.now += 10;
    app.tick();
    assert(!app_servo.armed());
    assert(app_servo.faulted());
    assert(!app.snapshot().attitude_ok);
    assert(console.last.find("disconnect servo power NOW") != std::string::npos);
    assert(!app.execute(parseCommand("arm")).ok); // Web path shares IMU/fault gate.
    assert(!app.execute(parseCommand("pwm 0 1500")).ok);
    app.onLine("arm"); // Serial path gets the same denied state.
    assert(!app_servo.armed());

    Clock healthy_clock;
    Pwm healthy_pwm;
    Imu healthy_imu(healthy_clock);
    Console healthy_console;
    CalibrationStore healthy_store;
    ServoController healthy_servos(healthy_pwm, healthy_clock, 1000, 2000,
                                   board::kCommandTimeoutMs,
                                   board::kSelectedServoChannels, 8);
    ChannelCalibration healthy_cal(healthy_store, healthy_servos,
                                   board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp healthy_app(healthy_servos, healthy_cal, healthy_imu, healthy_clock,
                         healthy_console);
    healthy_app.begin();
    assert(healthy_app.execute(parseCommand("arm")).ok);
    healthy_clock.now += 600000;
    healthy_app.tick();
    assert(healthy_servos.armed() && healthy_pwm.output_count == 0);
    assert(healthy_app.execute(parseCommand("pwm 0 1500")).ok);
    healthy_clock.now += 600000;
    healthy_app.tick();
    assert(healthy_servos.armed() && healthy_pwm.active[0]);
    assert(healthy_app.snapshot().manual_hold && healthy_app.snapshot().remaining_ms == 0);
    healthy_app.onLine("status");
    assert(healthy_console.last.find("MODE=manual-until-OFF") != std::string::npos);
    assert(healthy_app.execute(parseCommand("keepalive")).ok);
    assert(!healthy_app.execute(parseCommand("off 1")).ok);
    assert(healthy_servos.activeChannel() == 0 && healthy_pwm.active[0]);
    assert(healthy_app.execute(parseCommand("off 0")).ok);
    assert(healthy_servos.armed() && healthy_servos.activeChannel() == -1 &&
           !healthy_pwm.active[0]);
    assert(!healthy_app.execute(parseCommand("save 0")).ok);
    assert(healthy_store.save_count == 0);
    assert(healthy_app.execute(parseCommand("off")).ok);
    assert(!healthy_servos.armed() && healthy_servos.activeChannel() == -1);
    assert(healthy_app.execute(parseCommand("save 0")).ok);
    assert(healthy_store.save_count == 1);
    for (bool channel_active : healthy_pwm.active) assert(!channel_active);
    assert(!healthy_app.execute(parseCommand("pwm 0 1500")).ok);
    assert(healthy_pwm.output_count == 1);
    assert(healthy_app.execute(parseCommand("arm")).ok);
    assert(healthy_app.execute(parseCommand("pwm 0 1500")).ok);
    healthy_pwm.fail_off = true;
    assert(!healthy_app.execute(parseCommand("off")).ok);
    const DebugSnapshot manual_off_failure = healthy_app.snapshot();
    assert(manual_off_failure.fault && !manual_off_failure.armed &&
           manual_off_failure.active_channel == -1 && healthy_pwm.active[0]);
    healthy_app.onLine("status");
    assert(healthy_console.last.find("output may still be active") != std::string::npos);

    Clock timeout_clock;
    Pwm timeout_pwm;
    Imu timeout_imu(timeout_clock);
    Console timeout_console;
    CalibrationStore timeout_store;
    ServoController timeout_servos(timeout_pwm, timeout_clock, 1000, 2000, 3000,
                                   board::kSelectedServoChannels, 8);
    ChannelCalibration timeout_cal(timeout_store, timeout_servos,
                                   board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp timeout_app(timeout_servos, timeout_cal, timeout_imu, timeout_clock,
                         timeout_console);
    timeout_app.begin();
    timeout_app.onLine("arm");
    assert(timeout_app.execute(parseCommand("pwm 0 1500")).ok);
    timeout_pwm.fail_off = true;
    timeout_clock.now += 3000;
    timeout_app.tick();
    assert(timeout_servos.faulted());
    assert(timeout_console.last.find("disconnect servo power NOW") != std::string::npos);

    Clock center_clock;
    Pwm center_pwm;
    Imu center_imu(center_clock);
    Console center_console;
    CalibrationStore center_store;
    ServoController center_servos(center_pwm, center_clock, 1000, 2000,
                                  board::kCommandTimeoutMs,
                                  board::kSelectedServoChannels, 8);
    ChannelCalibration center_cal(center_store, center_servos,
                                   board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp center_app(center_servos, center_cal, center_imu, center_clock,
                        center_console);
    center_app.begin();
    assert(center_pwm.output_count == 0 && !center_app.snapshot().calibrating);
    assert(center_pwm.output_count == 0); // Calibration never starts automatically.
    assert(!center_app.execute(parseCommand("cal_save_all")).ok &&
           center_store.save_count == 0);
    assert(center_app.execute(parseCommand("cal_start")).ok);
    assert(center_app.snapshot().calibrating &&
           center_app.snapshot().calibration_pending &&
           center_app.snapshot().calibration_mask == 0x1E0F);
    assert(center_pwm.output_count == 8);
    assert(!center_app.execute(parseCommand("pwm 0 1600")).ok);
    assert(!center_app.execute(parseCommand("off 0")).ok);
    assert(!center_app.execute(parseCommand("cal_start")).ok);
    assert(center_pwm.output_count == 8);
    center_app.onLine("nudge 0 +1");
    assert(center_console.last.find("other channels hold") != std::string::npos);
    assert(center_pwm.output_count == 9 && center_pwm.active[11] &&
           center_cal.centerDeg(0) == 91);
    assert(!center_app.execute(parseCommand("save 0")).ok &&
           center_store.save_count == 0);
    center_app.onLine("status");
    assert(center_console.last.find("MODE=calibration") != std::string::npos);
    assert(center_app.execute(parseCommand("off")).ok);
    assert(!center_app.snapshot().calibrating);
    assert(center_app.execute(parseCommand("save 0")).ok &&
           center_store.value[0] == 91);
    assert(center_app.execute(parseCommand("cal_start")).ok);
    assert(center_app.execute(parseCommand("cal_save_all")).ok);
    assert(!center_servos.calibrating() && !center_app.snapshot().calibration_pending &&
           center_store.save_count == 9);
    assert(center_app.execute(parseCommand("cal_start")).ok);
    center_imu.fail = true;
    center_clock.now += 10;
    center_app.tick();
    assert(!center_app.snapshot().calibrating && !center_servos.armed());
    for (bool active : center_pwm.active) assert(!active);

    Clock partial_save_clock;
    Pwm partial_save_pwm;
    Imu partial_save_imu(partial_save_clock);
    Console partial_save_console;
    CalibrationStore partial_save_store;
    ServoController partial_save_ctl(partial_save_pwm, partial_save_clock,
                                     1000, 2000, board::kCommandTimeoutMs,
                                     board::kSelectedServoChannels, 8);
    ChannelCalibration partial_save_cal(partial_save_store, partial_save_ctl,
                                         board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp partial_save_app(partial_save_ctl, partial_save_cal,
                              partial_save_imu, partial_save_clock,
                              partial_save_console);
    partial_save_app.begin();
    assert(partial_save_app.execute(parseCommand("cal_start")).ok);
    partial_save_store.fail_save_on_call = 4;
    const DebugReply partial_result = partial_save_app.execute(parseCommand("cal_save_all"));
    assert(!partial_result.ok &&
           std::string(partial_result.message).find("partial") != std::string::npos);
    assert(!partial_save_ctl.armed() && !partial_save_ctl.calibrating() &&
           partial_save_app.snapshot().calibration_pending &&
           partial_save_store.save_count == 4);
    assert(partial_save_store.present[0] && !partial_save_store.present[3]);
    for (bool active : partial_save_pwm.active) assert(!active);
    partial_save_store.fail_save_on_call = -1;
    assert(partial_save_app.execute(parseCommand("cal_save_all")).ok);
    assert(partial_save_store.save_count == 12 &&
           !partial_save_app.snapshot().calibration_pending);

    Clock bad_exit_clock;
    Pwm bad_exit_pwm;
    Imu bad_exit_imu(bad_exit_clock);
    Console bad_exit_console;
    CalibrationStore bad_exit_store;
    ServoController bad_exit_ctl(bad_exit_pwm, bad_exit_clock, 1000, 2000,
                                  board::kCommandTimeoutMs,
                                  board::kSelectedServoChannels, 8);
    ChannelCalibration bad_exit_cal(bad_exit_store, bad_exit_ctl,
                                    board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp bad_exit_app(bad_exit_ctl, bad_exit_cal, bad_exit_imu,
                           bad_exit_clock, bad_exit_console);
    bad_exit_app.begin();
    assert(bad_exit_app.execute(parseCommand("cal_start")).ok);
    bad_exit_pwm.fail_off = true;
    assert(!bad_exit_app.execute(parseCommand("cal_save_all")).ok);
    assert(bad_exit_app.snapshot().fault &&
           bad_exit_app.snapshot().calibration_pending &&
           bad_exit_store.save_count == 0 && bad_exit_pwm.active[0]);
    assert(!bad_exit_app.execute(parseCommand("cal_save_all")).ok &&
           bad_exit_store.save_count == 0);

    // Field trace on COM4, static board: reported |a| is ~5.5 m/s^2 despite
    // a healthy HXY ID/I2C response. Attitude must not remain forever blank
    // just because the nominal accel scale has not been verified yet.
    Clock field_clock;
    Pwm field_pwm;
    Imu field_imu(field_clock);
    field_imu.accel[0] = -0.078f;
    field_imu.accel[1] = 0.845f;
    field_imu.accel[2] = 5.438f;
    field_imu.gyro[0] = -0.006f;
    field_imu.gyro[1] = 0.011f;
    field_imu.gyro[2] = -0.003f;
    Console field_console;
    CalibrationStore field_store;
    ServoController field_servos(field_pwm, field_clock, 1000, 2000, 3000,
                                 board::kSelectedServoChannels, 8);
    ChannelCalibration field_cal(field_store, field_servos,
                                 board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp field_app(field_servos, field_cal, field_imu, field_clock,
                       field_console);
    field_app.begin();
    for (int i = 1; i <= 200; ++i) {
        field_clock.now = i * 10;
        field_app.tick();
    }
    assert(field_app.snapshot().imu_ok);
    assert(field_app.snapshot().attitude_ok);
    assert(field_app.snapshot().gyro_calibrated);
    assert(field_app.snapshot().attitude_calibration_samples == 100);
    assert(field_app.snapshot().accel_norm_mps2 > 5.4f &&
           field_app.snapshot().accel_norm_mps2 < 5.6f);

    // Reproduce the reported webpage symptom at the real RobotApp seam:
    // a synchronous HTTP request occasionally delays one IMU poll by 40 ms.
    // The sensor remains healthy, but a 100-consecutive-sample calibration
    // must still eventually finish rather than reset on every page poll.
    Clock network_clock;
    Pwm network_pwm;
    Imu network_imu(network_clock);
    Console network_console;
    CalibrationStore network_store;
    ServoController network_servos(network_pwm, network_clock, 1000, 2000, 3000,
                                   board::kSelectedServoChannels, 8);
    ChannelCalibration network_cal(network_store, network_servos,
                                   board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp network_app(network_servos, network_cal, network_imu, network_clock,
                         network_console);
    network_app.begin();
    for (int i = 1; i <= 500; ++i) {
        network_clock.now += (i % 80 == 0) ? 40U : 10U;
        network_app.tick();
    }
    assert(network_app.snapshot().imu_ok);
    assert(network_app.snapshot().attitude_ok);

    // Combine the actual low-magnitude field vector with intermittent HTTP
    // pauses; neither condition may keep the webpage blank indefinitely.
    Clock combined_clock;
    Pwm combined_pwm;
    Imu combined_imu(combined_clock);
    combined_imu.accel[0] = field_imu.accel[0];
    combined_imu.accel[1] = field_imu.accel[1];
    combined_imu.accel[2] = field_imu.accel[2];
    combined_imu.gyro[0] = field_imu.gyro[0];
    combined_imu.gyro[1] = field_imu.gyro[1];
    combined_imu.gyro[2] = field_imu.gyro[2];
    Console combined_console;
    CalibrationStore combined_store;
    ServoController combined_servos(combined_pwm, combined_clock, 1000, 2000,
                                    3000, board::kSelectedServoChannels, 8);
    ChannelCalibration combined_cal(combined_store, combined_servos,
                                    board::kSelectedServoChannels, 8, 1000, 2000);
    RobotApp combined_app(combined_servos, combined_cal, combined_imu,
                          combined_clock, combined_console);
    combined_app.begin();
    for (int i = 1; i <= 500; ++i) {
        combined_clock.now += (i % 80 == 0) ? 40U : 10U;
        combined_app.tick();
    }
    assert(combined_app.snapshot().imu_ok && combined_app.snapshot().attitude_ok);
    assert(combined_app.snapshot().attitude_calibration_samples == 100);

    // User regression: silk Servo4 is CH3, its center must cross 135->136
    // in the default board configuration without a hidden range unlock.
    Clock full_clock;
    Pwm full_pwm;
    Imu full_imu(full_clock);
    Console full_console;
    CalibrationStore full_store;
    full_store.present[3] = true;
    full_store.value[3] = 135;
    ServoController full_ctl(full_pwm, full_clock, board::kBenchMinPulseUs,
                             board::kBenchMaxPulseUs, 0,
                             board::kSelectedServoChannels, 8,
                             board::kExtendedMinPulseUs, board::kExtendedMaxPulseUs);
    ChannelCalibration full_cal(full_store, full_ctl,
                                board::kSelectedServoChannels, 8, 500, 2500);
    RobotApp full_app(full_ctl, full_cal, full_imu, full_clock, full_console,
                      board::kRelativeServoLimits, 8);
    full_app.begin();
    assert(full_pwm.output_count == 0 && full_cal.centerDeg(3) == 135);
    assert(full_app.execute(parseCommand("cal_start")).ok);
    assert(full_app.execute(parseCommand("nudge 3 +1")).ok);
    assert(full_cal.centerDeg(3) == 136 && full_pwm.pulse_us[3] == 2011);
    assert(!full_app.execute(parseCommand("offset 3 +10")).ok); // Calibration mode exclusive.
    assert(full_store.value[3] == 135); // RAM adjustment does not save silently.
    assert(full_app.execute(parseCommand("cal_save_all")).ok);
    assert(full_store.value[3] == 136);
    assert(full_app.snapshot().pulse_min_us == 500 && full_app.snapshot().pulse_max_us == 2500);
    assert(std::string(full_app.snapshot().firmware_version) == "0.8.14");
    assert(full_app.snapshot().channels[3].max_offset_deg == 44);
    assert(parseCommand("offset 3 +30.0").type == CommandType::Offset);
    assert(parseCommand("offset 11 -50").offset_deg == -50);
    for (const char* bad : {"offset 3 nan", "offset 3 inf", "offset 3 1e1",
                           "offset 3 -181", "offset 3 2;evil", "offset 3 2 extra",
                           "offset 16 0", "offset 3 +", "offset 3 .5"})
        assert(parseCommand(bad).type == CommandType::Invalid);
    const int before_relative = full_pwm.output_count;
    assert(!full_app.execute(parseCommand("offset 3 +30")).ok); // No ARM.
    assert(full_pwm.output_count == before_relative);
    assert(full_app.execute(parseCommand("arm")).ok);
    assert(full_app.execute(parseCommand("offset 3 +30")).ok);
    assert(full_pwm.pulse_us[3] == 2344); // 136+30 = 166, NOT 90+30.
    assert(full_app.snapshot().active_pulse_us == 2344);
    assert(full_cal.centerDeg(3) == 136 && full_store.value[3] == 136);
    const int before_rejected = full_pwm.output_count;
    assert(!full_app.execute(parseCommand("offset 3 -1")).ok);
    assert(!full_app.execute(parseCommand("offset 3 +50")).ok); // 186 outside electrical range.
    assert(!full_app.execute(parseCommand("offset 4 0")).ok);
    assert(full_pwm.output_count == before_rejected);
    assert(full_app.execute(parseCommand("offset 12 +10")).ok);
    assert(full_pwm.pulse_us[12] == 1611 && full_pwm.pulse_us[8] == 0);
    const int before_old_l3h = full_pwm.output_count;
    assert(!full_app.execute(parseCommand("offset 8 0")).ok);
    assert(full_pwm.output_count == before_old_l3h);
    assert(full_app.execute(parseCommand("offset 3 0")).ok);
    assert(full_pwm.pulse_us[3] == 2011); // Return to calibrated center.
    for (const auto ch : {1, 11}) {
        const std::string allowed = "offset " + std::to_string(ch) + " -50";
        const std::string wrong = "offset " + std::to_string(ch) + " +1";
        assert(full_app.execute(parseCommand(allowed.c_str())).ok);
        assert(full_pwm.pulse_us[ch] == 944); // center90 -50 =40.
        const int outputs = full_pwm.output_count;
        assert(!full_app.execute(parseCommand(wrong.c_str())).ok);
        assert(full_pwm.output_count == outputs);
    }
    assert(full_app.execute(parseCommand("offset 9 +50")).ok);
    assert(full_pwm.pulse_us[9] == 2056); // center90 +50 =140.
    assert(!full_app.execute(parseCommand("offset 9 -1")).ok);
    assert(!full_app.execute(parseCommand("offset 9 +50.1")).ok);
    assert(full_app.execute(parseCommand("off")).ok);
    const int before_new_zero = full_pwm.output_count;
    full_store.value[3] = 120;
    full_app.begin(); // Newly saved mechanical neutral must become new origin.
    assert(full_pwm.output_count == before_new_zero);
    assert(full_app.execute(parseCommand("arm")).ok);
    assert(full_app.execute(parseCommand("offset 3 +30")).ok);
    assert(full_pwm.pulse_us[3] == 2167); // 120+30 =150.
    assert(full_cal.centerDeg(3) == 120 && full_store.value[3] == 120);
    assert(full_app.execute(parseCommand("off")).ok);

    // The pure map rejects invalid input without changing output and preserves
    // old NVS encoding exactly for every integer neutral at zero offset.
    for (int neutral = 0; neutral <= 180; ++neutral) {
        std::uint16_t pulse = 0;
        assert(relativeServoPulse(static_cast<float>(neutral), 0, -180, 180, pulse));
        assert(pulse == 500 + (2000*neutral+90)/180);
    }
    std::uint16_t relative_pulse = 1234;
    assert(!relativeServoPulse(120, std::nanf(""), 0, 50, relative_pulse));
    assert(!relativeServoPulse(-1, 0, -50, 0, relative_pulse));
    assert(!relativeServoPulse(100, -51, -50, 0, relative_pulse));
    assert(!relativeServoPulse(170, 50, 0, 50, relative_pulse));
    assert(relative_pulse == 1234);

    // One photo-derived magnitude curve is explicitly bound to all four shank
    // channels. Direction comes once from each original one-sided offset range.
    assert(parseCommand("shank 11 9").type == CommandType::Shank);
    assert(std::fabs(parseCommand("shank 11 13.9").shank_q_deg - 13.9f) < 0.001f);
    for (const char* bad : {"shank 11 nan", "shank 11 inf", "shank 11 1e1",
                            "shank 11 -1", "shank 11 .5", "shank 11 9;evil",
                            "shank 11 9 extra", "shank 16 9"})
        assert(parseCommand(bad).type == CommandType::Invalid);

    Clock q_clock;
    Pwm q_pwm;
    Imu q_imu(q_clock);
    Console q_console;
    CalibrationStore q_store;
    for (auto channel : board::kSelectedServoChannels) {
        q_store.present[channel] = true;
        q_store.value[channel] = 90;
    }
    q_store.value[1] = 95;
    q_store.value[3] = 105;
    q_store.value[9] = 110;
    q_store.value[11] = 100;
    ServoController q_ctl(q_pwm, q_clock, 500, 2500, 0,
                          board::kSelectedServoChannels, 8, 500, 2500);
    ChannelCalibration q_cal(q_store, q_ctl, board::kSelectedServoChannels, 8,
                             500, 2500);
    RobotApp q_app(q_ctl, q_cal, q_imu, q_clock, q_console,
                   board::kRelativeServoLimits, 8,
                   board::kShankMapConfigs, board::kShankMapConfigCount);
    q_app.begin();
    DebugSnapshot q_state = q_app.snapshot();
    struct ShankState { std::size_t index; std::uint8_t channel; int direction; };
    for (const auto& expected : {ShankState{1, 1, -1}, {3, 3, 1},
                                 {5, 9, 1}, {7, 11, -1}}) {
        const DebugSnapshot::Channel& channel = q_state.channels[expected.index];
        assert(channel.channel == expected.channel && channel.shank_q_available &&
               channel.shank_q_photo_candidate &&
               channel.shank_direction == expected.direction &&
               std::fabs(channel.max_shank_q_deg - 23.0f) < 0.001f &&
               channel.shank_map_point_count == 3);
        assert(channel.shank_map_points[1].u_deg == 30 &&
               channel.shank_map_points[1].q_deg == 9);
    }
    for (const std::size_t hip : {std::size_t{0}, std::size_t{2},
                                  std::size_t{4}, std::size_t{6}})
        assert(!q_state.channels[hip].shank_q_available &&
               q_state.channels[hip].shank_direction == 0);
    const int q_before_arm = q_pwm.output_count;
    assert(!q_app.execute(parseCommand("shank 1 0")).ok);
    assert(q_pwm.output_count == q_before_arm);
    assert(q_app.execute(parseCommand("arm")).ok);
    struct QPulse { const char* command; std::uint8_t channel; std::uint16_t pulse; };
    // q=0 always uses that channel's own current center.
    for (const auto& sample : {QPulse{"shank 1 0", 1, 1556},
                               {"shank 3 0", 3, 1667},
                               {"shank 9 0", 9, 1722},
                               {"shank 11 0", 11, 1611}}) {
        assert(q_app.execute(parseCommand(sample.command)).ok);
        assert(q_pwm.pulse_us[sample.channel] == sample.pulse);
    }
    // q=16 -> u=40; left legs subtract and right legs add exactly once.
    for (const auto& sample : {QPulse{"shank 1 16", 1, 1111},
                               {"shank 3 16", 3, 2111},
                               {"shank 9 16", 9, 2167},
                               {"shank 11 16", 11, 1167}}) {
        assert(q_app.execute(parseCommand(sample.command)).ok);
        assert(q_pwm.pulse_us[sample.channel] == sample.pulse);
    }
    // Endpoints and both segments remain shared with the original pure map.
    assert(q_app.execute(parseCommand("shank 11 9")).ok && q_pwm.pulse_us[11] == 1278);
    assert(q_app.execute(parseCommand("shank 11 23")).ok && q_pwm.pulse_us[11] == 1056);
    assert(q_cal.centerDeg(1) == 95 && q_cal.centerDeg(3) == 105 &&
           q_cal.centerDeg(9) == 110 && q_cal.centerDeg(11) == 100 &&
           q_store.value[1] == 95 && q_store.value[3] == 105 &&
           q_store.value[9] == 110 && q_store.value[11] == 100 &&
           q_store.save_count == 0);
    const int q_before_wrong_channel = q_pwm.output_count;
    assert(!q_app.execute(parseCommand("shank 0 9")).ok);
    assert(q_pwm.output_count == q_before_wrong_channel);
    Command nonfinite_q{};
    nonfinite_q.type = CommandType::Shank;
    nonfinite_q.channel = 11;
    nonfinite_q.shank_q_deg = std::nanf("");
    assert(!q_app.execute(nonfinite_q).ok);
    nonfinite_q.shank_q_deg = std::numeric_limits<float>::infinity();
    assert(!q_app.execute(nonfinite_q).ok);
    assert(q_pwm.output_count == q_before_wrong_channel);

    // A RAM center change is used immediately and is never saved by q.
    assert(q_app.execute(parseCommand("off")).ok);
    assert(q_app.execute(parseCommand("arm")).ok);
    assert(q_app.execute(parseCommand("nudge 11 -1")).ok);
    assert(q_cal.centerDeg(11) == 99 && q_store.value[11] == 100);
    assert(q_app.execute(parseCommand("shank 11 9")).ok);
    assert(q_pwm.pulse_us[11] == 1267); // 99 - u30 = 69 degrees.
    assert(q_app.execute(parseCommand("off")).ok);

    // Both actuator directions share q_max=13.9 near their electrical edge.
    // A fully clipped 0..0 range still publishes direction from raw config.
    q_store.value[1] = 37;
    q_store.value[3] = 143;
    q_store.value[9] = 180;
    q_store.value[11] = 0;
    const int q_before_reload = q_pwm.output_count;
    q_app.begin();
    assert(q_pwm.output_count == q_before_reload && q_cal.centerDeg(1) == 37 &&
           q_cal.centerDeg(3) == 143 && q_cal.centerDeg(9) == 180 &&
           q_cal.centerDeg(11) == 0);
    q_state = q_app.snapshot();
    assert(std::fabs(q_state.channels[1].max_shank_q_deg - 13.9f) < 0.001f &&
           q_state.channels[1].shank_direction == -1);
    assert(std::fabs(q_state.channels[3].max_shank_q_deg - 13.9f) < 0.001f &&
           q_state.channels[3].shank_direction == 1);
    assert(q_state.channels[5].max_shank_q_deg == 0 &&
           q_state.channels[5].shank_direction == 1);
    assert(q_state.channels[7].max_shank_q_deg == 0 &&
           q_state.channels[7].shank_direction == -1);
    assert(q_app.execute(parseCommand("arm")).ok);
    assert(q_app.execute(parseCommand("shank 1 13.9")).ok && q_pwm.pulse_us[1] == 500);
    assert(q_app.execute(parseCommand("shank 3 13.9")).ok && q_pwm.pulse_us[3] == 2500);
    assert(q_app.execute(parseCommand("shank 9 0")).ok && q_pwm.pulse_us[9] == 2500);
    assert(q_app.execute(parseCommand("shank 11 0")).ok && q_pwm.pulse_us[11] == 500);
    const int q_at_boundary = q_pwm.output_count;
    assert(!q_app.execute(parseCommand("shank 1 13.91")).ok);
    assert(!q_app.execute(parseCommand("shank 3 13.91")).ok);
    assert(!q_app.execute(parseCommand("shank 1 23")).ok);
    assert(!q_app.execute(parseCommand("shank 3 23")).ok);
    assert(q_pwm.output_count == q_at_boundary);

    // Eight-channel calibration remains exclusive and IMU loss cannot add a
    // movement. q never writes NVS in any rejected state.
    assert(q_app.execute(parseCommand("off")).ok);
    assert(q_app.execute(parseCommand("cal_start")).ok);
    const int q_during_cal = q_pwm.output_count;
    assert(!q_app.execute(parseCommand("shank 11 0")).ok);
    assert(q_pwm.output_count == q_during_cal);
    assert(q_app.execute(parseCommand("off")).ok);
    assert(q_app.execute(parseCommand("arm")).ok);
    q_imu.fail = true;
    q_clock.now += 10;
    q_app.tick();
    const int q_after_imu_fault = q_pwm.output_count;
    assert(!q_app.execute(parseCommand("shank 11 0")).ok);
    assert(q_pwm.output_count == q_after_imu_fault && q_store.save_count == 0);

    // Once the PWM controller faults, later q requests are rejected before a
    // second hardware write is attempted.
    Clock q_fault_clock;
    Pwm q_fault_pwm;
    Imu q_fault_imu(q_fault_clock);
    Console q_fault_console;
    CalibrationStore q_fault_store;
    q_fault_store.present[11] = true;
    q_fault_store.value[11] = 100;
    ServoController q_fault_ctl(q_fault_pwm, q_fault_clock, 500, 2500, 0,
                                board::kSelectedServoChannels, 8, 500, 2500);
    ChannelCalibration q_fault_cal(q_fault_store, q_fault_ctl,
                                   board::kSelectedServoChannels, 8, 500, 2500);
    RobotApp q_fault_app(q_fault_ctl, q_fault_cal, q_fault_imu, q_fault_clock,
                         q_fault_console, board::kRelativeServoLimits, 8,
                         board::kShankMapConfigs, board::kShankMapConfigCount);
    q_fault_app.begin();
    assert(q_fault_app.execute(parseCommand("arm")).ok);
    q_fault_pwm.fail_output = true;
    assert(!q_fault_app.execute(parseCommand("shank 11 9")).ok);
    const int q_fault_writes = q_fault_pwm.output_count;
    assert(!q_fault_app.execute(parseCommand("shank 11 9")).ok);
    assert(q_fault_pwm.output_count == q_fault_writes && q_fault_ctl.faulted());

    // Configuration-table mismatch must not treat an unselected channel's
    // default center as a valid calibration, even for q=0.
    constexpr std::uint8_t no_ch11_channels[] = {0};
    Clock q_mismatch_clock;
    Pwm q_mismatch_pwm;
    Imu q_mismatch_imu(q_mismatch_clock);
    Console q_mismatch_console;
    CalibrationStore q_mismatch_store;
    ServoController q_mismatch_ctl(q_mismatch_pwm, q_mismatch_clock, 500, 2500, 0,
                                   board::kSelectedServoChannels, 8, 500, 2500);
    ChannelCalibration q_mismatch_cal(q_mismatch_store, q_mismatch_ctl,
                                      no_ch11_channels, 1, 500, 2500);
    RobotApp q_mismatch_app(q_mismatch_ctl, q_mismatch_cal, q_mismatch_imu,
                            q_mismatch_clock, q_mismatch_console,
                            board::kRelativeServoLimits, 8,
                            board::kShankMapConfigs, board::kShankMapConfigCount);
    q_mismatch_app.begin();
    assert(q_mismatch_app.execute(parseCommand("arm")).ok);
    const int q_mismatch_before = q_mismatch_pwm.output_count;
    assert(!q_mismatch_app.execute(parseCommand("shank 11 0")).ok);
    assert(q_mismatch_pwm.output_count == q_mismatch_before);

    // Direction must be a one-sided raw configuration. Two-sided and zero-only
    // ranges are ambiguous even though center clipping could produce 0..0.
    RelativeServoLimit bad_direction_limit[] = {{11, "bad", -50, 50, true}};
    Clock bad_direction_clock;
    Pwm bad_direction_pwm;
    Imu bad_direction_imu(bad_direction_clock);
    Console bad_direction_console;
    CalibrationStore bad_direction_store;
    bad_direction_store.present[11] = true;
    bad_direction_store.value[11] = 100;
    ServoController bad_direction_ctl(bad_direction_pwm, bad_direction_clock,
                                      500, 2500, 0,
                                      board::kSelectedServoChannels, 8, 500, 2500);
    ChannelCalibration bad_direction_cal(bad_direction_store, bad_direction_ctl,
                                         board::kSelectedServoChannels, 8,
                                         500, 2500);
    RobotApp bad_direction_app(bad_direction_ctl, bad_direction_cal,
                               bad_direction_imu, bad_direction_clock,
                               bad_direction_console, bad_direction_limit, 1,
                               board::kShankMapConfigs,
                               board::kShankMapConfigCount);
    bad_direction_app.begin();
    assert(bad_direction_app.execute(parseCommand("arm")).ok);
    const int bad_direction_before = bad_direction_pwm.output_count;
    assert(!bad_direction_app.snapshot().channels[7].shank_q_available);
    assert(!bad_direction_app.execute(parseCommand("shank 11 0")).ok);
    assert(bad_direction_pwm.output_count == bad_direction_before);
    assert(bad_direction_app.execute(parseCommand("off")).ok);
    bad_direction_limit[0].min_offset_deg = 0;
    bad_direction_limit[0].max_offset_deg = 0;
    assert(!bad_direction_app.snapshot().channels[7].shank_q_available);
    assert(bad_direction_app.execute(parseCommand("arm")).ok);
    assert(!bad_direction_app.execute(parseCommand("shank 11 0")).ok);
    assert(bad_direction_pwm.output_count == bad_direction_before &&
           bad_direction_store.save_count == 0);

    // Explicit range unlock must cover every PWM path, be volatile, and
    // never move an actuator or write NVS just by changing the range.
    assert(parseCommand("range normal").type == CommandType::RangeNormal);
    assert(parseCommand("range\textended ").type == CommandType::RangeExtended);
    assert(parseCommand("range extended extra").type == CommandType::Invalid);
    assert(parseCommand("range extended;evil").type == CommandType::Invalid);
    assert(parseCommand("rangeextended").type == CommandType::Invalid);
    Clock range_clock;
    Pwm range_pwm;
    CalibrationStore range_store;
    Imu range_imu(range_clock);
    Console range_console;
    for (auto channel : board::kSelectedServoChannels) {
        range_store.present[channel] = true;
        range_store.value[channel] = 90;
    }
    range_store.value[0] = 89; // Prior NVS encoding remains byte-for-byte meaningful.
    range_store.value[1] = 1;
    range_store.value[2] = 179;
    ServoController range_ctl(range_pwm, range_clock, 1000, 2000, 0,
                              board::kSelectedServoChannels, 8, 500, 2500);
    ChannelCalibration range_cal(range_store, range_ctl,
                                 board::kSelectedServoChannels, 8, 500, 2500);
    RobotApp range_app(range_ctl, range_cal, range_imu, range_clock, range_console);
    range_app.begin();
    assert(!range_app.snapshot().extended_range && range_pwm.output_count == 0);
    assert(range_app.snapshot().pulse_min_us == 1000);
    assert(range_app.snapshot().pulse_max_us == 2000);
    assert(range_cal.centerDeg(0) == 89 && range_cal.centerUs(0) == 1489);
    assert(range_cal.centerDeg(1) == 1 && range_cal.centerDeg(2) == 179);
    assert(!range_app.execute(parseCommand("cal_start")).ok);
    assert(range_pwm.output_count == 0); // All-eight preflight: none can move.
    assert(range_app.execute(parseCommand("arm")).ok);
    assert(!range_app.execute(parseCommand("pwm 0 999")).ok);
    assert(!range_app.execute(parseCommand("pwm 0 2001")).ok);
    assert(!range_app.execute(parseCommand("range extended")).ok);
    assert(range_pwm.output_count == 0);
    assert(range_app.execute(parseCommand("off")).ok);
    const int off_before_range = range_pwm.off_count;
    assert(range_app.execute(parseCommand("range extended")).ok);
    assert(range_app.snapshot().extended_range && range_store.save_count == 0);
    assert(range_pwm.output_count == 0 && range_pwm.off_count == off_before_range);
    assert(range_app.snapshot().pulse_min_us == 500);
    assert(range_app.snapshot().pulse_max_us == 2500);
    assert(range_app.execute(parseCommand("arm")).ok);
    assert(!range_app.execute(parseCommand("pwm 0 499")).ok);
    assert(!range_app.execute(parseCommand("pwm 0 2501")).ok);
    assert(!range_app.execute(parseCommand("pwm 4 1500")).ok);
    assert(range_pwm.output_count == 0);
    assert(range_app.execute(parseCommand("pwm 0 500")).ok);
    assert(range_pwm.pulse_us[0] == 500);
    assert(range_app.execute(parseCommand("pwm 0 2500")).ok);
    assert(range_pwm.pulse_us[0] == 2500);
    assert(!range_app.execute(parseCommand("range normal")).ok);
    assert(range_app.execute(parseCommand("off 0")).ok);
    assert(!range_app.execute(parseCommand("range normal")).ok); // ARM still on.
    assert(range_app.execute(parseCommand("off")).ok);
    assert(range_app.execute(parseCommand("cal_start")).ok);
    assert(!range_app.execute(parseCommand("range normal")).ok);
    assert(!range_app.execute(parseCommand("pwm 0 1500")).ok);
    assert(range_app.execute(parseCommand("nudge 1 -1")).ok);
    assert(range_cal.centerDeg(1) == 0 && range_pwm.pulse_us[1] == 500);
    assert(!range_app.execute(parseCommand("nudge 1 -1")).ok);
    assert(range_app.execute(parseCommand("nudge 2 +1")).ok);
    assert(range_cal.centerDeg(2) == 180 && range_pwm.pulse_us[2] == 2500);
    assert(!range_app.execute(parseCommand("nudge 2 +1")).ok);
    assert(range_app.execute(parseCommand("cal_save_all")).ok);
    assert(range_store.value[1] == 0 && range_store.value[2] == 180);
    const int outputs_before_reboot = range_pwm.output_count;
    range_app.begin(); // Reset unlock but still load wide centers, no movement.
    assert(!range_app.snapshot().extended_range);
    assert(range_cal.centerDeg(1) == 0 && range_cal.centerDeg(2) == 180);
    assert(range_pwm.output_count == outputs_before_reboot);
    assert(!range_app.execute(parseCommand("cal_start")).ok);
    assert(range_pwm.output_count == outputs_before_reboot);
    assert(range_app.execute(parseCommand("range extended")).ok);
    assert(range_app.execute(parseCommand("range normal")).ok);
    assert(range_pwm.output_count == outputs_before_reboot && range_store.save_count == 8);

    Pwm invalid_range_pwm;
    ServoController invalid_range(invalid_range_pwm, range_clock, 1000, 2000, 0,
                                  nullptr, 0, 400, 2600);
    assert(invalid_range.begin());
    assert(invalid_range.setExtendedRange(true) == ServoResult::OutOfRange);
    assert(!invalid_range.extendedRange() && invalid_range_pwm.output_count == 0);
    range_pwm.fail_off = true;
    assert(!range_app.execute(parseCommand("off")).ok);
    assert(!range_app.execute(parseCommand("range extended")).ok);
    assert(!range_app.snapshot().extended_range);
}
