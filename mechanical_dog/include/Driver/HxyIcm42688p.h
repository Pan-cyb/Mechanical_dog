#pragma once

#include <cstdint>
#include "Interface/IClock.h"
#include "Interface/IImu.h"
#include "Interface/IRegisterBus.h"

namespace dog {

struct HxyDiagnostics {
    std::uint8_t who_am_i = 0;
    std::uint8_t com_cfg = 0;
    std::uint8_t filter_cfg = 0;
    std::uint8_t data_stat = 0;
    std::uint8_t acc_conf = 0;
    std::uint8_t acc_range = 0;
    std::uint8_t gyro_conf = 0;
    std::uint8_t gyro_range = 0;
    std::uint8_t power_ctrl = 0;
    std::uint8_t raw_bytes[12] = {};
    std::int16_t accel_raw[3] = {};
    std::int16_t gyro_raw[3] = {};
};

enum class HxyRangeTestError : std::uint8_t {
    None, NotReady, Identity, InitialRangeRead, UnexpectedInitialRange,
    ConfigRead, UnexpectedConfig,
    RangeWrite, RangeRead, RangeMismatch, StatusRead, DataNotReady, SampleRead,
    RestoreWrite, RestoreRead, RestoreMismatch
};

struct HxyRangeStep {
    std::uint8_t requested_range = 0;
    std::uint8_t observed_range = 0;
    std::uint8_t status[3] = {};
    std::int16_t accel_raw[3][3] = {};
    std::uint8_t frames_captured = 0;
    bool started = false;
    bool range_read_ok = false;
};

struct HxyRangeTest {
    std::uint8_t identity = 0;
    std::uint8_t original_range = 0;
    std::uint8_t acc_conf = 0;
    std::uint8_t power_ctrl = 0;
    HxyRangeStep steps[4] = {}; // baseline 4g, temporary 2g/8g, restored 4g
    std::uint8_t steps_started = 0;
    bool original_range_known = false;
    bool restoration_attempted = false;
    bool restoration_verified = false;
    std::uint8_t restored_readback = 0;
    bool data_returned_to_baseline = false; // Advisory; motion makes this inconclusive.
    HxyRangeTestError error = HxyRangeTestError::None;
};

// HXY-marked domestic ICM-42688P used by the user's M0 project. NOT the TDK
// register map (which uses 0x68/0x69 and WHO_AM_I at 0x75).
class HxyIcm42688p final : public IImu {
public:
    HxyIcm42688p(IRegisterBus& bus, IClock& clock, std::uint8_t address);
    bool begin() override;
    bool read(ImuSample& sample) override;
    // Read-only board diagnostic: configuration, status and one raw frame.
    // Does not change any register or publish a sensor sample.
    bool readDiagnostics(HxyDiagnostics& diagnostics);
    // Bench-only experiment; caller must verify all PWM channels are OFF and
    // disarmed. Writes only ACC_RANGE (0x41), attempts restoration on every
    // post-write path, and invalidates this driver if restoration is unverified.
    bool runAccelRangeTest(HxyRangeTest& result);
    std::uint8_t lastWhoAmI() const { return last_who_am_i_; }

private:
    bool writeReg(std::uint8_t reg, std::uint8_t value);
    IRegisterBus& bus_;
    IClock& clock_;
    const std::uint8_t address_;
    std::uint8_t last_who_am_i_ = 0;
    bool ready_ = false;
};

} // namespace dog
