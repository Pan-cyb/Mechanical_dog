#include "Driver/HxyIcm42688p.h"

namespace dog {
namespace {
constexpr std::uint8_t kWhoAmI = 0x01;
constexpr std::uint8_t kExpectedId = 0x6A;
constexpr std::uint8_t kDataStart = 0x0C;
constexpr float kGravityMps2 = 9.80665f;
constexpr float kRadPerDegree = 0.017453292519943295f;

std::int16_t signedBigEndian(const std::uint8_t* bytes) {
    const std::uint16_t value = static_cast<std::uint16_t>(bytes[0] << 8) | bytes[1];
    return value < 0x8000 ? static_cast<std::int16_t>(value)
                          : static_cast<std::int16_t>(static_cast<std::int32_t>(value) - 65536);
}
} // namespace

HxyIcm42688p::HxyIcm42688p(IRegisterBus& bus, IClock& clock, std::uint8_t address)
    : bus_(bus), clock_(clock), address_(address) {}

bool HxyIcm42688p::writeReg(std::uint8_t reg, std::uint8_t value) {
    return bus_.write(address_, reg, &value, 1);
}

bool HxyIcm42688p::begin() {
    ready_ = false;
    last_who_am_i_ = 0;
    if (!bus_.read(address_, kWhoAmI, &last_who_am_i_, 1) ||
        last_who_am_i_ != kExpectedId) return false;

    // HXY datasheet pp. 33-36: high-performance 100 Hz, +/-4 g, +/-1000 dps.
    if (!writeReg(0x7D, 0x0E)) return false;
    clock_.delayMs(10);
    if (!writeReg(0x40, 0xA8) || !writeReg(0x41, 0x01) ||
        !writeReg(0x42, 0xA8) || !writeReg(0x43, 0x01)) return false;
    clock_.delayMs(1);
    ready_ = true;
    return true;
}

bool HxyIcm42688p::read(ImuSample& sample) {
    if (!ready_) return false;
    std::uint8_t raw[12] = {};
    if (!bus_.read(address_, kDataStart, raw, sizeof(raw))) return false;

    ImuSample next{};
    for (int axis = 0; axis < 3; ++axis) {
        next.accel_mps2[axis] =
            signedBigEndian(raw + axis * 2) * (4.0f * kGravityMps2 / 32768.0f);
        next.gyro_radps[axis] =
            signedBigEndian(raw + 6 + axis * 2) *
            (1000.0f * kRadPerDegree / 32768.0f);
    }
    next.timestamp_ms = clock_.millis();
    sample = next; // Never publish partial or stale values on bus error.
    return true;
}

bool HxyIcm42688p::readDiagnostics(HxyDiagnostics& diagnostics) {
    if (!ready_) return false;
    HxyDiagnostics next{};
    // Individual register reads do not depend on COM_CFG Addr_Auto. In
    // particular, the readback reveals whether ACC_RANGE retained 0x01.
    if (!bus_.read(address_, 0x01, &next.who_am_i, 1) ||
        !bus_.read(address_, 0x05, &next.com_cfg, 1) ||
        !bus_.read(address_, 0x08, &next.filter_cfg, 1) ||
        !bus_.read(address_, 0x0B, &next.data_stat, 1) ||
        !bus_.read(address_, 0x40, &next.acc_conf, 1) ||
        !bus_.read(address_, 0x41, &next.acc_range, 1) ||
        !bus_.read(address_, 0x42, &next.gyro_conf, 1) ||
        !bus_.read(address_, 0x43, &next.gyro_range, 1) ||
        !bus_.read(address_, 0x7D, &next.power_ctrl, 1) ||
        !bus_.read(address_, kDataStart, next.raw_bytes, sizeof(next.raw_bytes)))
        return false;
    for (int axis = 0; axis < 3; ++axis) {
        next.accel_raw[axis] = signedBigEndian(next.raw_bytes + 2 * axis);
        next.gyro_raw[axis] = signedBigEndian(next.raw_bytes + 6 + 2 * axis);
    }
    diagnostics = next;
    return true;
}

bool HxyIcm42688p::runAccelRangeTest(HxyRangeTest& result) {
    HxyRangeTest next{};
    constexpr std::uint8_t kRangeRegister = 0x41;
    constexpr std::uint8_t kRanges[3] = {0x01, 0x00, 0x02};
    if (!ready_) {
        next.error = HxyRangeTestError::NotReady;
        result = next;
        return false;
    }
    if (!bus_.read(address_, kWhoAmI, &next.identity, 1) ||
        next.identity != kExpectedId) {
        next.error = HxyRangeTestError::Identity;
        result = next;
        return false;
    }
    if (!bus_.read(address_, kRangeRegister, &next.original_range, 1)) {
        next.error = HxyRangeTestError::InitialRangeRead;
        result = next;
        return false;
    }
    next.original_range_known = true;
    // Only exercise the exact configuration established by begin(). Never
    // change an unexpected live setting or assume that a register readback
    // proves the physical output sensitivity.
    if (next.original_range != 0x01) {
        next.error = HxyRangeTestError::UnexpectedInitialRange;
        result = next;
        return false;
    }
    if (!bus_.read(address_, 0x40, &next.acc_conf, 1) ||
        !bus_.read(address_, 0x7D, &next.power_ctrl, 1)) {
        next.error = HxyRangeTestError::ConfigRead;
        result = next;
        return false;
    }
    if (next.acc_conf != 0xA8 || next.power_ctrl != 0x0E) {
        next.error = HxyRangeTestError::UnexpectedConfig;
        result = next;
        return false;
    }

    auto capture = [&](std::uint8_t stage, std::uint8_t requested) {
        HxyRangeStep& step = next.steps[stage];
        step.requested_range = requested;
        step.started = true;
        ++next.steps_started;
        // Field trace showed two old/duplicate frames after a range change;
        // the 100 Hz output needs more than the datasheet's 1 ms register
        // settle time before the new scale reaches the data path.
        clock_.delayMs(stage == 0 ? 20 : 100);
        if (!bus_.read(address_, kRangeRegister, &step.observed_range, 1)) {
            next.error = HxyRangeTestError::RangeRead;
            return false;
        }
        step.range_read_ok = true;
        if (step.observed_range != step.requested_range) {
            next.error = HxyRangeTestError::RangeMismatch;
            return false;
        }
        for (std::uint8_t frame = 0; frame < 3; ++frame) {
            if (frame != 0) clock_.delayMs(20);
            bool fresh = false;
            for (std::uint8_t attempt = 0; attempt < 8; ++attempt) {
                if (!bus_.read(address_, 0x0B, &step.status[frame], 1)) {
                    next.error = HxyRangeTestError::StatusRead;
                    return false;
                }
                if ((step.status[frame] & 0x01U) != 0) {
                    fresh = true;
                    break;
                }
                clock_.delayMs(5);
            }
            if (!fresh) {
                next.error = HxyRangeTestError::DataNotReady;
                return false;
            }
            std::uint8_t raw[12] = {};
            if (!bus_.read(address_, kDataStart, raw, sizeof(raw))) {
                next.error = HxyRangeTestError::SampleRead;
                return false;
            }
            for (int axis = 0; axis < 3; ++axis)
                step.accel_raw[frame][axis] = signedBigEndian(raw + axis * 2);
            ++step.frames_captured;
        }
        return true;
    };

    bool changed_or_attempted = false;
    for (std::uint8_t stage = 0; stage < 3; ++stage) {
        if (stage != 0) {
            // A failed I2C write may still have reached the sensor; restore
            // even when the bus reports failure.
            changed_or_attempted = true;
            if (!writeReg(kRangeRegister, kRanges[stage])) {
                next.error = HxyRangeTestError::RangeWrite;
                break;
            }
        }
        if (!capture(stage, kRanges[stage])) break;
    }

    if (changed_or_attempted) {
        next.restoration_attempted = true;
        const bool wrote = writeReg(kRangeRegister, next.original_range);
        clock_.delayMs(1); // Datasheet minimum settle before readback.
        const bool read = bus_.read(address_, kRangeRegister,
                                    &next.restored_readback, 1);
        next.restoration_verified = wrote && read &&
                                    next.restored_readback == next.original_range;
        if (!next.restoration_verified) {
            ready_ = false; // Do not continue publishing samples at an unknown scale.
            if (next.error == HxyRangeTestError::None)
                next.error = !wrote ? HxyRangeTestError::RestoreWrite
                           : !read ? HxyRangeTestError::RestoreRead
                           : HxyRangeTestError::RestoreMismatch;
        } else {
            const HxyRangeTestError earlier_error = next.error;
            const bool post_restore_captured = capture(3, next.original_range);
            if (earlier_error != HxyRangeTestError::None) next.error = earlier_error;
            if (post_restore_captured && next.steps[0].frames_captured == 3) {
                // Stationary-board comparison, not a sensitivity calibration.
                // Movement during the test makes this inconclusive.
                std::int32_t peak = 0;
                for (int axis = 0; axis < 3; ++axis) {
                    std::int32_t baseline = 0;
                    for (int frame = 0; frame < 3; ++frame)
                        baseline += next.steps[0].accel_raw[frame][axis];
                    const std::int32_t absolute = baseline < 0 ? -baseline : baseline;
                    if (absolute > peak) peak = absolute;
                }
                const std::int32_t tolerance = 1200 + peak / 10; // ~400 + 10% per frame.
                next.data_returned_to_baseline = true;
                for (int axis = 0; axis < 3; ++axis) {
                    std::int32_t difference = 0;
                    for (int frame = 0; frame < 3; ++frame)
                        difference += static_cast<std::int32_t>(next.steps[3].accel_raw[frame][axis]) -
                                      next.steps[0].accel_raw[frame][axis];
                    if (difference > tolerance || difference < -tolerance)
                        next.data_returned_to_baseline = false;
                }
            }
        }
    }
    result = next; // Publish partial evidence and restoration status on error.
    return next.error == HxyRangeTestError::None && next.restoration_verified;
}

} // namespace dog
