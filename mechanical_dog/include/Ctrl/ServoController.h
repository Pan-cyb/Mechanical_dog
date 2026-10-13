#pragma once

#include <cstddef>
#include <cstdint>
#include "Interface/IClock.h"
#include "Interface/IServoPwm.h"

namespace dog {

enum class ServoResult {
    Ok, NotReady, NotArmed, NotActive, ModeConflict, OutOfRange, HardwareFault
};

class ServoController {
public:
    ServoController(IServoPwm& pwm, IClock& clock, std::uint16_t min_us,
                    std::uint16_t max_us, std::uint32_t timeout_ms,
                    const std::uint8_t* allowed_channels = nullptr,
                    std::size_t allowed_count = 0,
                    std::uint16_t extended_min_us = 0,
                    std::uint16_t extended_max_us = 0);

    bool begin();
    bool arm(); // Does not move any servo.
    ServoResult setExtendedRange(bool enabled); // Disarmed only, never sends PWM.
    bool extendedRange() const { return extended_range_; }
    std::uint16_t minPulseUs() const { return extended_range_ ? extended_min_us_ : min_us_; }
    std::uint16_t maxPulseUs() const { return extended_range_ ? extended_max_us_ : max_us_; }
    bool keepAlive();
    // Explicit eight-output bench calibration. Validates the whole set
    // before any PWM; not a gait mode and never started on boot.
    ServoResult startCalibration(const std::uint8_t* channels,
                                 const std::uint16_t* pulse_us, std::size_t count);
    ServoResult calibrationPulse(std::uint8_t channel, std::uint16_t pulse_us);
    ServoResult pulse(std::uint8_t channel, std::uint16_t pulse_us);
    // Coordinated motion validates the complete set before the first write.
    // PCA9685 channel writes are sequential (not electrically atomic).  A
    // failed member causes a best-effort global shutdown and latches fault.
    ServoResult coordinatedFrame(const std::uint8_t* channels,
                                 const std::uint16_t* pulse_us,
                                 std::size_t count);
    ServoResult offChannel(std::uint8_t channel); // Keeps ARM; only this channel.
    bool off();
    void tick();

    bool ready() const { return ready_; }
    bool armed() const { return armed_; }
    bool faulted() const { return faulted_; }
    bool manualHold() const { return timeout_ms_ == 0; }
    bool calibrating() const { return calibrating_; }
    bool coordinated() const { return coordinated_; }
    std::uint16_t calibrationMask() const { return calibration_mask_; }
    bool allowed(std::uint8_t channel) const;
    int activeChannel() const { return active_channel_; }
    std::uint16_t activePulseUs() const { return active_pulse_us_; }
    std::uint32_t remainingMs() const;

private:
    IServoPwm& pwm_;
    IClock& clock_;
    const std::uint16_t min_us_;
    const std::uint16_t max_us_;
    const std::uint16_t extended_min_us_;
    const std::uint16_t extended_max_us_;
    bool extended_range_ = false;
    const std::uint32_t timeout_ms_;
    const std::uint8_t* const allowed_channels_;
    const std::size_t allowed_count_;
    std::uint32_t last_command_ms_ = 0;
    int active_channel_ = -1;
    std::uint16_t active_pulse_us_ = 0;
    std::uint16_t calibration_mask_ = 0;
    bool calibrating_ = false;
    bool coordinated_ = false;
    bool ready_ = false;
    bool armed_ = false;
    bool faulted_ = false;
};

} // namespace dog
