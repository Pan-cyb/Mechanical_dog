#include "Ctrl/ServoController.h"

namespace dog {

ServoController::ServoController(IServoPwm& pwm, IClock& clock,
                                 std::uint16_t min_us, std::uint16_t max_us,
                                 std::uint32_t timeout_ms,
                                 const std::uint8_t* allowed_channels,
                                 std::size_t allowed_count,
                                 std::uint16_t extended_min_us,
                                 std::uint16_t extended_max_us)
    : pwm_(pwm), clock_(clock), min_us_(min_us), max_us_(max_us),
      extended_min_us_(extended_min_us), extended_max_us_(extended_max_us),
      timeout_ms_(timeout_ms), allowed_channels_(allowed_channels),
      allowed_count_(allowed_count) {}

bool ServoController::allowed(std::uint8_t channel) const {
    if (channel >= 16) return false;
    if (allowed_channels_ == nullptr || allowed_count_ == 0) return true;
    for (std::size_t i = 0; i < allowed_count_; ++i)
        if (allowed_channels_[i] == channel) return true;
    return false;
}

std::uint32_t ServoController::remainingMs() const {
    if (!armed_ || manualHold()) return 0;
    const std::uint32_t elapsed = clock_.millis() - last_command_ms_;
    return elapsed >= timeout_ms_ ? 0 : timeout_ms_ - elapsed;
}

bool ServoController::begin() {
    extended_range_ = false; // Never restore an unlock from NVS or a reboot.
    armed_ = false;
    active_channel_ = -1;
    active_pulse_us_ = 0;
    calibrating_ = false;
    coordinated_ = false;
    calibration_mask_ = 0;
    ready_ = pwm_.begin();
    if (ready_) ready_ = pwm_.allOff();
    if (!ready_) {
        faulted_ = true;
        (void)pwm_.allOff(); // Best effort; no physical OE control on this PCB.
    }
    return ready_;
}

bool ServoController::arm() {
    tick();
    if (!ready_ || faulted_ || armed_) return false;
    last_command_ms_ = clock_.millis();
    armed_ = true;
    return true;
}

ServoResult ServoController::setExtendedRange(bool enabled) {
    tick();
    if (faulted_) return ServoResult::HardwareFault;
    if (!ready_) return ServoResult::NotReady;
    if (armed_ || calibrating_ || coordinated_ || active_channel_ >= 0)
        return ServoResult::ModeConflict;
    if (enabled && (extended_min_us_ < 500 || extended_max_us_ > 2500 ||
        extended_min_us_ > min_us_ || extended_max_us_ < max_us_ ||
        extended_min_us_ >= extended_max_us_)) return ServoResult::OutOfRange;
    extended_range_ = enabled;
    return ServoResult::Ok;
}

bool ServoController::keepAlive() {
    tick();
    if (!ready_ || !armed_ || faulted_) return false;
    last_command_ms_ = clock_.millis();
    return true;
}

ServoResult ServoController::startCalibration(const std::uint8_t* channels,
                                               const std::uint16_t* pulse_us,
                                               std::size_t count) {
    tick();
    if (faulted_) return ServoResult::HardwareFault;
    if (!ready_) return ServoResult::NotReady;
    if (armed_ || calibrating_ || coordinated_ || active_channel_ >= 0)
        return ServoResult::ModeConflict;
    if (channels == nullptr || pulse_us == nullptr || count == 0 || count > 16 ||
        (allowed_channels_ != nullptr && allowed_count_ != 0 &&
         count != allowed_count_)) return ServoResult::OutOfRange;
    std::uint16_t mask = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint8_t channel = channels[i];
        if (!allowed(channel) || (mask & (1U << channel)) != 0 ||
            pulse_us[i] < minPulseUs() || pulse_us[i] > maxPulseUs())
            return ServoResult::OutOfRange;
        mask = static_cast<std::uint16_t>(mask | (1U << channel));
    }
    // Require precisely the configured physical pool, not an arbitrary
    // subset. No gait or logical joint mapping is used by this mode.
    if (allowed_channels_ != nullptr && allowed_count_ != 0) {
        for (std::size_t i = 0; i < allowed_count_; ++i)
            if (allowed_channels_[i] >= 16 ||
                (mask & (1U << allowed_channels_[i])) == 0)
                return ServoResult::OutOfRange;
    }
    if (!pwm_.allOff()) {
        (void)off();
        faulted_ = true;
        return ServoResult::HardwareFault;
    }
    for (std::size_t i = 0; i < count; ++i) {
        if (!pwm_.setPulseUs(channels[i], pulse_us[i])) {
            (void)off(); // Best effort; partial PWM may persist on I2C failure.
            faulted_ = true;
            return ServoResult::HardwareFault;
        }
    }
    calibration_mask_ = mask;
    calibrating_ = true;
    armed_ = true;
    active_channel_ = -1;
    active_pulse_us_ = 0;
    last_command_ms_ = clock_.millis();
    return ServoResult::Ok;
}

ServoResult ServoController::calibrationPulse(std::uint8_t channel,
                                               std::uint16_t pulse_us) {
    tick();
    if (faulted_) return ServoResult::HardwareFault;
    if (!ready_) return ServoResult::NotReady;
    if (!calibrating_ || !armed_) return ServoResult::ModeConflict;
    if (channel >= 16 || (calibration_mask_ & (1U << channel)) == 0 ||
        pulse_us < minPulseUs() || pulse_us > maxPulseUs())
        return ServoResult::OutOfRange;
    if (!pwm_.setPulseUs(channel, pulse_us)) {
        (void)off();
        faulted_ = true;
        return ServoResult::HardwareFault;
    }
    last_command_ms_ = clock_.millis();
    return ServoResult::Ok;
}

ServoResult ServoController::pulse(std::uint8_t channel, std::uint16_t pulse_us) {
    tick();
    if (faulted_) return ServoResult::HardwareFault;
    if (!ready_) return ServoResult::NotReady;
    if (!armed_) return ServoResult::NotArmed;
    if (calibrating_ || coordinated_) return ServoResult::ModeConflict;
    if (!allowed(channel) || pulse_us < minPulseUs() || pulse_us > maxPulseUs())
        return ServoResult::OutOfRange;
    if (active_channel_ >= 0 && active_channel_ != channel && !pwm_.allOff()) {
        (void)off();
        faulted_ = true;
        return ServoResult::HardwareFault;
    }
    if (!pwm_.setPulseUs(channel, pulse_us)) {
        (void)off();
        faulted_ = true;
        return ServoResult::HardwareFault;
    }
    active_channel_ = channel;
    active_pulse_us_ = pulse_us;
    last_command_ms_ = clock_.millis();
    return ServoResult::Ok;
}

ServoResult ServoController::coordinatedFrame(const std::uint8_t* channels,
                                               const std::uint16_t* pulse_us,
                                               std::size_t count) {
    tick();
    if (faulted_) return ServoResult::HardwareFault;
    if (!ready_) return ServoResult::NotReady;
    if (!armed_) return ServoResult::NotArmed;
    if (calibrating_ || active_channel_ >= 0) return ServoResult::ModeConflict;
    if (!channels || !pulse_us || count == 0 || count > 16 ||
        (allowed_channels_ && allowed_count_ && count != allowed_count_))
        return ServoResult::OutOfRange;
    std::uint16_t mask = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint8_t ch = channels[i];
        if (!allowed(ch) || (mask & (1U << ch)) ||
            pulse_us[i] < minPulseUs() || pulse_us[i] > maxPulseUs())
            return ServoResult::OutOfRange;
        mask = static_cast<std::uint16_t>(mask | (1U << ch));
    }
    if (allowed_channels_ && allowed_count_) {
        for (std::size_t i = 0; i < allowed_count_; ++i)
            if (allowed_channels_[i] >= 16 ||
                !(mask & (1U << allowed_channels_[i])))
                return ServoResult::OutOfRange;
    }
    // No allOff here: valid gait frames replace each other without a gap.
    for (std::size_t i = 0; i < count; ++i) {
        if (!pwm_.setPulseUs(channels[i], pulse_us[i])) {
            armed_ = false;
            coordinated_ = false;
            (void)pwm_.allOff();
            faulted_ = true;
            return ServoResult::HardwareFault;
        }
    }
    coordinated_ = true;
    last_command_ms_ = clock_.millis();
    return ServoResult::Ok;
}

ServoResult ServoController::offChannel(std::uint8_t channel) {
    tick();
    if (faulted_) return ServoResult::HardwareFault;
    if (!ready_) return ServoResult::NotReady;
    if (calibrating_ || coordinated_) return ServoResult::ModeConflict;
    if (!allowed(channel)) return ServoResult::OutOfRange;
    // This bench controller permits only one active channel. Refuse a wrong
    // selection instead of reporting success while another channel continues.
    if (active_channel_ != channel) return ServoResult::NotActive;
    if (!pwm_.offChannel(channel)) {
        (void)off(); // Best-effort global shutdown if the bus is unreliable.
        faulted_ = true; // Do not claim that the requested channel is off.
        return ServoResult::HardwareFault;
    }
    active_channel_ = -1;
    active_pulse_us_ = 0;
    return ServoResult::Ok;
}

bool ServoController::off() {
    armed_ = false;
    active_channel_ = -1;
    active_pulse_us_ = 0;
    calibrating_ = false;
    coordinated_ = false;
    calibration_mask_ = 0;
    const bool ok = pwm_.allOff();
    if (!ok) faulted_ = true;
    return ok;
}

void ServoController::tick() {
    // Unsigned subtraction is rollover-safe for ordinary short intervals.
    if (armed_ && !manualHold() &&
        static_cast<std::uint32_t>(clock_.millis() - last_command_ms_) >=
                      timeout_ms_) (void)off();
}

} // namespace dog
