#include "Ctrl/ChannelCalibration.h"

namespace dog {

ChannelCalibration::ChannelCalibration(ICalibrationStore& store, ServoController& servos,
                                       const std::uint8_t* channels, std::size_t count,
                                       std::uint16_t min_us, std::uint16_t max_us)
    : store_(store), servos_(servos), channels_(channels), count_(count),
      min_us_(min_us), max_us_(max_us) {}

std::uint16_t ChannelCalibration::pulseFor(std::uint8_t degrees) {
    // Preserve the existing on-board NVS centre encoding (90 deg = 1500 us).
    // Changing this now would reinterpret the user's eight saved centres.
    return static_cast<std::uint16_t>(500U + (2000U * degrees + 90U) / 180U);
}

bool ChannelCalibration::selected(std::uint8_t channel) const {
    if (channel >= 16 || channels_ == nullptr) return false;
    for (std::size_t i = 0; i < count_; ++i)
        if (channels_[i] == channel) return true;
    return false;
}

void ChannelCalibration::begin() {
    for (std::uint8_t& center : centers_deg_) center = 90;
    for (std::size_t i = 0; i < count_; ++i) {
        const std::uint8_t ch = channels_[i];
        if (ch >= 16) continue;
        std::uint8_t stored = 0;
        if (store_.load(ch, stored) && stored <= 180 &&
            pulseFor(stored) >= min_us_ && pulseFor(stored) <= max_us_)
            centers_deg_[ch] = stored;
    }
}

std::uint8_t ChannelCalibration::centerDeg(std::uint8_t channel) const {
    return selected(channel) ? centers_deg_[channel] : 0;
}

std::uint16_t ChannelCalibration::centerUs(std::uint8_t channel) const {
    return selected(channel) ? pulseFor(centers_deg_[channel]) : 0;
}

ServoResult ChannelCalibration::startAll() {
    if (channels_ == nullptr || count_ != 8) return ServoResult::OutOfRange;
    std::uint16_t pulses[8] = {};
    for (std::size_t i = 0; i < count_; ++i) {
        const std::uint8_t ch = channels_[i];
        if (!selected(ch)) return ServoResult::OutOfRange;
        pulses[i] = centerUs(ch);
        if (pulses[i] < min_us_ || pulses[i] > max_us_)
            return ServoResult::OutOfRange;
    }
    return servos_.startCalibration(channels_, pulses, count_);
}

CalibrationResult ChannelCalibration::nudge(std::uint8_t channel, int delta_deg) {
    if (!selected(channel)) return CalibrationResult::InvalidChannel;
    if (delta_deg != 1 && delta_deg != -1) return CalibrationResult::InvalidStep;
    // Avoid a jump from an unrelated manual slider pulse back to the draft
    // center when the user first presses +/-1 degree.
    if (servos_.armed() && servos_.activeChannel() == channel &&
        servos_.activePulseUs() != centerUs(channel))
        return CalibrationResult::ManualPulseActive;
    const int next = static_cast<int>(centers_deg_[channel]) + delta_deg;
    if (next < 0 || next > 180) return CalibrationResult::OutOfRange;
    const std::uint16_t pulse = pulseFor(static_cast<std::uint8_t>(next));
    if (pulse < min_us_ || pulse > max_us_) return CalibrationResult::OutOfRange;
    switch (servos_.calibrating()
                ? servos_.calibrationPulse(channel, pulse)
                : servos_.pulse(channel, pulse)) {
    case ServoResult::Ok:
        centers_deg_[channel] = static_cast<std::uint8_t>(next);
        return CalibrationResult::Ok;
    case ServoResult::NotArmed: return CalibrationResult::NotArmed;
    case ServoResult::OutOfRange: return CalibrationResult::OutOfRange;
    default: return CalibrationResult::HardwareFault;
    }
}

CalibrationResult ChannelCalibration::save(std::uint8_t channel) {
    if (!selected(channel)) return CalibrationResult::InvalidChannel;
    if (servos_.faulted() || !servos_.ready()) return CalibrationResult::HardwareFault;
    if (servos_.armed()) return CalibrationResult::DisarmBeforeSave;
    if (!store_.save(channel, centers_deg_[channel]))
        return CalibrationResult::StorageFault;
    return CalibrationResult::Ok;
}

CalibrationResult ChannelCalibration::saveAll(std::size_t& saved_count) {
    saved_count = 0;
    if (servos_.faulted() || !servos_.ready()) return CalibrationResult::HardwareFault;
    if (servos_.armed()) return CalibrationResult::DisarmBeforeSave;
    if (channels_ == nullptr || count_ != 8) return CalibrationResult::InvalidChannel;
    for (std::size_t i = 0; i < count_; ++i) {
        const std::uint8_t ch = channels_[i];
        if (!selected(ch) || !store_.save(ch, centers_deg_[ch]))
            return CalibrationResult::StorageFault; // Earlier keys may persist.
        ++saved_count;
    }
    return CalibrationResult::Ok;
}

} // namespace dog
