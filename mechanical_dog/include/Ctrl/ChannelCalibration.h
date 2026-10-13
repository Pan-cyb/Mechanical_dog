#pragma once

#include <cstddef>
#include <cstdint>
#include "Ctrl/ServoController.h"
#include "Interface/ICalibrationStore.h"

namespace dog {

enum class CalibrationResult {
    Ok, InvalidChannel, InvalidStep, OutOfRange, NotArmed,
    HardwareFault, ManualPulseActive, DisarmBeforeSave, StorageFault
};

// Bench-only center per physical output; does not assign legs or joints.
class ChannelCalibration {
public:
    ChannelCalibration(ICalibrationStore& store, ServoController& servos,
                       const std::uint8_t* channels, std::size_t count,
                       std::uint16_t min_us, std::uint16_t max_us);
    void begin(); // Loads only. Never sends a servo pulse.
    std::size_t count() const { return count_; }
    std::uint8_t channelAt(std::size_t index) const {
        return index < count_ ? channels_[index] : 0xFF;
    }
    bool selected(std::uint8_t channel) const;
    std::uint8_t centerDeg(std::uint8_t channel) const;
    std::uint16_t centerUs(std::uint8_t channel) const;
    ServoResult startAll(); // Explicitly holds the selected eight at saved centers.
    CalibrationResult nudge(std::uint8_t channel, int delta_deg);
    CalibrationResult save(std::uint8_t channel);
    CalibrationResult saveAll(std::size_t& saved_count); // Disarmed only; partial on NVS fault.

private:
    static std::uint16_t pulseFor(std::uint8_t degrees);
    ICalibrationStore& store_;
    ServoController& servos_;
    const std::uint8_t* const channels_;
    const std::size_t count_;
    const std::uint16_t min_us_;
    const std::uint16_t max_us_;
    std::uint8_t centers_deg_[16] = {};
};

} // namespace dog
