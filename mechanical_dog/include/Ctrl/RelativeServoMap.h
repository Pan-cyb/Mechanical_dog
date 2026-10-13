#pragma once
#include <cstdint>

namespace dog {
struct RelativeServoLimit {
    std::uint8_t channel;
    const char* joint; // Configuration-owned label, never network input.
    float min_offset_deg;
    float max_offset_deg;
    bool shank; // Hip electrical envelope is not a measured mechanical limit.
};

// Servo command degrees, NOT knee/linkage angles. Neutral is explicit and
// can be any calibrated 0..180 value. Reject rather than clamp requests.
bool relativeServoBounds(float center_deg, float configured_min, float configured_max,
                         float& min_offset, float& max_offset);
bool relativeServoPulse(float center_deg, float offset_deg, float configured_min,
                        float configured_max, std::uint16_t& pulse_us);
} // namespace dog
