#include "Ctrl/RelativeServoMap.h"
#include <algorithm>
#include <cmath>

namespace dog {
bool relativeServoBounds(float center, float lo, float hi, float& lower, float& upper) {
    if (!std::isfinite(center) || !std::isfinite(lo) || !std::isfinite(hi) ||
        center < 0 || center > 180 || lo > 0 || hi < 0 || lo > hi) return false;
    lower = std::max(lo, -center);
    upper = std::min(hi, 180.0f - center);
    return true;
}

bool relativeServoPulse(float center, float offset, float lo, float hi,
                        std::uint16_t& pulse) {
    float lower = 0, upper = 0;
    if (!std::isfinite(offset) || !relativeServoBounds(center, lo, hi, lower, upper) ||
        offset < lower || offset > upper) return false;
    // Same integer encoding as existing NVS centers, with fractional offsets.
    pulse = static_cast<std::uint16_t>(std::lround(500.0 +
                              2000.0 * (static_cast<double>(center) + offset) / 180.0));
    return true;
}
} // namespace dog
