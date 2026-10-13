#include "Ctrl/V10LegIk.h"

#include <cmath>

namespace dog {
namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadPerDegree = kPi / 180.0f;
constexpr float kDegPerRad = 180.0f / kPi;
constexpr float kSingularityMarginMm = 0.0001f;
}

bool V10LegIk::solve(float x_mm, float y_down_mm, KneeFold fold,
                     V10LegAngles& out) const {
    const float a = geometry_.upper_mm;
    const float b = geometry_.lower_mm;
    if (!std::isfinite(x_mm) || !std::isfinite(y_down_mm) ||
        !std::isfinite(a) || !std::isfinite(b) || a <= 0.0f || b <= 0.0f ||
        (fold != KneeFold::Clockwise && fold != KneeFold::CounterClockwise))
        return false;

    const float distance = std::hypot(x_mm, y_down_mm);
    if (!std::isfinite(distance) ||
        distance <= std::fabs(a - b) + kSingularityMarginMm ||
        distance >= a + b - kSingularityMarginMm) return false;
    const float c = (distance * distance - a * a - b * b) / (2.0f * a * b);
    if (!std::isfinite(c) || c < -1.0f || c > 1.0f) return false;
    const float signed_knee = (fold == KneeFold::Clockwise ? 1.0f : -1.0f) *
                              std::acos(c);
    const float hip = std::atan2(y_down_mm, x_mm) -
                      std::atan2(b * std::sin(signed_knee),
                                 a + b * std::cos(signed_knee));
    if (!std::isfinite(hip) || !std::isfinite(signed_knee)) return false;
    V10LegAngles next{};
    next.hip_deg = hip * kDegPerRad;
    next.knee_turn_deg = signed_knee * kDegPerRad;
    out = next;
    return true;
}

bool V10LegIk::forward(const V10LegAngles& angles, float& x_mm,
                       float& y_down_mm) const {
    const float a = geometry_.upper_mm;
    const float b = geometry_.lower_mm;
    if (!std::isfinite(a) || !std::isfinite(b) || a <= 0.0f || b <= 0.0f ||
        !std::isfinite(angles.hip_deg) ||
        !std::isfinite(angles.knee_turn_deg)) return false;
    const float hip = angles.hip_deg * kRadPerDegree;
    const float shank = (angles.hip_deg + angles.knee_turn_deg) * kRadPerDegree;
    const float next_x = a * std::cos(hip) + b * std::cos(shank);
    const float next_y = a * std::sin(hip) + b * std::sin(shank);
    if (!std::isfinite(next_x) || !std::isfinite(next_y)) return false;
    x_mm = next_x;
    y_down_mm = next_y;
    return true;
}

} // namespace dog
