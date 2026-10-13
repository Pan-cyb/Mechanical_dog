#include "Ctrl/V10ShankLinearMap.h"

#include <cmath>

namespace dog {
namespace {
bool finiteNonnegative(float value) {
    return std::isfinite(value) && value >= 0.0f;
}

float interpolate(float input, float in_a, float in_b,
                  float out_a, float out_b) {
    return static_cast<float>(static_cast<double>(out_a) +
        (static_cast<double>(input) - in_a) *
        (static_cast<double>(out_b) - out_a) /
        (static_cast<double>(in_b) - in_a));
}
}

bool V10ShankLinearMap::valid() const {
    if (!points_ || point_count_ < 2 ||
        points_[0].u_deg != 0.0f || points_[0].q_deg != 0.0f)
        return false;
    for (std::size_t i = 0; i < point_count_; ++i) {
        if (!finiteNonnegative(points_[i].u_deg) ||
            !finiteNonnegative(points_[i].q_deg)) return false;
        if (i > 0 && (points_[i].u_deg <= points_[i - 1].u_deg ||
                      points_[i].q_deg <= points_[i - 1].q_deg)) return false;
    }
    return true;
}

bool V10ShankLinearMap::qFromU(float u, float& q) const {
    if (!valid() || !finiteNonnegative(u) || u > maxU()) return false;
    for (std::size_t i = 1; i < point_count_; ++i) {
        if (u <= points_[i].u_deg) {
            const float value = interpolate(u, points_[i - 1].u_deg,
                                            points_[i].u_deg,
                                            points_[i - 1].q_deg,
                                            points_[i].q_deg);
            if (!std::isfinite(value)) return false;
            q = value;
            return true;
        }
    }
    return false;
}

bool V10ShankLinearMap::uFromQ(float q, float& u) const {
    if (!valid() || !finiteNonnegative(q) || q > maxQ()) return false;
    for (std::size_t i = 1; i < point_count_; ++i) {
        if (q <= points_[i].q_deg) {
            const float value = interpolate(q, points_[i - 1].q_deg,
                                            points_[i].q_deg,
                                            points_[i - 1].u_deg,
                                            points_[i].u_deg);
            if (!std::isfinite(value)) return false;
            u = value;
            return true;
        }
    }
    return false;
}

float V10ShankLinearMap::maxU() const {
    return valid() ? points_[point_count_ - 1].u_deg : 0.0f;
}

float V10ShankLinearMap::maxQ() const {
    return valid() ? points_[point_count_ - 1].q_deg : 0.0f;
}

} // namespace dog
