#pragma once

#include <cstddef>
#include <cstdint>

namespace dog {

// u is the positive servo-command travel magnitude away from the channel's
// calibrated mechanical neutral. q is the observed shank turn magnitude away
// from that same neutral. Neither value is an absolute knee or servo angle.
struct V10ShankMapPoint {
    float u_deg;
    float q_deg;
};

// Each binding names one physical PCA channel. Channels confirmed to share the
// same mechanism and mechanical-neutral convention may point at one node table;
// their calibrated centers and actuator directions remain independent inputs.
struct V10ShankMapConfig {
    std::uint8_t channel;
    const V10ShankMapPoint* points;
    std::size_t point_count;
};

// Pure bounded piecewise-linear interpolation. A valid table starts at (0,0)
// and is strictly increasing in both u and q. Both conversions reject
// extrapolation, invalid/non-finite tables and non-finite inputs without
// modifying their output argument.
class V10ShankLinearMap {
public:
    V10ShankLinearMap(const V10ShankMapPoint* points, std::size_t point_count)
        : points_(points), point_count_(point_count) {}

    bool valid() const;
    bool qFromU(float u_deg, float& q_deg) const;
    bool uFromQ(float q_deg, float& u_deg) const;
    float maxU() const;
    float maxQ() const;

private:
    const V10ShankMapPoint* points_;
    std::size_t point_count_;
};

} // namespace dog
