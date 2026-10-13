#include "Ctrl/V10BodyPose.h"

#include <cmath>

namespace dog {
namespace {
constexpr float kRadiansPerDegree = 0.01745329251994329577f;
bool finite(float value) { return std::isfinite(value); }
}

bool V10BodyPose::valid() const {
    return finite(body_length_mm_) && finite(body_width_mm_) &&
           body_length_mm_ > 0.0f && body_width_mm_ > 0.0f;
}

bool V10BodyPose::apply(const float nominal_forward[4], const float nominal_down[4],
                        const V10BodyPoseCommand& command,
                        float forward[4], float down[4]) const {
    if (!valid() || !nominal_forward || !nominal_down || !forward || !down ||
        !finite(command.pitch_deg) || !finite(command.roll_deg) ||
        !finite(command.body_x_mm) || command.pitch_deg < kBodyPitchMinDeg ||
        command.pitch_deg > kBodyPitchMaxDeg || command.roll_deg < kBodyRollMinDeg ||
        command.roll_deg > kBodyRollMaxDeg || std::fabs(command.body_x_mm) > 40.0f)
        return false;

    const float pitch = command.pitch_deg * kRadiansPerDegree;
    const float roll = command.roll_deg * kRadiansPerDegree;
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    const float cr = std::cos(roll), sr = std::sin(roll);
    // Body-to-world R = Ry(-pitch) * Rx(roll).  Use R^T below to express
    // fixed zero-pose world feet in the commanded body frame.
    const float r00=cp,      r02=-sp*cr;
    const float r10=0.0f,    r12=-sr;
    const float r20=sp,      r22=cp*cr;
    const float half_length = body_length_mm_ * 0.5f;
    const float half_width = body_width_mm_ * 0.5f;
    const float hip_x[4]={half_length,half_length,-half_length,-half_length};
    const float hip_y[4]={half_width,-half_width,-half_width,half_width};
    for(int leg=0;leg<4;++leg) {
        if(!finite(nominal_forward[leg]) || !finite(nominal_down[leg])) return false;
        const float world_x=hip_x[leg]+nominal_forward[leg]-command.body_x_mm;
        const float world_y=hip_y[leg];
        const float world_z=-nominal_down[leg];
        const float body_x=r00*world_x+r10*world_y+r20*world_z-hip_x[leg];
        const float body_z=r02*world_x+r12*world_y+r22*world_z;
        if(!finite(body_x) || !finite(body_z)) return false;
        forward[leg]=body_x;
        down[leg]=-body_z;
    }
    return true;
}

} // namespace dog
