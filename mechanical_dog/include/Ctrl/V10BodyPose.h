#pragma once

namespace dog {

// Probe-derived command envelope. Every concrete height/pitch/roll tuple is
// still validated through the complete Stand transform/IK/map/servo path.
inline constexpr float kBodyPitchMinDeg = -7.0f;
inline constexpr float kBodyPitchMaxDeg = 5.0f;
inline constexpr float kBodyRollMinDeg = -10.0f;
inline constexpr float kBodyRollMaxDeg = 10.0f;

// Body frame: +X forward, +Y left, +Z up. Positive pitch raises the nose;
// positive roll raises the left side. The 8-DOF leg model discards the
// transformed lateral component, so this is an explicit sagittal projection,
// not a complete 3-D contact/COM model.
struct V10BodyPoseCommand {
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float body_x_mm = 0.0f;
};

class V10BodyPose {
public:
    V10BodyPose(float body_length_mm, float body_width_mm)
        : body_length_mm_(body_length_mm), body_width_mm_(body_width_mm) {}

    bool valid() const;
    bool apply(const float nominal_forward_mm[4], const float nominal_down_mm[4],
               const V10BodyPoseCommand& command,
               float forward_mm[4], float down_mm[4]) const;

private:
    float body_length_mm_;
    float body_width_mm_;
};

} // namespace dog
