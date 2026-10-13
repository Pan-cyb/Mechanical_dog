#pragma once

namespace dog {

// Planar geometry for the user's V10 serial leg. All dimensions are physical
// pivot-to-pivot distances in mm; deliberately no implicit defaults.
// `lower_mm` depends on the chosen foot reference (toe round centre vs
// ground-contact point), so a caller must provide it explicitly.
struct V10LegGeometry {
    float upper_mm;
    float lower_mm;
};

// Local side-view frame: origin at hip, +X to the photograph's right,
// +Y vertically downward. This local +X is not robot-forward; body/leg
// mirroring belongs in the caller's explicit frame adapter (V10Gait provides
// the active one), never as a hidden x=-x inside IK. Clockwise knee turn is
// positive in this frame.
enum class KneeFold { Clockwise, CounterClockwise };

struct V10LegAngles {
    float hip_deg = 0.0f;       // Upper link relative to +X, clockwise positive.
    float knee_turn_deg = 0.0f; // Signed turn from upper-link extension to shank.
};

// Geometry only: neither field is a servo angle. No PWM, linkage fit or
// automatic mirroring of the other three legs is provided here.
class V10LegIk {
public:
    explicit V10LegIk(V10LegGeometry geometry) : geometry_(geometry) {}
    bool solve(float x_mm, float y_down_mm, KneeFold fold,
               V10LegAngles& out) const;
    bool forward(const V10LegAngles& angles, float& x_mm,
                 float& y_down_mm) const;
    const V10LegGeometry& geometry() const { return geometry_; }

private:
    V10LegGeometry geometry_;
};

} // namespace dog
