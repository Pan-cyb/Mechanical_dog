#include <cassert>
#include <cmath>
#include <initializer_list>
#include <limits>

#include "Ctrl/V10LegIk.h"

namespace {
bool near(float a, float b, float tolerance = 0.01f) {
    return std::fabs(a - b) < tolerance;
}
}

int main() {
    using namespace dog;

    // These explicit lengths are a geometry fixture, not an assertion that
    // the user's printed V10 parts have been measured or motion is safe.
    constexpr float upper_mm = 80.0f;
    constexpr float lower_mm = 61.357f;
    const V10LegIk leg(V10LegGeometry{upper_mm, lower_mm});

    // x points right; y points down. In the photographed 90-degree assembly,
    // the thigh points down and the shank points left. A clockwise turn in
    // this screen-coordinate convention is positive.
    V10LegAngles clockwise{};
    assert(leg.solve(-lower_mm, upper_mm, KneeFold::Clockwise, clockwise));
    assert(near(clockwise.hip_deg, 90.0f));
    assert(near(clockwise.knee_turn_deg, 90.0f));
    float foot_x = 0.0f, foot_y = 0.0f;
    assert(leg.forward(clockwise, foot_x, foot_y));
    assert(near(foot_x, -lower_mm) && near(foot_y, upper_mm));

    // The alternate fold can reach the same foot, but it is not the same
    // physical posture. The signed knee angle preserves the branch for FK.
    V10LegAngles counterclockwise{};
    assert(leg.solve(-lower_mm, upper_mm, KneeFold::CounterClockwise,
                     counterclockwise));
    assert(near(counterclockwise.knee_turn_deg, -90.0f));
    assert(!near(counterclockwise.hip_deg, clockwise.hip_deg, 1.0f));
    assert(leg.forward(counterclockwise, foot_x, foot_y));
    assert(near(foot_x, -lower_mm) && near(foot_y, upper_mm));

    // A small perturbation of the foot must remain continuous on the selected
    // branch. Round-trip checks use only measured lengths supplied to IK.
    V10LegAngles near_neutral{};
    assert(leg.solve(-lower_mm + 0.2f, upper_mm - 0.1f,
                     KneeFold::Clockwise, near_neutral));
    assert(std::fabs(near_neutral.hip_deg - clockwise.hip_deg) < 1.0f);
    assert(std::fabs(near_neutral.knee_turn_deg - clockwise.knee_turn_deg) < 1.0f);
    assert(leg.forward(near_neutral, foot_x, foot_y));
    assert(near(foot_x, -lower_mm + 0.2f));
    assert(near(foot_y, upper_mm - 0.1f));

    // Sign contract: pure V10 IK must never secretly mirror local X.
    // Body-forward/left-right transforms will be a separate, tested adapter.
    for (float target_x : {-10.0f, 10.0f}) {
        V10LegAngles side{};
        assert(leg.solve(target_x, 100.0f, KneeFold::Clockwise, side));
        assert(leg.forward(side, foot_x, foot_y));
        assert(near(foot_x, target_x) && near(foot_y, 100.0f));
    }

    // The IK has no unique knee branch at either straight or fully folded
    // singularity; neither should silently become a motor target.
    const V10LegIk integer_leg(V10LegGeometry{80.0f, 60.0f});
    V10LegAngles unchanged{13.0f, 27.0f};
    assert(!integer_leg.solve(140.0f, 0.0f, KneeFold::Clockwise, unchanged));
    assert(!integer_leg.solve(20.0f, 0.0f, KneeFold::Clockwise, unchanged));
    assert(!integer_leg.solve(141.0f, 0.0f, KneeFold::Clockwise, unchanged));
    assert(!integer_leg.solve(19.0f, 0.0f, KneeFold::Clockwise, unchanged));
    assert(near(unchanged.hip_deg, 13.0f) && near(unchanged.knee_turn_deg, 27.0f));

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    assert(!leg.solve(nan, upper_mm, KneeFold::Clockwise, unchanged));
    assert(!leg.solve(-lower_mm, inf, KneeFold::Clockwise, unchanged));
    assert(!leg.solve(-lower_mm, upper_mm,
                      static_cast<KneeFold>(255), unchanged));
    assert(near(unchanged.hip_deg, 13.0f) && near(unchanged.knee_turn_deg, 27.0f));
    const V10LegIk zero_length(V10LegGeometry{80.0f, 0.0f});
    assert(!zero_length.solve(-lower_mm, upper_mm, KneeFold::Clockwise,
                              unchanged));
    const V10LegIk invalid_length(V10LegGeometry{inf, lower_mm});
    assert(!invalid_length.solve(-lower_mm, upper_mm, KneeFold::Clockwise,
                                 unchanged));

    const float previous_x = foot_x, previous_y = foot_y;
    assert(!leg.forward(V10LegAngles{nan, 90.0f}, foot_x, foot_y));
    assert(!leg.forward(V10LegAngles{90.0f, inf}, foot_x, foot_y));
    assert(near(foot_x, previous_x) && near(foot_y, previous_y));
}
