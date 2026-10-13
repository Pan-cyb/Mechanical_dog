#pragma once

#include <cstddef>
#include <cstdint>
#include "Ctrl/V10LegIk.h"
#include "Ctrl/V10ShankLinearMap.h"
#include "Ctrl/V10BodyPose.h"
#include "Ctrl/V10DriveControl.h"

namespace dog {

enum class MotionMode : std::uint8_t {
    Idle, Hold, Stand, Walk, Trot, Leg1, Leg2, Leg3, Leg4, HoldError
};

struct V10MotionParameters {
    float step_length_mm = 17.9f;
    float lift_mm = 12.8f;
    float period_s = 0.5f;
    float stand_down_mm = 114.0f;
    float stand_forward_mm = 0.0f;
    float walk_body_shift_mm = 2.0f;
};

struct V10MotionConfig {
    V10LegGeometry geometry;
    float hip_geometry_neutral_deg;
    float knee_fold_neutral_deg;
    float hip_window_deg;
    float transition_s;
    std::uint32_t update_ms;
    std::uint32_t max_tick_gap_ms;
    std::uint8_t channels[8];
    std::int8_t hip_direction[4];
    std::int8_t shank_direction[4];
    const V10ShankMapPoint* shank_points;
    std::size_t shank_point_count;
    float body_length_mm;
    float body_width_mm;
};

struct V10AdvancedMotionParameters {
    V10BodyPoseCommand body{};
    // Volatile open-loop angle feedforward. Applied only to Trot before the
    // common BodyPose transform; Stand/Walk/Leg trajectories ignore it.
    float trot_pitch_offset_deg = 0.0f;
    V10DriveIntent drive{};
    V10TrotFeedforwardConfig trot_feedforward{};
    bool cg_walk_enabled = false;
    float cg_shift_mm = 2.5f;
    float dynamic_body_x_mm = 0.0f;
    bool cg_body_x_override_valid = false;
    float cg_body_x_override_mm = 0.0f;
};

enum class V10PreflightError : std::uint8_t {
    None, InvalidParameters, BodyTransform, Ik, HipWindow, ShankMap,
    ServoRange, SupportMargin, DriveIntent
};

struct V10PreflightReport {
    bool ok = false;
    V10PreflightError error = V10PreflightError::InvalidParameters;
    int leg = -1;
    int joint = -1;
    float phase = 0.0f;
    float min_support_margin_mm = 0.0f;
};

struct V10MotionFrame {
    float forward_mm[4] = {};
    float down_mm[4] = {};
    float servo_deg[8] = {};
    std::uint16_t pulse_us[8] = {};
    bool swing[4] = {};
};

class V10PreflightCursor;

class V10Gait {
public:
    explicit V10Gait(const V10MotionConfig& config) : config_(config) {}
    bool parametersValid(const V10MotionParameters& p) const;
    bool frame(MotionMode mode, float phase, const V10MotionParameters& p,
               const std::uint8_t centers_deg[8], V10MotionFrame& out) const;
    bool frameAdvanced(MotionMode mode, float phase, const V10MotionParameters& p,
                       const V10AdvancedMotionParameters& advanced,
                       const std::uint8_t centers_deg[8], V10MotionFrame& out) const;
    V10PreflightReport frameAdvancedReport(MotionMode mode,float phase,
                       const V10MotionParameters& p,
                       const V10AdvancedMotionParameters& advanced,
                       const std::uint8_t centers_deg[8],V10MotionFrame& out) const;
    bool preflight(MotionMode mode, const V10MotionParameters& p,
                   const std::uint8_t centers_deg[8]) const;
    V10PreflightReport preflightAdvanced(MotionMode mode,
                   const V10MotionParameters& p,
                   const V10AdvancedMotionParameters& advanced,
                   const std::uint8_t centers_deg[8]) const;
    V10PreflightReport preflightPlannedCg(
                   const V10MotionParameters& p,
                   const V10AdvancedMotionParameters& advanced,
                   const std::uint8_t centers_deg[8],
                   const V10CgPlannerConfig& planner_config,
                   float dynamic_envelope_mm,float dynamic_slew_mm_s,
                   float& actual_cycle_s) const;
    const V10MotionConfig& config() const { return config_; }
    static const char* modeName(MotionMode mode);

private:
    friend class V10PreflightCursor;
    bool configValid() const;
    bool advancedValid(const V10AdvancedMotionParameters& advanced) const;
    bool frameDetailed(MotionMode mode,float phase,const V10MotionParameters& p,
                       const V10AdvancedMotionParameters& advanced,
                       const std::uint8_t centers_deg[8],V10MotionFrame& out,
                       V10PreflightError& error,int& leg) const;
    V10MotionConfig config_;
};

// Resumable pure-C++ preflight. Each step evaluates at most one trajectory
// frame; synchronous wrappers below drive this same cursor to completion.
class V10PreflightCursor {
public:
    bool beginAdvanced(const V10MotionConfig& config, MotionMode mode,
                       const V10MotionParameters& parameters,
                       const V10AdvancedMotionParameters& advanced,
                       const std::uint8_t centers_deg[8]);
    bool beginPlannedCg(const V10MotionConfig& config,
                        const V10MotionParameters& parameters,
                        const V10AdvancedMotionParameters& advanced,
                        const std::uint8_t centers_deg[8],
                        const V10CgPlannerConfig& planner_config,
                        float dynamic_envelope_mm, float dynamic_slew_mm_s);
    bool step(); // Returns true when the report is final.
    bool active() const { return started_ && !finished_; }
    bool finished() const { return finished_; }
    const V10PreflightReport& report() const { return report_; }
    float actualCycleS() const { return actual_cycle_s_; }
    std::uint32_t processedFrames() const { return processed_frames_; }

private:
    enum class Kind : std::uint8_t { None, Advanced, PlannedCg };
    void finish(const V10PreflightReport& report);
    void beginPlannedScenario();
    bool stepAdvanced();
    bool stepPlannedCg();

    Kind kind_ = Kind::None;
    bool started_ = false;
    bool finished_ = true;
    V10MotionConfig config_{};
    MotionMode mode_ = MotionMode::Idle;
    V10MotionParameters parameters_{};
    V10AdvancedMotionParameters advanced_{};
    std::uint8_t centers_[8] = {};
    V10PreflightReport report_{};
    int samples_ = 0;
    int sample_index_ = 0;
    V10CgPlannerConfig planner_config_{20.0f, 0.05f, 15000U};
    V10CgPlanner planner_{planner_config_};
    V10CgPlanOutput plan_{};
    float front_target_ = 0;
    float rear_target_ = 0;
    float dynamic_envelope_ = 0;
    float dynamic_slew_ = 0;
    float dynamic_ = 0;
    float dynamic_direction_ = 1;
    int scenario_ = 0;
    int scenario_count_ = 0;
    float actual_cycle_s_ = 0;
    std::uint32_t processed_frames_ = 0;
};

} // namespace dog
