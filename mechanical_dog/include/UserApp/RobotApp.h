#pragma once

#include <cstdint>
#include "Ctrl/ChannelCalibration.h"
#include "Ctrl/Command.h"
#include "Ctrl/ServoController.h"
#include "Ctrl/AttitudeEstimator.h"
#include "Ctrl/RelativeServoMap.h"
#include "Ctrl/V10ShankLinearMap.h"
#include "Ctrl/V10Gait.h"
#include "Ctrl/V10StaticBalance.h"
#include "Ctrl/V10ControlProfile.h"
#include "Interface/IClock.h"
#include "Interface/IConsole.h"
#include "Interface/IImu.h"
#include "Interface/IMotionParameterStore.h"
#include "Interface/IControlProfileStore.h"

namespace dog {

struct DebugReply {
    DebugReply(bool success = false, const char* text = "Invalid command")
        : ok(success), message(text) {}
    bool ok;
    const char* message; // Static strings only.
};

struct DebugSnapshot {
    const char* firmware_version = "0.8.14";
    bool pca = false;
    bool armed = false;
    bool calibrating = false;
    bool calibration_pending = false;
    std::uint16_t calibration_mask = 0;
    bool fault = false;
    bool imu_ok = false;
    ImuSample imu{};
    bool attitude_ok = false;
    AttitudeSample attitude{};
    bool gyro_calibrated = false;
    std::uint16_t attitude_calibration_samples = 0;
    std::uint16_t attitude_calibration_required = AttitudeEstimator::kCalibrationSampleCount;
    float accel_norm_mps2 = 0.0f;
    int active_channel = -1;
    std::uint16_t active_pulse_us = 0;
    std::uint32_t remaining_ms = 0;
    bool manual_hold = false;
    bool extended_range = false;
    const char* motion_mode = "idle";
    const char* motion_error = "none";
    bool motion_pending = false;
    bool motion_transition = false;
    float motion_phase = 0;
    V10MotionParameters motion_parameters{};
    const char* motion_parameter_state = "built_in_default";
    bool motion_parameters_saved = false;
    std::uint32_t motion_parameter_revision = 0;
    std::uint32_t motion_frame_count = 0;
    std::uint32_t motion_last_update_gap_ms = 0;
    std::uint32_t motion_max_update_gap_ms = 0;
    std::uint32_t motion_missed_deadlines = 0;
    V10AdvancedMotionParameters advanced_parameters{};
    std::uint32_t advanced_parameter_revision = 0;
    float body_length_mm = 0;
    float body_width_mm = 0;
    bool body_dimensions_measured = false;
    bool balance_capture_pending = false;
    std::uint8_t balance_capture_samples = 0;
    std::uint8_t balance_capture_required = 20;
    bool balance_reference_ready = false;
    bool balance_enabled = false;
    bool balance_enable_pending = false;
    float balance_reference_pitch_deg = 0;
    float balance_reference_roll_deg = 0;
    float balance_pitch_correction_deg = 0;
    float balance_roll_correction_deg = 0;
    std::int8_t balance_roll_sensor_sign = kDefaultRollSensorSign;
    const char* balance_error = "none";
    const char* preview_error = "not_run";
    int preview_leg = -1;
    int preview_joint = -1;
    float preview_phase = 0;
    float preview_min_support_margin_mm = 0;
    bool preview_pending = false;
    const char* preview_stage = "idle";
    const char* preview_result = "not_run";
    std::uint32_t preview_frames_checked = 0;
    std::uint32_t preview_max_slice_ms = 0;
    float cg_actual_body_x_min_mm = 0;
    float cg_actual_body_x_max_mm = 0;
    std::uint32_t preflight_duration_ms = 0;
    float preflight_actual_cycle_s = 0;
    V10DriveIntent drive_intent{};
    float straight_trim = kDefaultStraightTrim;
    float effective_drive_turn = 0.0f;
    float trot_pitch_offset_deg = 0.0f;
    float trot_effective_pitch_deg = 0.0f;
    std::uint32_t trot_pitch_revision = 0;
    bool drive_session_active = false;
    std::uint32_t drive_epoch = 1;
    std::uint32_t drive_seq = 0;
    std::uint32_t drive_lease_remaining_ms = 0;
    const char* drive_session_reason = "idle";
    bool moving_cg_enabled = false;
    bool moving_cg_enable_pending = false;
    float moving_cg_shift_mm = 0;
    const char* moving_cg_error = "disabled";
    bool cg_phase_waiting = false;
    std::uint8_t cg_stage = 0;
    float motion_nominal_elapsed_s = 0;
    float motion_actual_elapsed_s = 0;
    float trot_feedforward_mm = 0;
    V10ControlProfile control_profile{};
    const char* control_profile_state = "built_in_default";
    bool control_profile_saved = false;
    std::uint32_t control_profile_revision = 0;
    float target_forward_mm[4] = {};
    float target_down_mm[4] = {};
    float command_deg[8] = {};
    std::uint16_t pulse_min_us = 500;
    std::uint16_t pulse_max_us = 2500;
    std::uint8_t channel_count = 0;
    struct Channel {
        Channel(std::uint8_t servo_label = 0, std::uint8_t raw_channel = 0,
                std::uint16_t center = 1500)
            : servo(servo_label), channel(raw_channel), center_us(center) {}
        std::uint8_t servo; // One-based silkscreen label.
        std::uint8_t channel; // Zero-based PCA channel.
        std::uint16_t center_us;
        std::uint8_t center_deg = 90;
        const char* joint = "unconfigured";
        bool relative_available = false;
        bool shank = false;
        float min_offset_deg = 0;
        float max_offset_deg = 0;
        bool shank_q_available = false;
        bool shank_q_photo_candidate = false;
        std::int8_t shank_direction = 0;
        float max_shank_q_deg = 0;
        std::uint8_t shank_map_point_count = 0;
        V10ShankMapPoint shank_map_points[3] = {};
    } channels[8] = {};
};

// V10 board application: retains the single-channel/eight-centre bench tools
// and adds an explicitly-started, IMU-gated, eight-channel open-loop gait.
// Motion telemetry is commanded state only; there is no joint/foot feedback.
class RobotApp {
public:
    RobotApp(ServoController& servos, ChannelCalibration& calibration,
             IImu& imu, IClock& clock, IConsole& console,
             const RelativeServoLimit* relative_limits = nullptr,
             std::size_t relative_count = 0,
             const V10ShankMapConfig* shank_maps = nullptr,
             std::size_t shank_map_count = 0,
             const V10MotionConfig* motion_config = nullptr,
             IMotionParameterStore* motion_store = nullptr,
             IControlProfileStore* profile_store = nullptr);
    void begin();
    void tick();
    void onLine(const char* line);
    DebugReply execute(const Command& command); // Shared by serial and Web.
    DebugSnapshot snapshot() const;

private:
    void status();
    void imuReport();
    void attitudeReport();
    bool imuFresh() const;
    const RelativeServoLimit* relativeLimit(std::uint8_t channel) const;
    const V10ShankMapConfig* shankMapConfig(std::uint8_t channel) const;
    bool shankRange(std::uint8_t channel, float center_deg,
                    std::int8_t& direction, float& max_u_deg,
                    float& max_q_deg) const;
    void centers(std::uint8_t out[8]) const;
    bool motionActive() const;
    bool motionRunningOrChanging() const;
    bool motionTuningAllowed() const;
    bool balanceBusy() const;
    enum class PreflightPurpose : std::uint8_t { None, MotionStart, Preview, DriveStart };
    struct PreflightJob {
        bool active = false;
        PreflightPurpose purpose = PreflightPurpose::None;
        MotionMode mode = MotionMode::Idle;
        V10MotionParameters parameters{};
        V10AdvancedMotionParameters advanced{};
        V10MovingCgConfig moving_config{110.0f,1.1f,5.0f,0.10f,0.5f,2.0f,0.005f,0.05f};
        V10CgPlannerConfig planner_config{20.0f,0.05f,15000U};
        std::uint8_t centers[8] = {};
        V10PreflightCursor cursor{};
        V10PreflightReport combined{};
        bool moving_envelope = false;
        bool planned_cg = false;
        int stage = 0;
        int dynamic_samples = 0;
        int dynamic_index = 0;
        float dynamic = 0;
        float dynamic_direction = 1;
        std::uint32_t started_ms = 0;
        std::uint32_t generation = 0;
        std::uint32_t motion_revision = 0;
        std::uint32_t advanced_revision = 0;
        std::uint32_t profile_revision = 0;
        std::uint32_t balance_reference_ms = 0;
        std::uint32_t attitude_reference_ms = 0;
        bool attitude_was_ready = false;
        std::uint32_t drive_epoch = 0;
    };
    bool queuePreflight(MotionMode mode,
                        const V10AdvancedMotionParameters& candidate,
                        PreflightPurpose purpose, std::uint32_t drive_epoch = 0);
    void preflightTick();
    void advancePreflightStage();
    void completePreflight();
    void cancelPreflight(const char* reason,
                         bool drive_session_already_stopped = false,
                         const char* error = "cancelled");
    bool preflightSnapshotValid() const;
    DebugReply requestMotion(MotionMode mode);
    DebugReply requestMotionWithAdvanced(MotionMode mode,
                                         const V10AdvancedMotionParameters& candidate,
                                         PreflightPurpose purpose = PreflightPurpose::MotionStart,
                                         std::uint32_t drive_epoch = 0);
    DebugReply armAndStand();
    bool validateAppliedParameters(const V10MotionParameters& parameters) const;
    static MotionParameterRecord makeMotionRecord(const V10MotionParameters& parameters);
    static bool decodeMotionRecord(const MotionParameterRecord& record,
                                   V10MotionParameters& parameters);
    static ControlProfileRecord makeProfileRecord(const V10ControlProfile& profile);
    static bool decodeProfileRecord(const ControlProfileRecord& record,
                                    V10ControlProfile& profile);
    bool validateControlProfile(const V10ControlProfile& profile) const;
    void applyControlProfile(const V10ControlProfile& profile,const char* state,
                             bool saved);
    void cancelMotion(const char* error, MotionMode mode);
    bool writeMotionFrame(const V10MotionFrame& frame);
    void motionTick();
    void balanceCaptureTick();
    void balanceTick();
    void disableBalance(bool clear_reference,const char* reason);
    bool balanceEnvelopeValid() const;
    bool currentStandMatchesApplied() const;
    V10PreflightReport preview(MotionMode mode) const;
    V10PreflightReport previewAdvanced(MotionMode mode,
                                       const V10AdvancedMotionParameters& advanced) const;
    void stopDriveSession(const char* reason);
    void disableMovingCg(const char* reason);
    bool movingCgUpdate();
    bool movingCgHealthy() const;
    bool cgTargets(float& front,float& rear) const;
    bool effectiveDrive(const V10DriveIntent& requested,
                        V10DriveIntent& effective) const;
    static const char* preflightErrorName(V10PreflightError error);

    ServoController& servos_;
    ChannelCalibration& calibration_;
    IImu& imu_;
    IClock& clock_;
    IConsole& console_;
    const RelativeServoLimit* const relative_limits_;
    const std::size_t relative_count_;
    const V10ShankMapConfig* const shank_maps_;
    const std::size_t shank_map_count_;
    const V10MotionConfig* const motion_config_;
    IMotionParameterStore* const motion_store_;
    IControlProfileStore* const profile_store_;
    ImuSample last_sample_{};
    AttitudeEstimator attitude_estimator_{};
    AttitudeSample last_attitude_{};
    std::uint32_t last_poll_ms_ = 0;
    bool imu_ready_ = false;
    bool sample_ready_ = false;
    bool attitude_ready_ = false;
    bool calibration_session_pending_ = false;
    V10MotionParameters motion_parameters_{};
    V10AdvancedMotionParameters advanced_parameters_{};
    MotionMode motion_mode_ = MotionMode::Idle;
    MotionMode motion_requested_ = MotionMode::Idle;
    bool motion_pending_ = false;
    bool motion_transition_ = false;
    bool motion_frame_valid_ = false;
    std::uint32_t imu_sample_generation_ = 0;
    std::uint32_t motion_request_sample_generation_ = 0;
    std::uint32_t motion_last_tick_ms_ = 0;
    std::uint32_t motion_transition_start_ms_ = 0;
    std::uint32_t motion_phase_start_ms_ = 0;
    V10MotionFrame motion_frame_{};
    V10MotionFrame motion_transition_from_{};
    V10MotionFrame motion_transition_to_{};
    const char* motion_error_ = "none";
    const char* motion_parameter_state_ = "built_in_default";
    bool motion_parameters_saved_ = false;
    bool saved_motion_parameters_valid_ = false;
    V10MotionParameters saved_motion_parameters_{};
    std::uint32_t motion_parameter_revision_ = 0;
    std::uint32_t motion_frame_count_ = 0;
    std::uint32_t motion_last_update_gap_ms_ = 0;
    std::uint32_t motion_max_update_gap_ms_ = 0;
    std::uint32_t motion_missed_deadlines_ = 0;
    std::uint32_t advanced_parameter_revision_ = 0;
    PreflightJob preflight_job_{};
    std::uint32_t preflight_generation_ = 0;
    const char* preflight_stage_ = "idle";
    const char* preflight_result_ = "not_run";
    std::uint32_t preflight_frames_checked_ = 0;
    std::uint32_t preflight_max_slice_ms_ = 0;
    V10PreflightReport last_preview_{};
    const char* preview_error_ = "not_run";
    V10StaticBalance balance_pitch_{{0.8f,0.08f,1.5f,4.0f,0.005f,0.05f}};
    V10StaticBalance balance_roll_{{0.8f,0.08f,1.5f,4.0f,0.005f,0.05f}};
    bool balance_capture_pending_ = false;
    bool balance_reference_ready_ = false;
    bool balance_enabled_ = false;
    bool balance_enable_pending_ = false;
    std::uint8_t balance_capture_count_ = 0;
    float balance_capture_pitch_sum_ = 0;
    float balance_capture_roll_sum_ = 0;
    float balance_capture_previous_pitch_ = 0;
    float balance_capture_previous_roll_ = 0;
    float balance_capture_pitch_min_ = 0;
    float balance_capture_pitch_max_ = 0;
    float balance_capture_roll_min_ = 0;
    float balance_capture_roll_max_ = 0;
    std::uint32_t balance_capture_last_ms_ = 0;
    std::uint32_t balance_reference_attitude_ms_ = 0;
    std::uint32_t balance_enable_sample_generation_ = 0;
    std::uint32_t balance_last_update_ms_ = 0;
    float balance_reference_pitch_deg_ = 0;
    float balance_reference_roll_deg_ = 0;
    float balance_pitch_correction_deg_ = 0;
    float balance_roll_correction_deg_ = 0;
    std::int8_t balance_roll_sensor_sign_ = kDefaultRollSensorSign;
    const char* balance_error_ = "none";
    V10DriveSession drive_session_{500};
    MotionMode drive_session_mode_ = MotionMode::Idle;
    const char* drive_session_reason_ = "idle";
    float straight_trim_ = kDefaultStraightTrim;
    std::uint32_t trot_pitch_revision_ = 0;
    V10MovingCg moving_cg_{{110.0f,1.1f,5.0f,0.10f,0.5f,2.0f,0.005f,0.05f}};
    bool moving_cg_enabled_ = false;
    bool moving_cg_enable_pending_ = false;
    std::uint32_t moving_cg_enable_generation_ = 0;
    std::uint32_t moving_cg_last_attitude_ms_ = 0;
    float moving_cg_shift_mm_ = 0;
    const char* moving_cg_error_ = "disabled";
    V10CgPlanner cg_planner_{{20.0f,0.05f,15000}};
    V10CgPlanOutput cg_plan_{};
    float motion_nominal_elapsed_s_ = 0;
    float motion_actual_elapsed_s_ = 0;
    mutable std::uint32_t preflight_duration_ms_ = 0;
    mutable float preflight_actual_cycle_s_ = 0;
    V10ControlProfile control_profile_{};
    V10ControlProfile saved_control_profile_{};
    bool saved_control_profile_valid_=false;
    bool control_profile_saved_=false;
    const char* control_profile_state_="built_in_default";
    std::uint32_t control_profile_revision_=0;
};

} // namespace dog
