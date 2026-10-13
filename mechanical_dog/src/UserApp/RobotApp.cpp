#include "UserApp/RobotApp.h"

#include <cstdio>
#include <cmath>
#include <cstring>
#include "Ctrl/Command.h"

namespace dog {
namespace {
float smooth(float t) { return t*t*(3.0f-2.0f*t); }
bool configuredShankDirection(const RelativeServoLimit& limit,
                              std::int8_t& direction) {
    if (!limit.shank || !std::isfinite(limit.min_offset_deg) ||
        !std::isfinite(limit.max_offset_deg) || limit.min_offset_deg > 0.0f ||
        limit.max_offset_deg < 0.0f ||
        limit.min_offset_deg > limit.max_offset_deg) return false;
    const bool negative = limit.min_offset_deg < 0.0f;
    const bool positive = limit.max_offset_deg > 0.0f;
    // Reject a two-sided range and the zero-only range. Direction comes from
    // the original configuration, never from center-clipped 0..0 bounds.
    if (negative == positive) return false;
    direction = negative ? -1 : 1;
    return true;
}
std::uint32_t checksumRecord(const MotionParameterRecord& record) {
    std::uint32_t hash = 2166136261U;
    const auto mix = [&hash](const void* data, std::size_t size) {
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        for (std::size_t i=0;i<size;++i) { hash ^= bytes[i]; hash *= 16777619U; }
    };
    mix(&record.magic,sizeof(record.magic));
    mix(&record.version,sizeof(record.version));
    mix(&record.length,sizeof(record.length));
    mix(record.values,sizeof(record.values));
    return hash;
}
std::uint32_t checksumProfileRecord(const ControlProfileRecord& record) {
    std::uint32_t hash=2166136261U;
    const auto mix=[&hash](const void* data,std::size_t size){const auto* bytes=
        static_cast<const std::uint8_t*>(data);for(std::size_t i=0;i<size;++i){hash^=bytes[i];hash*=16777619U;}};
    mix(&record.magic,sizeof(record.magic));mix(&record.version,sizeof(record.version));
    mix(&record.length,sizeof(record.length));mix(&record.flags,sizeof(record.flags));
    mix(record.values,sizeof(record.values));return hash;
}
bool sameParameters(const V10MotionParameters& a,const V10MotionParameters& b) {
    const float av[6]={a.step_length_mm,a.lift_mm,a.period_s,a.stand_down_mm,
                       a.stand_forward_mm,a.walk_body_shift_mm};
    const float bv[6]={b.step_length_mm,b.lift_mm,b.period_s,b.stand_down_mm,
                       b.stand_forward_mm,b.walk_body_shift_mm};
    for(int i=0;i<6;++i) if(std::fabs(av[i]-bv[i])>0.0001f) return false;
    return true;
}
}

RobotApp::RobotApp(ServoController& servos, ChannelCalibration& calibration,
                   IImu& imu, IClock& clock, IConsole& console,
                   const RelativeServoLimit* relative_limits, std::size_t relative_count,
                   const V10ShankMapConfig* shank_maps, std::size_t shank_map_count,
                   const V10MotionConfig* motion_config,
                   IMotionParameterStore* motion_store,
                   IControlProfileStore* profile_store)
    : servos_(servos), calibration_(calibration), imu_(imu), clock_(clock),
      console_(console), relative_limits_(relative_limits), relative_count_(relative_count),
      shank_maps_(shank_maps), shank_map_count_(shank_map_count),
      motion_config_(motion_config), motion_store_(motion_store),
      profile_store_(profile_store) {}

void RobotApp::centers(std::uint8_t out[8]) const {
    for (int i=0;i<8;++i) out[i] = calibration_.centerDeg(motion_config_->channels[i]);
}

bool RobotApp::motionActive() const {
    return preflight_job_.active || motion_pending_ || motion_transition_ ||
           servos_.coordinated();
}

bool RobotApp::motionRunningOrChanging() const {
    return preflight_job_.active || motion_pending_ || motion_transition_ ||
        motion_mode_ == MotionMode::Walk ||
        motion_mode_ == MotionMode::Trot ||
        (motion_mode_ >= MotionMode::Leg1 && motion_mode_ <= MotionMode::Leg4);
}

bool RobotApp::motionTuningAllowed() const {
    if (motionRunningOrChanging() || servos_.calibrating() ||
        servos_.activeChannel() >= 0 || servos_.faulted() || balanceBusy() ||
        moving_cg_enabled_ || moving_cg_enable_pending_) return false;
    if (motion_mode_ == MotionMode::Idle) return true;
    return motion_frame_valid_ && servos_.coordinated() &&
           (motion_mode_ == MotionMode::Stand || motion_mode_ == MotionMode::Hold);
}

bool RobotApp::balanceBusy() const {
    return balance_capture_pending_ || balance_enabled_ || balance_enable_pending_;
}

const char* RobotApp::preflightErrorName(V10PreflightError error) {
    switch(error) {
    case V10PreflightError::None:return "none";
    case V10PreflightError::InvalidParameters:return "invalid_parameters";
    case V10PreflightError::BodyTransform:return "body_transform";
    case V10PreflightError::Ik:return "ik";
    case V10PreflightError::HipWindow:return "hip_window";
    case V10PreflightError::ShankMap:return "shank_map";
    case V10PreflightError::ServoRange:return "servo_range";
    case V10PreflightError::SupportMargin:return "support_margin";
    case V10PreflightError::DriveIntent:return "drive_intent";
    }
    return "unknown";
}

V10PreflightReport RobotApp::preview(MotionMode mode) const {
    return previewAdvanced(mode,advanced_parameters_);
}

bool RobotApp::effectiveDrive(const V10DriveIntent& requested,
                              V10DriveIntent& effective) const {
    return applyStraightTrim(requested,straight_trim_,effective);
}

V10PreflightReport RobotApp::previewAdvanced(
        MotionMode mode,const V10AdvancedMotionParameters& advanced) const {
    if(!motion_config_)return {};
    const std::uint32_t started=clock_.millis();
    std::uint8_t c[8];centers(c);
    V10Gait gait(*motion_config_);
    preflight_actual_cycle_s_=0;
    const bool moving_envelope=moving_cg_enabled_||moving_cg_enable_pending_;
    V10PreflightReport combined=mode==MotionMode::Walk&&advanced.cg_walk_enabled ?
        gait.preflightPlannedCg(motion_parameters_,advanced,c,cg_planner_.config(),
            moving_envelope?moving_cg_.config().max_abs_shift_mm:0.0f,
            moving_envelope?moving_cg_.config().max_slew_mm_s:0.0f,
            preflight_actual_cycle_s_) :
        gait.preflightAdvanced(mode,motion_parameters_,advanced,c);
    if(!combined.ok){preflight_duration_ms_=clock_.millis()-started;return combined;}
    if(moving_envelope&&!(mode==MotionMode::Walk&&advanced.cg_walk_enabled)) {
        const float envelopes[2]={-moving_cg_.config().max_abs_shift_mm,
                                   moving_cg_.config().max_abs_shift_mm};
        for(float envelope:envelopes) {
            V10AdvancedMotionParameters test=advanced;test.dynamic_body_x_mm=envelope;
            const V10PreflightReport report=gait.preflightAdvanced(mode,motion_parameters_,test,c);
            if(!report.ok){preflight_duration_ms_=clock_.millis()-started;return report;}
            if((combined.min_support_margin_mm==0||report.min_support_margin_mm<combined.min_support_margin_mm)&&
               report.min_support_margin_mm!=0)combined.min_support_margin_mm=report.min_support_margin_mm;
        }
        const int samples=mode==MotionMode::Stand?1:
            static_cast<int>(std::ceil(motion_parameters_.period_s*1000.0f/motion_config_->update_ms));
        float dynamic=-moving_cg_.config().max_abs_shift_mm,direction=1.0f;
        for(int i=0;i<samples;++i){
            const float phase=samples==1?0.0f:static_cast<float>(i)/samples;
            dynamic+=direction*moving_cg_.config().max_slew_mm_s*
                (motion_config_->update_ms*0.001f);
            if(dynamic>=moving_cg_.config().max_abs_shift_mm){dynamic=moving_cg_.config().max_abs_shift_mm;direction=-1;}
            if(dynamic<=-moving_cg_.config().max_abs_shift_mm){dynamic=-moving_cg_.config().max_abs_shift_mm;direction=1;}
            V10AdvancedMotionParameters test=advanced;test.dynamic_body_x_mm=dynamic;
            V10MotionFrame frame{};const V10PreflightReport report=gait.frameAdvancedReport(
                mode,phase,motion_parameters_,test,c,frame);
            if(!report.ok){preflight_duration_ms_=clock_.millis()-started;return report;}
        }
    }
    preflight_duration_ms_=clock_.millis()-started;return combined;
}

bool RobotApp::queuePreflight(MotionMode mode,
        const V10AdvancedMotionParameters& candidate,
        PreflightPurpose purpose, std::uint32_t drive_epoch) {
    if (!motion_config_ || preflight_job_.active) return false;
    PreflightJob job{};
    job.active = true;
    job.purpose = purpose;
    job.mode = mode;
    job.parameters = motion_parameters_;
    job.advanced = candidate;
    job.moving_config = moving_cg_.config();
    job.planner_config = cg_planner_.config();
    centers(job.centers);
    job.moving_envelope = moving_cg_enabled_ || moving_cg_enable_pending_;
    job.planned_cg = mode == MotionMode::Walk && candidate.cg_walk_enabled;
    job.started_ms = clock_.millis();
    job.generation = ++preflight_generation_;
    job.motion_revision = motion_parameter_revision_;
    job.advanced_revision = advanced_parameter_revision_;
    job.profile_revision = control_profile_revision_;
    job.balance_reference_ms = balance_reference_attitude_ms_;
    job.attitude_reference_ms = last_attitude_.yaw_reference_ms;
    job.attitude_was_ready = attitude_ready_;
    job.drive_epoch = drive_epoch;
    const bool begun = job.planned_cg ?
        job.cursor.beginPlannedCg(*motion_config_, job.parameters, job.advanced,
            job.centers, job.planner_config,
            job.moving_envelope ? job.moving_config.max_abs_shift_mm : 0.0f,
            job.moving_envelope ? job.moving_config.max_slew_mm_s : 0.0f) :
        job.cursor.beginAdvanced(*motion_config_, mode, job.parameters,
                                 job.advanced, job.centers);
    last_preview_ = V10PreflightReport{};
    preview_error_ = "checking";
    preflight_result_ = "checking";
    preflight_stage_ = job.planned_cg ? "cg_planner" : "trajectory";
    preflight_frames_checked_ = 0;
    preflight_max_slice_ms_ = 0;
    preflight_actual_cycle_s_ = 0;
    preflight_duration_ms_ = 0;
    if (!begun) {
        last_preview_ = job.cursor.report();
        preview_error_ = preflightErrorName(last_preview_.error);
        preflight_result_ = "rejected";
        preflight_stage_ = "complete";
        return false;
    }
    preflight_job_ = job;
    return true;
}

void RobotApp::advancePreflightStage() {
    PreflightJob& job = preflight_job_;
    const V10PreflightReport report = job.cursor.report();
    if (!report.ok) {
        job.combined = report;
        completePreflight();
        return;
    }
    if (job.stage == 0) job.combined = report;
    else {
        if ((job.combined.min_support_margin_mm == 0 ||
             report.min_support_margin_mm < job.combined.min_support_margin_mm) &&
            report.min_support_margin_mm != 0)
            job.combined.min_support_margin_mm = report.min_support_margin_mm;
    }
    if (job.planned_cg) {
        preflight_actual_cycle_s_ = job.cursor.actualCycleS();
        completePreflight();
        return;
    }
    if (!job.moving_envelope) {
        completePreflight();
        return;
    }
    if (job.stage < 2) {
        ++job.stage;
        V10AdvancedMotionParameters test = job.advanced;
        test.dynamic_body_x_mm = job.stage == 1 ?
            -job.moving_config.max_abs_shift_mm :
             job.moving_config.max_abs_shift_mm;
        if (!job.cursor.beginAdvanced(*motion_config_, job.mode, job.parameters,
                                      test, job.centers)) {
            job.combined = job.cursor.report();
            completePreflight();
            return;
        }
        preflight_stage_ = job.stage == 1 ? "envelope_negative" :
                                                "envelope_positive";
        return;
    }
    job.stage = 3;
    job.dynamic_samples = job.mode == MotionMode::Stand ? 1 :
        static_cast<int>(std::ceil(job.parameters.period_s * 1000.0f /
                                   motion_config_->update_ms));
    job.dynamic_index = 0;
    job.dynamic = -job.moving_config.max_abs_shift_mm;
    job.dynamic_direction = 1.0f;
    preflight_stage_ = "dynamic_ramp";
}

bool RobotApp::preflightSnapshotValid() const {
    if (!preflight_job_.active ||
        preflight_job_.motion_revision != motion_parameter_revision_ ||
        preflight_job_.advanced_revision != advanced_parameter_revision_ ||
        preflight_job_.profile_revision != control_profile_revision_)
        return false;
    std::uint8_t current_centers[8];
    centers(current_centers);
    if (std::memcmp(current_centers, preflight_job_.centers,
                    sizeof(current_centers)) != 0) return false;
    if (preflight_job_.balance_reference_ms != balance_reference_attitude_ms_)
        return false;
    if(preflight_job_.attitude_was_ready&&
       (!attitude_ready_||preflight_job_.attitude_reference_ms!=last_attitude_.yaw_reference_ms))
        return false;
    return true;
}

void RobotApp::completePreflight() {
    PreflightJob& job = preflight_job_;
    last_preview_ = job.combined;
    preflight_duration_ms_ = clock_.millis() - job.started_ms;
    preflight_stage_ = "complete";
    if (!job.combined.ok) {
        preview_error_ = preflightErrorName(job.combined.error);
        preflight_result_ = "rejected";
        if (job.purpose == PreflightPurpose::DriveStart &&
            drive_session_.active() && drive_session_.epoch() == job.drive_epoch) {
            drive_session_.invalidate();
            drive_session_mode_ = MotionMode::Idle;
            drive_session_reason_ = "preflight_rejected";
        }
        job.active = false;
        return;
    }
    preview_error_ = "none";
    if (job.purpose == PreflightPurpose::Preview) {
        preflight_result_ = "passed";
        job.active = false;
        return;
    }
    if (!preflightSnapshotValid()) {
        preflight_result_ = "cancelled_state_changed";
        cancelPreflight(preflight_result_,false,"state_changed");
        return;
    }
    if (!imuFresh()) {
        preflight_result_ = "rejected_imu_stale";
        cancelPreflight(preflight_result_,false,"imu_stale");
        return;
    }
    if (!servos_.armed() || servos_.faulted()) {
        preflight_result_ = "rejected_output_unhealthy";
        cancelPreflight(preflight_result_,false,"output_unhealthy");
        return;
    }
    if (job.moving_envelope && !movingCgHealthy()) {
        preflight_result_ = "rejected_reference";
        cancelPreflight(preflight_result_,false,"reference_invalid");
        return;
    }
    if (job.purpose == PreflightPurpose::DriveStart &&
        (!drive_session_.active() || drive_session_.epoch() != job.drive_epoch ||
         drive_session_.remainingMs(clock_.millis()) == 0)) {
        preflight_result_ = "cancelled_lease";
        cancelPreflight(preflight_result_,false,"lease_expired");
        return;
    }
    V10AdvancedMotionParameters runtime = job.advanced;
    runtime.dynamic_body_x_mm = moving_cg_shift_mm_;
    V10MotionFrame target{};
    if (!V10Gait(*motion_config_).frameAdvanced(
            job.mode, 0, job.parameters, runtime, job.centers, target)) {
        preflight_result_ = "rejected_initial_frame";
        cancelPreflight(preflight_result_,false,"initial_frame_invalid");
        return;
    }
    const PreflightPurpose purpose = job.purpose;
    const MotionMode mode = job.mode;
    const V10AdvancedMotionParameters candidate = job.advanced;
    job.active = false;
    if (purpose == PreflightPurpose::MotionStart &&
        (mode == MotionMode::Stand || mode == MotionMode::Walk ||
         mode == MotionMode::Trot)) {
        stopDriveSession(mode == MotionMode::Stand ? "manual_stand" :
                         mode == MotionMode::Walk ? "manual_walk" : "manual_trot");
        if (mode == MotionMode::Stand) disableMovingCg("manual_stand");
    }
    advanced_parameters_ = candidate;
    if (balanceBusy()) disableBalance(false, "paused_for_motion_transition");
    motion_requested_ = mode;
    motion_pending_ = true;
    motion_request_sample_generation_ = imu_sample_generation_;
    motion_error_ = "none";
    preflight_result_ = "passed";
    if (purpose == PreflightPurpose::DriveStart) {
        drive_session_mode_ = mode;
        drive_session_reason_ = "active";
    }
}

void RobotApp::cancelPreflight(const char* reason,
                               bool drive_session_already_stopped,
                               const char* error) {
    if (!preflight_job_.active) return;
    const PreflightPurpose purpose = preflight_job_.purpose;
    const std::uint32_t drive_epoch = preflight_job_.drive_epoch;
    preflight_duration_ms_ = clock_.millis() - preflight_job_.started_ms;
    preflight_job_.active = false;
    ++preflight_generation_;
    preflight_stage_ = "complete";
    preflight_result_ = reason;
    preview_error_ = error;
    if (purpose == PreflightPurpose::DriveStart &&
        !drive_session_already_stopped && drive_session_.active() &&
        drive_session_.epoch() == drive_epoch) {
        drive_session_.invalidate();
        drive_session_mode_ = MotionMode::Idle;
        drive_session_reason_ = reason;
    }
}

void RobotApp::preflightTick() {
    if (!preflight_job_.active) return;
    const PreflightPurpose purpose=preflight_job_.purpose;
    if(preflight_job_.balance_reference_ms!=balance_reference_attitude_ms_){
        const bool moving=preflight_job_.moving_envelope;
        cancelPreflight("cancelled_reference_reset",false,"reference_invalid");
        if(moving)disableMovingCg("preflight_reference_invalid");
        return;
    }
    if(preflight_job_.attitude_was_ready&&
       (!attitude_ready_||preflight_job_.attitude_reference_ms!=last_attitude_.yaw_reference_ms)){
        const bool moving=preflight_job_.moving_envelope;
        cancelPreflight("cancelled_attitude_reset",false,"reference_invalid");
        if(moving)disableMovingCg("preflight_reference_invalid");
        return;
    }
    if(purpose!=PreflightPurpose::Preview){
        if(!imuFresh()){
            cancelPreflight("rejected_imu_stale",false,"imu_stale");return;
        }
        if(!servos_.armed()||servos_.faulted()){
            cancelPreflight("rejected_output_unhealthy",false,"output_unhealthy");return;
        }
        if(preflight_job_.moving_envelope&&!movingCgHealthy()){
            cancelPreflight("rejected_reference",false,"reference_invalid");
            disableMovingCg("preflight_reference_invalid");return;
        }
    }
    if(purpose==PreflightPurpose::DriveStart&&
       (!drive_session_.active()||drive_session_.epoch()!=preflight_job_.drive_epoch||
        drive_session_.remainingMs(clock_.millis())==0)){
        cancelPreflight("cancelled_lease",false,"lease_expired");return;
    }
    const std::uint32_t started = clock_.millis();
    PreflightJob& job = preflight_job_;
    if (job.stage < 3) {
        const std::uint32_t before = job.cursor.processedFrames();
        const bool done = job.cursor.step();
        const std::uint32_t evaluated=job.cursor.processedFrames()-before;
        preflight_frames_checked_ += evaluated;
        if (done) advancePreflightStage();
    } else {
        const float phase = job.dynamic_samples == 1 ? 0.0f :
            static_cast<float>(job.dynamic_index) / job.dynamic_samples;
        job.dynamic += job.dynamic_direction * job.moving_config.max_slew_mm_s *
            (motion_config_->update_ms * 0.001f);
        if (job.dynamic >= job.moving_config.max_abs_shift_mm) {
            job.dynamic = job.moving_config.max_abs_shift_mm;
            job.dynamic_direction = -1.0f;
        }
        if (job.dynamic <= -job.moving_config.max_abs_shift_mm) {
            job.dynamic = -job.moving_config.max_abs_shift_mm;
            job.dynamic_direction = 1.0f;
        }
        V10AdvancedMotionParameters test = job.advanced;
        test.dynamic_body_x_mm = job.dynamic;
        V10MotionFrame frame{};
        V10PreflightReport report = V10Gait(*motion_config_).frameAdvancedReport(
            job.mode, phase, job.parameters, test, job.centers, frame);
        ++preflight_frames_checked_;
        if (!report.ok) {
            job.combined = report;
            completePreflight();
        } else {
            if (job.active) {
                ++job.dynamic_index;
                if (job.dynamic_index >= job.dynamic_samples)
                    completePreflight();
            }
        }
    }
    const std::uint32_t elapsed = clock_.millis() - started;
    if (elapsed > preflight_max_slice_ms_) preflight_max_slice_ms_ = elapsed;
}

MotionParameterRecord RobotApp::makeMotionRecord(const V10MotionParameters& p) {
    MotionParameterRecord record{};
    record.magic=kMotionParameterMagic; record.version=kMotionParameterVersion;
    record.length=static_cast<std::uint16_t>(sizeof(record));
    record.values[0]=p.step_length_mm; record.values[1]=p.lift_mm;
    record.values[2]=p.period_s; record.values[3]=p.stand_down_mm;
    record.values[4]=p.stand_forward_mm; record.values[5]=p.walk_body_shift_mm;
    record.checksum=checksumRecord(record);
    return record;
}

bool RobotApp::decodeMotionRecord(const MotionParameterRecord& record,
                                  V10MotionParameters& p) {
    if(record.magic!=kMotionParameterMagic || record.version!=kMotionParameterVersion ||
       record.length!=sizeof(record) || record.checksum!=checksumRecord(record)) return false;
    p.step_length_mm=record.values[0]; p.lift_mm=record.values[1];
    p.period_s=record.values[2]; p.stand_down_mm=record.values[3];
    p.stand_forward_mm=record.values[4]; p.walk_body_shift_mm=record.values[5];
    return true;
}

ControlProfileRecord RobotApp::makeProfileRecord(const V10ControlProfile& p){
    ControlProfileRecord r{};r.magic=kControlProfileMagic;r.version=kControlProfileVersion;
    r.length=static_cast<std::uint16_t>(sizeof(r));r.flags=(p.cg_walk_enabled?1U:0U)|
        (p.trot_feedforward.enabled?2U:0U);
    const float values[23]={p.body.pitch_deg,p.body.roll_deg,p.body.body_x_mm,p.cg_shift_mm,
      p.planner.body_slew_mm_s,p.planner.target_tolerance_mm,static_cast<float>(p.planner.wait_timeout_ms),
      p.moving.height_mm,p.moving.angle_gain,p.moving.rate_gain_mm_per_rad_s,p.moving.filter_tau_s,
      p.moving.max_abs_shift_mm,p.moving.max_slew_mm_s,p.moving.min_dt_s,p.moving.max_dt_s,
      p.pitch_kp,p.pitch_kd_s,p.roll_kp,p.roll_kd_s,p.trot_feedforward.forward_gain,
      p.trot_feedforward.backward_gain,p.trot_feedforward.turn_gain,p.trot_feedforward.max_abs_mm};
    for(int i=0;i<23;++i)r.values[i]=values[i];
    r.checksum=checksumProfileRecord(r);return r;
}
bool RobotApp::decodeProfileRecord(const ControlProfileRecord& r,V10ControlProfile& p){
    if(r.magic!=kControlProfileMagic||r.version!=kControlProfileVersion||r.length!=sizeof(r)||
       (r.flags&~3U)!=0||r.checksum!=checksumProfileRecord(r))return false;
    for(float value:r.values)if(!std::isfinite(value))return false;
    if(r.values[6]<1000.0f||r.values[6]>30000.0f||std::floor(r.values[6])!=r.values[6])return false;
    p.body.pitch_deg=r.values[0];p.body.roll_deg=r.values[1];p.body.body_x_mm=r.values[2];
    p.cg_shift_mm=r.values[3];p.cg_walk_enabled=(r.flags&1U)!=0;
    p.planner.body_slew_mm_s=r.values[4];p.planner.target_tolerance_mm=r.values[5];
    p.planner.wait_timeout_ms=static_cast<std::uint32_t>(r.values[6]);
    p.moving.height_mm=r.values[7];p.moving.angle_gain=r.values[8];
    p.moving.rate_gain_mm_per_rad_s=r.values[9];p.moving.filter_tau_s=r.values[10];
    p.moving.max_abs_shift_mm=r.values[11];p.moving.max_slew_mm_s=r.values[12];
    p.moving.min_dt_s=r.values[13];p.moving.max_dt_s=r.values[14];
    p.pitch_kp=r.values[15];p.pitch_kd_s=r.values[16];
    p.roll_kp=r.values[17];p.roll_kd_s=r.values[18];
    p.trot_feedforward.enabled=(r.flags&2U)!=0;
    p.trot_feedforward.forward_gain=r.values[19];p.trot_feedforward.backward_gain=r.values[20];
    p.trot_feedforward.turn_gain=r.values[21];p.trot_feedforward.max_abs_mm=r.values[22];
    return validControlProfile(p);
}
bool RobotApp::validateControlProfile(const V10ControlProfile& p)const{
    if(!motion_config_||!validControlProfile(p))return false;
    V10AdvancedMotionParameters a=advanced_parameters_;a.body=p.body;
    a.cg_walk_enabled=p.cg_walk_enabled;a.cg_shift_mm=p.cg_shift_mm;
    a.dynamic_body_x_mm=0;a.cg_body_x_override_valid=false;
    std::uint8_t c[8];centers(c);
    return V10Gait(*motion_config_).preflightAdvanced(MotionMode::Stand,motion_parameters_,a,c).ok;
}
void RobotApp::applyControlProfile(const V10ControlProfile& p,const char* state,bool saved){
    control_profile_=p;advanced_parameters_.body=p.body;
    advanced_parameters_.cg_walk_enabled=p.cg_walk_enabled;
    advanced_parameters_.cg_shift_mm=p.cg_shift_mm;
    advanced_parameters_.trot_feedforward=p.trot_feedforward;
    advanced_parameters_.dynamic_body_x_mm=0;advanced_parameters_.cg_body_x_override_valid=false;
    cg_planner_=V10CgPlanner(p.planner);moving_cg_=V10MovingCg(p.moving);
    balance_pitch_=V10StaticBalance({p.pitch_kp,p.pitch_kd_s,1.5f,4.0f,0.005f,0.05f});
    balance_roll_=V10StaticBalance({p.roll_kp,p.roll_kd_s,1.5f,4.0f,0.005f,0.05f});
    disableBalance(true,"profile_applied_recapture_required");
    disableMovingCg("profile_applied");cg_planner_.clear();
    control_profile_state_=state;control_profile_saved_=saved;
}

bool RobotApp::validateAppliedParameters(const V10MotionParameters& p) const {
    if(!motion_config_) return false;
    std::uint8_t c[8]; centers(c);
    V10Gait gait(*motion_config_);
    return gait.parametersValid(p) &&
        gait.preflightAdvanced(MotionMode::Stand,p,advanced_parameters_,c).ok;
}

DebugReply RobotApp::armAndStand() {
    if (!motion_config_) return {false,"V10 motion configuration unavailable"};
    if (!imuFresh()) return {false,"IMU not healthy; enable-and-stand denied"};
    if (servos_.armed() || motionActive() || servos_.calibrating() ||
        servos_.activeChannel() >= 0)
        return {false,"Already enabled or output active; request rejected with no PWM"};
    std::uint8_t c[8]; centers(c);
    V10Gait gait(*motion_config_); V10MotionFrame target{};
    last_preview_=preview(MotionMode::Stand);
    preview_error_=preflightErrorName(last_preview_.error);
    preflight_stage_="complete";preflight_frames_checked_=1;
    preflight_max_slice_ms_=preflight_duration_ms_;
    preflight_result_=last_preview_.ok?"passed":"rejected";
    if (!last_preview_.ok)
        return {false,"Stand preflight failed; robot remains disabled"};
    if(!gait.frameAdvanced(MotionMode::Stand,0,motion_parameters_,advanced_parameters_,c,target)){
        preview_error_="initial_frame_invalid";preflight_result_="rejected_initial_frame";
        return {false,"Stand preflight failed; robot remains disabled"};
    }
    if (!servos_.arm()) return {false,"PCA fault; robot remains disabled"};
    motion_requested_=MotionMode::Stand; motion_pending_=true;
    motion_request_sample_generation_=imu_sample_generation_; motion_error_="none";
    return {true,"Enabled and Stand queued; first centre frame waits for a fresh IMU tick"};
}

void RobotApp::cancelMotion(const char* error, MotionMode mode) {
    if(preflight_job_.active)cancelPreflight("cancelled_motion_state");
    motion_pending_ = false;
    motion_transition_ = false;
    motion_mode_ = mode;
    motion_error_ = error;
}

bool RobotApp::writeMotionFrame(const V10MotionFrame& frame) {
    const ServoResult result = servos_.coordinatedFrame(
        motion_config_->channels, frame.pulse_us, 8);
    if (result != ServoResult::Ok) {
        cancelMotion(result == ServoResult::HardwareFault ? "pwm_write_failed" :
                     "coordinated_mode_rejected", MotionMode::HoldError);
        stopDriveSession("pwm_write_failed");disableMovingCg("pwm_write_failed");
        return false;
    }
    motion_frame_ = frame;
    motion_frame_valid_ = true;
    ++motion_frame_count_;
    return true;
}

DebugReply RobotApp::requestMotion(MotionMode mode) {
    return requestMotionWithAdvanced(mode,advanced_parameters_);
}

DebugReply RobotApp::requestMotionWithAdvanced(
        MotionMode mode,const V10AdvancedMotionParameters& candidate,
        PreflightPurpose purpose,std::uint32_t drive_epoch) {
    if (!motion_config_) return {false,"V10 motion configuration unavailable"};
    if (!imuFresh()) return {false,"IMU not healthy; motion denied"};
    if (!servos_.armed()) return {false,"Use ARM first; motion never arms implicitly"};
    if (preflight_job_.active) return {false,"Another preflight is already checking"};
    if (servos_.calibrating() || servos_.activeChannel() >= 0)
        return {false,"Single/calibration output active; use global OFF then ARM"};
    if(!movingCgHealthy()) {
        disableMovingCg("moving_cg_unhealthy_before_transition");
        stopDriveSession("moving_cg_unhealthy_before_transition");
        motion_pending_=false;motion_transition_=false;cg_planner_.clear();
        motion_mode_=motion_frame_valid_?MotionMode::HoldError:MotionMode::Idle;
        motion_error_="moving_cg_unhealthy_before_transition";
        return {false,"Moving CG attitude/reference invalid; transition not started"};
    }
    if (!queuePreflight(mode,candidate,purpose,drive_epoch))
        return {false,"Motion preflight request rejected before checking"};
    return {true,"Motion preflight queued; no PWM sent; watch preview status"};
}

void RobotApp::motionTick() {
    if (!motion_config_ || (!motion_pending_ && !motion_transition_ &&
        motion_mode_ != MotionMode::Walk && motion_mode_ != MotionMode::Trot &&
        !(motion_mode_ >= MotionMode::Leg1 && motion_mode_ <= MotionMode::Leg4))) return;
    const std::uint32_t now = clock_.millis();
    if (!imuFresh() || !servos_.armed() || servos_.faulted()) {
        (void)servos_.off();
        cancelMotion("imu_or_arm_lost",MotionMode::HoldError);
        return;
    }
    if((motion_pending_||motion_transition_)&&!movingCgHealthy()) {
        cancelMotion("moving_cg_invalid_during_transition",motion_frame_valid_?MotionMode::HoldError:MotionMode::Idle);
        stopDriveSession("moving_cg_invalid_transition");
        disableMovingCg("moving_cg_invalid_transition");return;
    }
    V10Gait gait(*motion_config_);
    std::uint8_t c[8]; centers(c);
    if (motion_pending_) {
        // A start request only plans.  Its first frame is gated on an IMU
        // sample acquired by a later scheduler tick, never the sample that
        // happened to be fresh while the command was parsed.
        if (imu_sample_generation_ == motion_request_sample_generation_) return;
        V10AdvancedMotionParameters runtime_advanced=advanced_parameters_;
        runtime_advanced.dynamic_body_x_mm=moving_cg_shift_mm_;
        if (!gait.frameAdvanced(motion_requested_,0,motion_parameters_,runtime_advanced,
                                c,motion_transition_to_)) {
            cancelMotion("initial_frame_invalid",MotionMode::HoldError); return;
        }
        if (!motion_frame_valid_ || !servos_.coordinated()) {
            V10MotionFrame neutral{};
            for (int leg=0;leg<4;++leg) {
                neutral.forward_mm[leg]=motion_parameters_.stand_forward_mm;
                neutral.down_mm[leg]=motion_parameters_.stand_down_mm;
            }
            for (int i=0;i<8;++i) {
                neutral.servo_deg[i]=c[i];
                neutral.pulse_us[i]=static_cast<std::uint16_t>(500+(2000*c[i]+90)/180);
            }
            if (!writeMotionFrame(neutral)) return;
        }
        motion_transition_from_=motion_frame_;
        motion_pending_=false;
        motion_transition_=true;
        motion_mode_=motion_requested_;
        motion_transition_start_ms_=now;
        motion_last_tick_ms_=now;
        return;
    }
    const std::uint32_t gap=now-motion_last_tick_ms_;
    if (gap >= motion_config_->update_ms) {
        motion_last_update_gap_ms_=gap;
        if(gap>motion_max_update_gap_ms_) motion_max_update_gap_ms_=gap;
        const std::uint32_t intervals=gap/motion_config_->update_ms;
        if(intervals>1) motion_missed_deadlines_+=intervals-1;
    }
    if (gap > motion_config_->max_tick_gap_ms) {
        cancelMotion("scheduler_gap_gt_200ms",MotionMode::HoldError); return;
    }
    if (gap < motion_config_->update_ms) return;
    motion_last_tick_ms_=now;
    if (motion_transition_) {
        float t=static_cast<float>(now-motion_transition_start_ms_)/
            (motion_config_->transition_s*1000.0f);
        if (t>1) t=1;
        const float eased=smooth(t);
        V10MotionFrame f=motion_transition_to_;
        for (int i=0;i<8;++i) {
            f.servo_deg[i]=motion_transition_from_.servo_deg[i]+
                (motion_transition_to_.servo_deg[i]-motion_transition_from_.servo_deg[i])*eased;
            f.pulse_us[i]=static_cast<std::uint16_t>(500+std::floor(2000*f.servo_deg[i]/180+0.5f));
        }
        for (int i=0;i<4;++i) {
            f.forward_mm[i]=motion_transition_from_.forward_mm[i]+
                (motion_transition_to_.forward_mm[i]-motion_transition_from_.forward_mm[i])*eased;
            f.down_mm[i]=motion_transition_from_.down_mm[i]+
                (motion_transition_to_.down_mm[i]-motion_transition_from_.down_mm[i])*eased;
        }
        if (!writeMotionFrame(f)) return;
        if (t>=1) {
            motion_transition_=false;
            motion_phase_start_ms_=now;
            motion_nominal_elapsed_s_=motion_actual_elapsed_s_=0;
            cg_planner_.clear();cg_plan_=V10CgPlanOutput{};
            if(moving_cg_enabled_&&
               (motion_mode_==MotionMode::Walk||motion_mode_==MotionMode::Trot)) {
                const float pitch=V10StaticBalance::bodyPitchFromSensor(last_attitude_.pitch_deg);
                if(!attitude_ready_||last_attitude_.yaw_reference_ms!=balance_reference_attitude_ms_||
                   !moving_cg_.rebase(pitch,last_attitude_.timestamp_ms,
                                      last_attitude_.yaw_reference_ms)) {
                    cancelMotion("moving_cg_rebase_failed",MotionMode::HoldError);
                    stopDriveSession("moving_cg_rebase_failed");
                    disableMovingCg("moving_cg_rebase_failed");return;
                }
                moving_cg_last_attitude_ms_=last_attitude_.timestamp_ms;
            }
            if(motion_mode_==MotionMode::Walk&&advanced_parameters_.cg_walk_enabled){
                float front=0,rear=0;
                if(!cgTargets(front,rear)){cancelMotion("cg_targets_invalid",MotionMode::HoldError);return;}
                cg_planner_.begin(advanced_parameters_.body.body_x_mm,front,rear);
                if(!cg_planner_.active()){cancelMotion("cg_planner_invalid",MotionMode::HoldError);return;}
            }
            if (motion_mode_==MotionMode::Stand) motion_error_="none";
        }
        return;
    }
    if(!movingCgUpdate()) {
        motion_pending_=false;motion_transition_=false;
        motion_mode_=motion_frame_valid_?MotionMode::HoldError:MotionMode::Idle;
        stopDriveSession("moving_cg_fault");return;
    }
    float phase=std::fmod(static_cast<float>(now-motion_phase_start_ms_)/
                          (motion_parameters_.period_s*1000.0f),1.0f);
    V10AdvancedMotionParameters runtime_advanced=advanced_parameters_;
    runtime_advanced.dynamic_body_x_mm=moving_cg_shift_mm_;
    motion_actual_elapsed_s_+=gap*0.001f;
    motion_nominal_elapsed_s_=motion_actual_elapsed_s_;
    if(motion_mode_==MotionMode::Walk&&advanced_parameters_.cg_walk_enabled){
        float front=0,rear=0;
        if(!cgTargets(front,rear)||!cg_planner_.tick(gap*0.001f,motion_parameters_.period_s,
             advanced_parameters_.body.body_x_mm,front,rear,cg_plan_)){
            cancelMotion("cg_wait_timeout_or_invalid",MotionMode::HoldError);
            stopDriveSession("cg_wait_timeout");disableMovingCg("cg_wait_timeout");return;
        }
        phase=cg_plan_.phase;motion_nominal_elapsed_s_=cg_plan_.nominal_elapsed_s;
        motion_actual_elapsed_s_=cg_plan_.actual_elapsed_s;
        runtime_advanced.cg_body_x_override_valid=true;
        runtime_advanced.cg_body_x_override_mm=cg_plan_.body_x_mm;
    }
    V10MotionFrame f{};
    const V10PreflightReport runtime_report=gait.frameAdvancedReport(
        motion_mode_,phase,motion_parameters_,runtime_advanced,c,f);
    if (!runtime_report.ok) {
        cancelMotion("runtime_frame_invalid",MotionMode::HoldError); return;
    }
    (void)writeMotionFrame(f);
}

void RobotApp::disableBalance(bool clear_reference,const char* reason) {
    if(clear_reference)cancelPreflight("cancelled_reference_reset");
    balance_capture_pending_=false;
    balance_enable_pending_=false;
    balance_enabled_=false;
    balance_pitch_.clear();balance_roll_.clear();
    balance_pitch_correction_deg_=balance_roll_correction_deg_=0.0f;
    balance_capture_count_=0;
    if(clear_reference) {
        balance_reference_ready_=false;
        balance_reference_pitch_deg_=balance_reference_roll_deg_=0.0f;
        balance_reference_attitude_ms_=0;
    }
    balance_error_=reason;
}

void RobotApp::stopDriveSession(const char* reason) {
    if(preflight_job_.active&&
       preflight_job_.purpose==PreflightPurpose::DriveStart)
        cancelPreflight(reason,true);
    drive_session_.invalidate();
    drive_session_mode_=MotionMode::Idle;
    drive_session_reason_=reason;
    advanced_parameters_.drive=V10DriveIntent{};
}

void RobotApp::disableMovingCg(const char* reason) {
    if(preflight_job_.active)cancelPreflight("cancelled_moving_cg_change");
    moving_cg_enable_pending_=false;moving_cg_enabled_=false;
    moving_cg_.clear();moving_cg_shift_mm_=0;moving_cg_last_attitude_ms_=0;
    moving_cg_error_=reason;
}

bool RobotApp::cgTargets(float& front,float& rear) const {
    float left=0,right=0;
    if(!driveScales(advanced_parameters_.drive,left,right))return false;
    const float s_left=motion_parameters_.step_length_mm*left;
    const float s_right=motion_parameters_.step_length_mm*right;
    front=motion_parameters_.stand_forward_mm+std::fmin(0.0f,s_left*0.5f)-
          advanced_parameters_.cg_shift_mm;
    rear=motion_parameters_.stand_forward_mm+
         std::fmax(s_right*0.5f,(s_left+s_right)*0.5f)+
         advanced_parameters_.cg_shift_mm;
    return std::isfinite(front)&&std::isfinite(rear);
}

bool RobotApp::movingCgUpdate() {
    if(!moving_cg_enable_pending_&&!moving_cg_enabled_)return true;
    if(!imuFresh()||!attitude_ready_||!balance_reference_ready_||
       last_attitude_.yaw_reference_ms!=balance_reference_attitude_ms_) {
        disableMovingCg("moving_cg_reference_or_imu_lost");return false;
    }
    const float body_pitch=V10StaticBalance::bodyPitchFromSensor(last_attitude_.pitch_deg);
    if(!std::isfinite(body_pitch)||
       std::fabs(body_pitch-balance_reference_pitch_deg_)>8.0f) {
        disableMovingCg("moving_cg_tilt_or_nonfinite");return false;
    }
    if(moving_cg_enable_pending_) {
        if(imu_sample_generation_==moving_cg_enable_generation_)return true;
        moving_cg_.start(balance_reference_pitch_deg_,body_pitch,last_attitude_.timestamp_ms,
                         last_attitude_.yaw_reference_ms);
        if(!moving_cg_.referenceValid()){disableMovingCg("moving_cg_start_failed");return false;}
        moving_cg_enable_pending_=false;moving_cg_enabled_=true;
        moving_cg_last_attitude_ms_=last_attitude_.timestamp_ms;
        moving_cg_error_="none";return true;
    }
    if(motion_mode_!=MotionMode::Walk&&motion_mode_!=MotionMode::Trot)return true;
    if(last_attitude_.timestamp_ms==moving_cg_last_attitude_ms_)return true;
    float shift=0;
    if(!moving_cg_.update(body_pitch,last_attitude_.timestamp_ms,
                          last_attitude_.yaw_reference_ms,shift)) {
        disableMovingCg("moving_cg_dt_or_reference_invalid");return false;
    }
    moving_cg_shift_mm_=shift;moving_cg_last_attitude_ms_=last_attitude_.timestamp_ms;
    return true;
}

bool RobotApp::movingCgHealthy() const {
    if(!moving_cg_enabled_&&!moving_cg_enable_pending_)return true;
    if(!imuFresh()||!attitude_ready_||!balance_reference_ready_||
       last_attitude_.yaw_reference_ms!=balance_reference_attitude_ms_)return false;
    const float pitch=V10StaticBalance::bodyPitchFromSensor(last_attitude_.pitch_deg);
    return std::isfinite(pitch)&&std::fabs(pitch-balance_reference_pitch_deg_)<=8.0f;
}

bool RobotApp::balanceEnvelopeValid() const {
    if(!motion_config_||!balance_reference_ready_)return false;
    std::uint8_t c[8];centers(c);V10Gait gait(*motion_config_);
    const float pitch_delta[2]={-1.5f,1.5f};
    const float roll_delta[2]={balance_roll_sensor_sign_==0?0.0f:-1.5f,
                               balance_roll_sensor_sign_==0?0.0f:1.5f};
    for(float pd:pitch_delta)for(float rd:roll_delta) {
        V10AdvancedMotionParameters a=advanced_parameters_;
        a.body.pitch_deg+=pd;a.body.roll_deg+=rd;
        if(!gait.preflightAdvanced(MotionMode::Stand,motion_parameters_,a,c).ok)return false;
    }
    return true;
}

bool RobotApp::currentStandMatchesApplied() const {
    if(!motion_config_||!motion_frame_valid_||motion_mode_!=MotionMode::Stand||
       motion_pending_||motion_transition_)return false;
    std::uint8_t c[8];centers(c);V10MotionFrame expected{};
    if(!V10Gait(*motion_config_).frameAdvanced(MotionMode::Stand,0,motion_parameters_,
                                               advanced_parameters_,c,expected))return false;
    for(int i=0;i<8;++i)
        if(std::fabs(expected.servo_deg[i]-motion_frame_.servo_deg[i])>0.05f)return false;
    return true;
}

void RobotApp::balanceCaptureTick() {
    if(!balance_capture_pending_||!attitude_ready_)return;
    if(motion_pending_||motion_transition_||motion_mode_!=MotionMode::Stand||
       !servos_.armed()||!servos_.coordinated()) {
        disableBalance(true,"capture_state_lost");return;
    }
    const std::uint32_t timestamp=last_attitude_.timestamp_ms;
    if(timestamp==balance_capture_last_ms_)return;
    if(last_attitude_.yaw_reference_ms==0) {
        disableBalance(true,"capture_attitude_reference_invalid");return;
    }
    const float pitch=V10StaticBalance::bodyPitchFromSensor(last_attitude_.pitch_deg);
    const float sensor_roll=last_attitude_.roll_deg;
    if(!std::isfinite(pitch)||!std::isfinite(sensor_roll)) {
        disableBalance(true,"capture_nonfinite");return;
    }
    if(balance_capture_count_>0) {
        const std::uint32_t dt=timestamp-balance_capture_last_ms_;
        if(dt<5)return;
        if(last_attitude_.yaw_reference_ms!=balance_reference_attitude_ms_) {
            disableBalance(true,"capture_attitude_reference_changed");return;
        }
        const float dt_s=dt*0.001f;
        if(dt>50||std::fabs(pitch-balance_capture_previous_pitch_)/dt_s>2.0f||
           std::fabs(sensor_roll-balance_capture_previous_roll_)/dt_s>2.0f) {
            balance_capture_count_=0;balance_capture_pitch_sum_=0;
            balance_capture_roll_sum_=0;
        }
    } else {
        balance_reference_attitude_ms_=last_attitude_.yaw_reference_ms;
        balance_capture_pitch_min_=balance_capture_pitch_max_=pitch;
        balance_capture_roll_min_=balance_capture_roll_max_=sensor_roll;
    }
    if(balance_capture_count_==0) {
        balance_reference_attitude_ms_=last_attitude_.yaw_reference_ms;
        balance_capture_pitch_min_=balance_capture_pitch_max_=pitch;
        balance_capture_roll_min_=balance_capture_roll_max_=sensor_roll;
    } else {
        balance_capture_pitch_min_=std::fmin(balance_capture_pitch_min_,pitch);
        balance_capture_pitch_max_=std::fmax(balance_capture_pitch_max_,pitch);
        balance_capture_roll_min_=std::fmin(balance_capture_roll_min_,sensor_roll);
        balance_capture_roll_max_=std::fmax(balance_capture_roll_max_,sensor_roll);
        if(balance_capture_pitch_max_-balance_capture_pitch_min_>0.6f||
           balance_capture_roll_max_-balance_capture_roll_min_>0.6f) {
            balance_capture_count_=0;balance_capture_pitch_sum_=0;
            balance_capture_roll_sum_=0;
            balance_capture_pitch_min_=balance_capture_pitch_max_=pitch;
            balance_capture_roll_min_=balance_capture_roll_max_=sensor_roll;
        }
    }
    balance_capture_previous_pitch_=pitch;
    balance_capture_previous_roll_=sensor_roll;
    balance_capture_last_ms_=timestamp;
    balance_capture_pitch_sum_+=pitch;
    balance_capture_roll_sum_+=sensor_roll;
    ++balance_capture_count_;
    if(balance_capture_count_>=20) {
        balance_reference_pitch_deg_=balance_capture_pitch_sum_/balance_capture_count_;
        balance_reference_roll_deg_=balance_roll_sensor_sign_==0?0.0f:
            balance_roll_sensor_sign_*balance_capture_roll_sum_/balance_capture_count_;
        balance_capture_pending_=false;balance_reference_ready_=true;
        balance_error_="none";
    }
}

void RobotApp::balanceTick() {
    if(!balance_enable_pending_&&!balance_enabled_)return;
    if(!imuFresh()||!attitude_ready_||motion_pending_||motion_transition_||
       motion_mode_!=MotionMode::Stand||!servos_.armed()||!servos_.coordinated()) {
        (void)servos_.off();disableBalance(true,"balance_state_or_imu_lost");
        cancelMotion("balance_state_or_imu_lost",MotionMode::HoldError);
        motion_frame_valid_=false;return;
    }
    if(balance_enable_pending_) {
        if(imu_sample_generation_==balance_enable_sample_generation_)return;
        const float pitch=V10StaticBalance::bodyPitchFromSensor(last_attitude_.pitch_deg);
        const float roll=balance_roll_sensor_sign_==0?0.0f:
            balance_roll_sensor_sign_*last_attitude_.roll_deg;
        if(last_attitude_.yaw_reference_ms!=balance_reference_attitude_ms_) {
            (void)servos_.off();disableBalance(true,"attitude_reference_changed");
            cancelMotion("attitude_reference_changed",MotionMode::HoldError);
            motion_frame_valid_=false;return;
        }
        balance_pitch_.start(balance_reference_pitch_deg_,pitch,last_attitude_.timestamp_ms,
                             last_attitude_.yaw_reference_ms);
        if(balance_roll_sensor_sign_!=0)
            balance_roll_.start(balance_reference_roll_deg_,roll,last_attitude_.timestamp_ms,
                                last_attitude_.yaw_reference_ms);
        if(!balance_pitch_.referenceValid() ||
           (balance_roll_sensor_sign_!=0&&!balance_roll_.referenceValid())) {
            disableBalance(true,"balance_start_failed");return;
        }
        balance_enable_pending_=false;balance_enabled_=true;
        balance_last_update_ms_=last_attitude_.timestamp_ms;balance_error_="none";
        return;
    }
    const std::uint32_t timestamp=last_attitude_.timestamp_ms;
    const std::uint32_t elapsed=timestamp-balance_last_update_ms_;
    if(elapsed<20)return;
    const float pitch=V10StaticBalance::bodyPitchFromSensor(last_attitude_.pitch_deg);
    const float roll=balance_roll_sensor_sign_==0?0.0f:
        balance_roll_sensor_sign_*last_attitude_.roll_deg;
    if(std::fabs(pitch-balance_reference_pitch_deg_)>8.0f||
       (balance_roll_sensor_sign_!=0&&std::fabs(roll-balance_reference_roll_deg_)>8.0f)) {
        (void)servos_.off();disableBalance(true,"tilt_limit_exceeded");
        cancelMotion("tilt_limit_exceeded",MotionMode::HoldError);
        motion_frame_valid_=false;return;
    }
    float pitch_correction=0,roll_correction=0;
    if(!balance_pitch_.update(pitch,timestamp,last_attitude_.yaw_reference_ms,
                              pitch_correction)||
       (balance_roll_sensor_sign_!=0&&!balance_roll_.update(
           roll,timestamp,last_attitude_.yaw_reference_ms,roll_correction))) {
        (void)servos_.off();disableBalance(true,"balance_dt_or_reference_invalid");
        cancelMotion("balance_dt_or_reference_invalid",MotionMode::HoldError);
        motion_frame_valid_=false;return;
    }
    V10AdvancedMotionParameters applied=advanced_parameters_;
    applied.body.pitch_deg+=pitch_correction;
    applied.body.roll_deg+=roll_correction;
    std::uint8_t c[8];centers(c);V10MotionFrame frame{};
    if(!V10Gait(*motion_config_).frameAdvanced(MotionMode::Stand,0,motion_parameters_,
                                               applied,c,frame)) {
        (void)servos_.off();disableBalance(true,"balance_frame_invalid");
        cancelMotion("balance_frame_invalid",MotionMode::HoldError);
        motion_frame_valid_=false;return;
    }
    if(!writeMotionFrame(frame)) {disableBalance(true,"balance_pwm_failed");return;}
    balance_pitch_correction_deg_=pitch_correction;
    balance_roll_correction_deg_=roll_correction;
    balance_last_update_ms_=timestamp;
}

const RelativeServoLimit* RobotApp::relativeLimit(std::uint8_t channel) const {
    if (!relative_limits_) return nullptr;
    for (std::size_t i = 0; i < relative_count_; ++i)
        if (relative_limits_[i].channel == channel) return &relative_limits_[i];
    return nullptr;
}

const V10ShankMapConfig* RobotApp::shankMapConfig(std::uint8_t channel) const {
    if (!shank_maps_) return nullptr;
    for (std::size_t i = 0; i < shank_map_count_; ++i) {
        const V10ShankMapConfig& config = shank_maps_[i];
        if (config.channel == channel && config.point_count <= 3 &&
            V10ShankLinearMap(config.points, config.point_count).valid()) return &config;
    }
    return nullptr;
}

bool RobotApp::shankRange(std::uint8_t channel, float center,
                          std::int8_t& direction, float& max_u,
                          float& max_q) const {
    const RelativeServoLimit* limit = relativeLimit(channel);
    const V10ShankMapConfig* config = shankMapConfig(channel);
    float lower = 0, upper = 0;
    if (!limit || !config || !configuredShankDirection(*limit, direction) ||
        !relativeServoBounds(center, limit->min_offset_deg, limit->max_offset_deg,
                             lower, upper))
        return false;
    const V10ShankLinearMap map(config->points, config->point_count);
    const float directional_u = direction < 0 ? -lower : upper;
    if (!std::isfinite(directional_u) || directional_u < 0.0f) return false;
    const float available_u = std::fmin(map.maxU(), directional_u);
    float available_q = 0;
    if (!map.qFromU(available_u, available_q)) return false;
    max_u = available_u;
    max_q = available_q;
    return true;
}

void RobotApp::begin() {
    console_.line("Applepy_v1 V10 adapted V7.3 control firmware 0.8.14");
    calibration_session_pending_ = false;
    preflight_job_ = PreflightJob{};
    ++preflight_generation_;
    preflight_stage_="idle";preflight_result_="not_run";
    preflight_frames_checked_=preflight_max_slice_ms_=0;
    cancelMotion("none",MotionMode::Idle);
    motion_frame_valid_=false;
    advanced_parameters_=V10AdvancedMotionParameters{};
    advanced_parameter_revision_=0;
    balance_roll_sensor_sign_=kDefaultRollSensorSign;
    disableBalance(true,"disabled_on_boot");
    drive_session_mode_=MotionMode::Idle;drive_session_reason_="boot";
    straight_trim_=kDefaultStraightTrim;
    trot_pitch_revision_=0;
    advanced_parameters_.drive=V10DriveIntent{};
    disableMovingCg("disabled_on_boot");
    cg_planner_.clear();cg_plan_=V10CgPlanOutput{};
    const bool pwm_ok = servos_.begin();
    console_.line(pwm_ok ? "PCA9685 0x40 ready; all channels OFF"
                         : "FAULT: PCA9685 unavailable; outputs may NOT be safe");
    imu_ready_ = imu_.begin();
    sample_ready_ = imu_ready_ && imu_.read(last_sample_);
    if (sample_ready_) ++imu_sample_generation_;
    if (!sample_ready_) imu_ready_ = false;
    attitude_estimator_.reset();
    attitude_ready_ = sample_ready_ &&
                      attitude_estimator_.update(last_sample_, last_attitude_);
    console_.line(imu_ready_ ? "HXY IMU 0x19 ID=0x6A ready"
                             : "FAULT: HXY IMU 0x19 missing/read failed; ARM disabled");
    calibration_.begin(); // Loads centre values only; never moves a servo.
    motion_parameter_state_="built_in_default";
    motion_parameters_saved_=false;
    saved_motion_parameters_valid_=false;
    if (motion_store_) {
        MotionParameterRecord record{}; V10MotionParameters loaded{};
        const MotionStoreResult result=motion_store_->load(record);
        if(result==MotionStoreResult::Ok && decodeMotionRecord(record,loaded) &&
           validateAppliedParameters(loaded)) {
            motion_parameters_=loaded; saved_motion_parameters_=loaded;
            saved_motion_parameters_valid_=true; motion_parameters_saved_=true;
            motion_parameter_state_="loaded_nvs";
        } else if(result==MotionStoreResult::Missing) {
            motion_parameter_state_="nvs_missing_using_default";
        } else if(result==MotionStoreResult::IoError) {
            motion_parameter_state_="nvs_load_failed_using_default";
        } else {
            motion_parameter_state_="nvs_invalid_using_default";
        }
    }
    control_profile_=V10ControlProfile{};saved_control_profile_valid_=false;
    control_profile_saved_=false;control_profile_state_="built_in_default";
    if(profile_store_){
        ControlProfileRecord record{};V10ControlProfile loaded{};
        const ControlProfileStoreResult result=profile_store_->load(record);
        if(result==ControlProfileStoreResult::Ok&&decodeProfileRecord(record,loaded)&&
           validateControlProfile(loaded)){
            saved_control_profile_=loaded;saved_control_profile_valid_=true;
            applyControlProfile(loaded,"loaded_nvs",true);
        }else if(result==ControlProfileStoreResult::Missing)
            applyControlProfile(control_profile_,"nvs_missing_using_default",false);
        else if(result==ControlProfileStoreResult::IoError)
            applyControlProfile(control_profile_,"nvs_load_failed_using_default",false);
        else applyControlProfile(control_profile_,"nvs_invalid_using_default",false);
    }else applyControlProfile(control_profile_,"storage_unavailable_using_default",false);
    last_poll_ms_ = clock_.millis();
    console_.line("Type help. Servo power must be current-limited; no hardware E-stop/OE on this PCB.");
}

void RobotApp::tick() {
    const bool was_armed = servos_.armed();
    const bool was_faulted = servos_.faulted();
    servos_.tick();
    if (!was_faulted && servos_.faulted())
        console_.line("FAULT: PWM shutdown I2C failed; physically disconnect servo power NOW");
    else if (was_armed && !servos_.armed() && !servos_.manualHold())
        console_.line("Servo command timeout; all PWM off (I2C acknowledged)");
    const std::uint32_t now = clock_.millis();
    (void)drive_session_.expire(now);
    if(drive_session_.consumeExpired()) {
        cancelPreflight("cancelled_lease_expired",true);
        drive_session_mode_=MotionMode::Idle;drive_session_reason_="lease_expired_hold";
        advanced_parameters_.drive=V10DriveIntent{};
        motion_pending_=false;motion_transition_=false;
        motion_mode_=motion_frame_valid_?MotionMode::Hold:MotionMode::Idle;
        disableMovingCg("lease_expired");cg_planner_.clear();
    }
    if (imu_ready_ && static_cast<std::uint32_t>(now - last_poll_ms_) >= 10) {
        last_poll_ms_ = now;
        ImuSample sample{};
        if (!imu_.read(sample)) {
            imu_ready_ = false;
            sample_ready_ = false;
            attitude_ready_ = false;
            attitude_estimator_.reset();
            cancelPreflight("cancelled_imu_failure",true);
            disableBalance(true,"imu_read_failed");
            stopDriveSession("imu_read_failed");disableMovingCg("imu_read_failed");
            const bool off_ok = servos_.off();
            cancelMotion("imu_read_failed",MotionMode::HoldError);
            motion_frame_valid_=false;
            console_.line(off_ok ? "FAULT: IMU read failed; PWM off acknowledged, reboot required"
                                 : "FAULT: IMU and PWM shutdown I2C failed; disconnect servo power NOW");
        } else {
            last_sample_ = sample;
            sample_ready_ = true;
            ++imu_sample_generation_;
            attitude_ready_ = attitude_estimator_.update(sample, last_attitude_);
        }
    }
    motionTick();
    balanceCaptureTick();
    balanceTick();
    if(moving_cg_enable_pending_&&!movingCgUpdate())
        moving_cg_error_="enable_failed";
    preflightTick();
}

bool RobotApp::imuFresh() const {
    return imu_ready_ && sample_ready_ &&
           static_cast<std::uint32_t>(clock_.millis() - last_sample_.timestamp_ms) <= 100;
}

void RobotApp::status() {
    const DebugSnapshot s = snapshot();
    char text[120];
    if (s.calibrating)
        std::snprintf(text, sizeof(text),
                      "PCA=%s ARMED=yes FAULT=%s IMU=%s MODE=calibration MASK=%04X",
                      s.pca ? "ready" : "failed", s.fault ? "yes" : "no",
                      s.imu_ok ? "fresh" : "unavailable",
                      static_cast<unsigned>(s.calibration_mask));
    else if (s.manual_hold)
        std::snprintf(text, sizeof(text),
                      "PCA=%s ARMED=%s FAULT=%s IMU=%s ACTIVE=%d MODE=manual-until-OFF",
                      s.pca ? "ready" : "failed", s.armed ? "yes" : "no",
                      s.fault ? "yes" : "no", s.imu_ok ? "fresh" : "unavailable",
                      s.active_channel);
    else
        std::snprintf(text, sizeof(text),
                      "PCA=%s ARMED=%s FAULT=%s IMU=%s ACTIVE=%d REMAIN=%lu ms",
                      s.pca ? "ready" : "failed", s.armed ? "yes" : "no",
                      s.fault ? "yes" : "no", s.imu_ok ? "fresh" : "unavailable",
                      s.active_channel, static_cast<unsigned long>(s.remaining_ms));
    char range_text[100];
    std::snprintf(range_text, sizeof(range_text), "PULSE_RANGE=%u..%u us EXTENDED=%s (volatile unlock)",
                  static_cast<unsigned>(s.pulse_min_us),
                  static_cast<unsigned>(s.pulse_max_us), s.extended_range ? "yes" : "no");
    console_.line(range_text);
    console_.line(text);
    if (s.fault)
        console_.line("FAULT: output may still be active; physically disconnect servo power NOW");
}

void RobotApp::imuReport() {
    if (!imuFresh()) {
        console_.line("ERR: no fresh IMU sample");
        return;
    }
    char text[180];
    std::snprintf(text, sizeof(text),
                  "IMU t=%lu ms a[m/s2]=%.3f,%.3f,%.3f w[rad/s]=%.3f,%.3f,%.3f",
                  static_cast<unsigned long>(last_sample_.timestamp_ms),
                  last_sample_.accel_mps2[0], last_sample_.accel_mps2[1],
                  last_sample_.accel_mps2[2], last_sample_.gyro_radps[0],
                  last_sample_.gyro_radps[1], last_sample_.gyro_radps[2]);
    console_.line(text);
}

void RobotApp::attitudeReport() {
    const DebugSnapshot s = snapshot();
    if (!s.attitude_ok) {
        char text[130];
        if (s.gyro_calibrated)
            std::snprintf(text, sizeof(text),
                          "RPY unavailable after gyro calibration; check IMU/accel norm %.2f m/s2",
                          s.accel_norm_mps2);
        else
            std::snprintf(text, sizeof(text),
                          "RPY calibrating %u/%u; keep board still, accel norm %.2f m/s2",
                          static_cast<unsigned>(s.attitude_calibration_samples),
                          static_cast<unsigned>(s.attitude_calibration_required),
                          s.accel_norm_mps2);
        console_.line(text);
        return;
    }
    char text[140];
    std::snprintf(text, sizeof(text),
                  "RPY sensor frame [deg] roll=%.2f pitch=%.2f yaw=%.2f (relative, drifts)",
                  s.attitude.roll_deg, s.attitude.pitch_deg, s.attitude.yaw_deg);
    console_.line(text);
}

void RobotApp::onLine(const char* line) {
    const Command cmd = parseCommand(line);
    switch (cmd.type) {
    case CommandType::Help:
        console_.line("help | status | imu | rpy | arm | pwm <ch> <us> | nudge <ch> +1/-1");
        console_.line("offset <ch> <signed_deg>: current calibrated center + offset; does NOT change center");
        console_.line("shank <ch> <q_deg>: mapped shank channels use current calibrated neutral and configured direction");
        console_.line("Board calibration input is 500..2500us / nominal0..180deg; check status for active range.");
        console_.line("Legacy range normal/extended: global OFF first; default on this board is already full range.");
        console_.line("Electrical range is NOT a mechanical limit; start near center, small steps, physical power cutoff ready.");
        console_.line("cal_start (eight stored centers, default 90 deg) | cal_save_all (OFF then save 8)");
        console_.line("off <ch> (single mode) | center <ch> | save <ch> (after OFF)");
        console_.line("keepalive (timed mode only); ch=0..3,8..11 (Servo1..4,9..12)");
        console_.line("CAL_START MOVES 8 SERVOS: loosen linkages and prepare physical servo-power cutoff first.");
        console_.line("Manual hold: outputs remain until OFF or successful fault shutdown; physically cut power if control is lost.");
        console_.line("Wi-Fi AP starts on boot: use PC browser at http://192.168.4.1/ . No auto movement.");
        console_.line("arm_stand (validate, ARM, queue Stand) | stand | walk | trot | leg_test <1..4> | motion_stop | off");
        console_.line("motion_set <step> <lift> <period_s> <height> <forward> <shift> | motion_preview <stand|walk|trot|leg1..leg4>");
        console_.line("motion_save | motion_load: versioned gait record; require all outputs OFF/disarmed; centre keys unchanged");
        console_.line("body_set <pitch_deg> <roll_deg> <body_x_mm> <cg_0_or_1> <cg_shift_mm> | pose_live_set <height> <pitch> <roll> <body_x>");
        console_.line("balance_capture | balance_on | balance_off | balance_roll_sign <-1|0|1>; roll defaults -1 confirmed, 0 disables");
        console_.line("drive_begin <epoch> <seq> <walk|trot> <forward> <turn> <step0|1> | drive_renew <epoch> <seq> | drive_end <epoch> <seq>");
        console_.line("moving_cg_on | moving_cg_off: reference required; feedback enable is volatile and never arms");
        console_.line("control_set <planner_slew> <wait_s> <angle_gain> <rate_gain> <tau_s> <max_shift> <feedback_slew> <pitch_kp> <pitch_kd> <roll_kp> <roll_kd> | profile_save/load");
        console_.line("trot_ff_set <enable0|1> <forward_gain> <backward_gain> <turn_gain> <max_abs_mm> (Profile RAM)");
        console_.line("straight_trim_set <-0.30..+0.30> (RAM only; + corrects left drift; restart restores default 0.25)");
        console_.line("trot_pitch_set <-0.5..0.0> (deg, RAM only; negative lowers nose in Trot)");
        break;
    case CommandType::Status: status(); break;
    case CommandType::Imu: imuReport(); break;
    case CommandType::Attitude: attitudeReport(); break;
    case CommandType::Center: {
        if (!calibration_.selected(cmd.channel)) {
            console_.line("ERR: channel not selected for this board");
            break;
        }
        char text[80];
        std::snprintf(text, sizeof(text), "Servo%u CH%u center=%u deg (%u us)",
                      static_cast<unsigned>(cmd.channel + 1),
                      static_cast<unsigned>(cmd.channel),
                      static_cast<unsigned>(calibration_.centerDeg(cmd.channel)),
                      static_cast<unsigned>(calibration_.centerUs(cmd.channel)));
        console_.line(text);
        break;
    }
    default:
        console_.line(execute(cmd).message);
        break;
    }
}

DebugReply RobotApp::execute(const Command& cmd) {
    // STOP/OFF are priority paths: never let a pending start emit its first
    // frame while servicing the cancellation request.
    if (preflight_job_.active && cmd.type != CommandType::MotionStop &&
        cmd.type != CommandType::Off && cmd.type != CommandType::BalanceOff &&
        cmd.type != CommandType::MovingCgOff &&
        cmd.type != CommandType::DriveRenew && cmd.type != CommandType::DriveEnd)
        return {false,"Preflight checking in progress; use Stop/Off to cancel"};
    const bool conflictsWithCoordinated = cmd.type == CommandType::Pwm ||
        cmd.type == CommandType::Offset || cmd.type == CommandType::Shank ||
        cmd.type == CommandType::Nudge || cmd.type == CommandType::OffChannel ||
        cmd.type == CommandType::CalStart || cmd.type == CommandType::CalSaveAll ||
        cmd.type == CommandType::Save || cmd.type == CommandType::RangeNormal ||
        cmd.type == CommandType::RangeExtended;
    // Reject conflicting requests before tick(): servicing a read-only preview
    // or a single-channel action must itself cause zero periodic PWM writes.
    if (motionActive() && conflictsWithCoordinated)
        return {false,"Coordinated motion/hold active; use global OFF before this action"};
    if ((cmd.type == CommandType::MotionPreview || cmd.type == CommandType::MotionSet ||
         cmd.type == CommandType::BodySet || cmd.type == CommandType::ControlSet ||
         cmd.type == CommandType::TrotFeedforwardSet ||
         cmd.type == CommandType::StraightTrimSet ||
         cmd.type == CommandType::TrotPitchSet) &&
        !motionTuningAllowed())
        return {false,"Motion is running/changing or output is unstable; Stop to Hold or use OFF"};
    // execute() never advances the scheduler. Rejected, read-only and config
    // requests therefore cannot sneak an overdue gait/balance PWM frame in.
    switch (cmd.type) {
    case CommandType::Stand: {
        V10AdvancedMotionParameters candidate=advanced_parameters_;
        candidate.drive=V10DriveIntent{};candidate.dynamic_body_x_mm=0;
        return requestMotionWithAdvanced(MotionMode::Stand,candidate);
    }
    case CommandType::ArmStand: return armAndStand();
    case CommandType::Walk: {
        V10AdvancedMotionParameters candidate=advanced_parameters_;
        if(!effectiveDrive(V10DriveIntent{},candidate.drive))
            return {false,"Straight trim invalid"};
        return requestMotionWithAdvanced(MotionMode::Walk,candidate);
    }
    case CommandType::Trot: {
        V10AdvancedMotionParameters candidate=advanced_parameters_;
        if(!effectiveDrive(V10DriveIntent{},candidate.drive))
            return {false,"Straight trim invalid"};
        return requestMotionWithAdvanced(MotionMode::Trot,candidate);
    }
    case CommandType::LegTest:
        return requestMotion(static_cast<MotionMode>(static_cast<int>(MotionMode::Leg1)+cmd.leg-1));
    case CommandType::MotionStop:
        cancelPreflight("cancelled_motion_stop",true);
        disableBalance(false,"stopped_by_user");
        disableMovingCg("motion_stop");stopDriveSession("motion_stop");cg_planner_.clear();
        motion_pending_=false;
        motion_transition_=false;
        motion_mode_=motion_frame_valid_ ? MotionMode::Hold : MotionMode::Idle;
        motion_error_="none";
        return {true,"Motion phase stopped; last commanded frame holds; no further periodic writes"};
    case CommandType::DriveBegin: {
        if(!servos_.armed())return {false,"drive_begin requires existing ARM; it never arms implicitly"};
        if(!drive_session_.canBegin(cmd.epoch,cmd.seq))
            return {false,"Drive begin rejected: active session or stale epoch/seq"};
        V10DriveIntent effective{};float left=0,right=0;
        if(!effectiveDrive(cmd.drive_intent,effective)||
           !driveScales(effective,left,right)||
           (cmd.drive_mode!=MotionMode::Walk&&cmd.drive_mode!=MotionMode::Trot))
            return {false,"Invalid drive intent or gait mode"};
        V10AdvancedMotionParameters candidate=advanced_parameters_;
        candidate.drive=effective;
        if(!drive_session_.begin(cmd.epoch,cmd.seq,clock_.millis()))
            return {false,"Internal drive session commit failed"};
        const std::uint32_t provisional_epoch=drive_session_.epoch();
        drive_session_mode_=cmd.drive_mode;drive_session_reason_="preflight_pending";
        const DebugReply request=requestMotionWithAdvanced(
            cmd.drive_mode,candidate,PreflightPurpose::DriveStart,provisional_epoch);
        if(!request.ok){
            if(drive_session_.active()&&drive_session_.epoch()==provisional_epoch)
                drive_session_.invalidate();
            drive_session_mode_=MotionMode::Idle;drive_session_reason_="preflight_rejected";
            return request;
        }
        return {true,"Drive preflight queued; provisional lease active; no PWM sent"};
    }
    case CommandType::DriveRenew: {
        const bool renewed=drive_session_.renew(cmd.epoch,cmd.seq,clock_.millis());
        if(!renewed&&drive_session_.consumeExpired()){
            cancelPreflight("cancelled_lease_expired",true);
            drive_session_mode_=MotionMode::Idle;drive_session_reason_="lease_expired_hold";
            motion_pending_=false;motion_transition_=false;
            motion_mode_=motion_frame_valid_?MotionMode::Hold:MotionMode::Idle;
            disableMovingCg("lease_expired");cg_planner_.clear();
        }
        return renewed ? DebugReply{true,"Drive lease renewed; phase and transition unchanged"} :
            DebugReply{false,"Drive renew rejected: expired, replayed, out-of-order, or wrong epoch"};
    }
    case CommandType::DriveEnd:
        if(!drive_session_.end(cmd.epoch,cmd.seq))return {false,"Drive end rejected: wrong or inactive epoch"};
        cancelPreflight("cancelled_drive_release",true);
        drive_session_mode_=MotionMode::Idle;drive_session_reason_="released_hold";
        advanced_parameters_.drive=V10DriveIntent{};disableMovingCg("drive_released");
        motion_pending_=false;motion_transition_=false;cg_planner_.clear();
        motion_mode_=motion_frame_valid_?MotionMode::Hold:MotionMode::Idle;
        return {true,"Drive released: last complete command frame frozen; servo torque remains; command Stand explicitly before unsupported handling"};
    case CommandType::MovingCgOn: {
        if(moving_cg_enabled_||moving_cg_enable_pending_)return {false,"Moving CG feedback already active/pending"};
        if(balanceBusy())return {false,"Disable/cancel static balance before moving CG feedback"};
        if(!balance_reference_ready_||last_attitude_.yaw_reference_ms!=balance_reference_attitude_ms_)
            return {false,"Fresh captured Stand reference required for moving CG feedback"};
        if(motion_mode_!=MotionMode::Stand||motion_pending_||motion_transition_||
           !servos_.armed()||!servos_.coordinated()||!currentStandMatchesApplied())
            return {false,"moving_cg_on requires current stable applied Stand; it never arms or moves in Stand"};
        moving_cg_enable_pending_=true;moving_cg_enable_generation_=imu_sample_generation_;
        moving_cg_error_="enable_pending";
        return {true,"Moving CG feedback queued for next fresh sample; Stand remains unchanged; no PWM sent"};
    }
    case CommandType::MovingCgOff:
        cancelPreflight("cancelled_moving_cg_off",true);
        disableMovingCg("disabled_by_user");disableBalance(false,"moving_cg_off");
        stopDriveSession("moving_cg_off");
        motion_pending_=false;motion_transition_=false;cg_planner_.clear();
        motion_mode_=motion_frame_valid_?MotionMode::Hold:MotionMode::Idle;
        return {true,"Moving CG feedback and drive stopped; last complete frame holds with servo torque"};
    case CommandType::MotionPreview: {
        if (!motion_config_) return {false,"V10 motion configuration unavailable"};
        if (!motionTuningAllowed()) return {false,"Stop to stable Hold or use OFF before preview"};
        V10AdvancedMotionParameters candidate=advanced_parameters_;
        if((cmd.motion_mode==MotionMode::Walk||cmd.motion_mode==MotionMode::Trot)&&
           !effectiveDrive(V10DriveIntent{},candidate.drive))
            return {false,"Straight trim invalid"};
        if(!queuePreflight(cmd.motion_mode,candidate,PreflightPurpose::Preview))
            return {false,"Read-only preview request rejected before checking"};
        return {true,"Read-only preview queued; no PWM sent"};
    }
    case CommandType::MotionSet: {
        if (!motion_config_) return {false,"V10 motion configuration unavailable"};
        if (!motionTuningAllowed()) return {false,"Stop to stable Hold or use OFF before changing parameters"};
        V10MotionParameters p{};
        p.step_length_mm=cmd.motion_values[0]; p.lift_mm=cmd.motion_values[1];
        p.period_s=cmd.motion_values[2]; p.stand_down_mm=cmd.motion_values[3];
        p.stand_forward_mm=cmd.motion_values[4]; p.walk_body_shift_mm=cmd.motion_values[5];
        V10Gait gait(*motion_config_); std::uint8_t c[8]; centers(c);
        if (!gait.parametersValid(p) ||
            !gait.preflightAdvanced(MotionMode::Stand,p,advanced_parameters_,c).ok)
            return {false,"Parameters fail finite/range/Stand geometry validation; applied values unchanged"};
        motion_parameters_=p;
        disableBalance(true,"motion_parameters_changed_recapture_required");
        ++motion_parameter_revision_;
        motion_parameters_saved_=saved_motion_parameters_valid_ &&
            sameParameters(motion_parameters_,saved_motion_parameters_);
        motion_parameter_state_=motion_parameters_saved_ ? "applied_matches_nvs" : "applied_ram";
        return {true,"Motion parameters applied in RAM for next start; state readback required; no PWM sent"};
    }
    case CommandType::PoseLiveSet: {
        if(!motion_config_)return {false,"V10 motion configuration unavailable"};
        if(servos_.calibrating()||servos_.activeChannel()>=0||servos_.faulted()||
           balanceBusy()||moving_cg_enabled_||moving_cg_enable_pending_||
           motion_mode_==MotionMode::Walk||motion_mode_==MotionMode::Trot||
           (motion_mode_>=MotionMode::Leg1&&motion_mode_<=MotionMode::Leg4))
            return {false,"Live pose requires OFF/idle or Stand/Hold without calibration, gait, or feedback control"};
        const bool off_idle=!servos_.armed()&&motion_mode_==MotionMode::Idle;
        const bool stable_pose=servos_.armed()&&servos_.coordinated()&&motion_frame_valid_&&
            (motion_mode_==MotionMode::Stand||motion_mode_==MotionMode::Hold);
        if(!off_idle&&!stable_pose)
            return {false,"Live pose requires outputs OFF/idle or stable armed Stand/Hold; it never arms implicitly"};
        if(stable_pose&&!imuFresh())
            return {false,"Live pose retarget requires a fresh IMU sample; applied values unchanged"};
        V10MotionParameters motion_candidate=motion_parameters_;
        motion_candidate.stand_down_mm=cmd.pose_live_values[0];
        V10ControlProfile profile_candidate=control_profile_;
        profile_candidate.body.pitch_deg=cmd.pose_live_values[1];
        profile_candidate.body.roll_deg=cmd.pose_live_values[2];
        profile_candidate.body.body_x_mm=cmd.pose_live_values[3];
        V10AdvancedMotionParameters advanced_candidate=advanced_parameters_;
        advanced_candidate.body=profile_candidate.body;
        advanced_candidate.dynamic_body_x_mm=0.0f;
        advanced_candidate.cg_body_x_override_valid=false;
        std::uint8_t c[8];centers(c);
        V10Gait gait(*motion_config_);
        if(!gait.parametersValid(motion_candidate)||!validControlProfile(profile_candidate)||
           !gait.preflightAdvanced(MotionMode::Stand,motion_candidate,advanced_candidate,c).ok)
            return {false,"Live height/Pitch/Roll/body_x tuple is outside the current Stand transform/IK/map/servo envelope; unchanged"};
        const bool motion_saved=saved_motion_parameters_valid_&&
            sameParameters(motion_candidate,saved_motion_parameters_);
        const bool profile_saved=saved_control_profile_valid_&&
            sameControlProfile(profile_candidate,saved_control_profile_);
        motion_parameters_=motion_candidate;
        motion_parameters_saved_=motion_saved;
        motion_parameter_state_=motion_saved?"applied_matches_nvs":"applied_ram";
        applyControlProfile(profile_candidate,profile_saved?"applied_matches_nvs":"applied_ram",
                            profile_saved);
        ++motion_parameter_revision_;++advanced_parameter_revision_;++control_profile_revision_;
        if(stable_pose){
            motion_requested_=MotionMode::Stand;
            motion_pending_=true;
            motion_request_sample_generation_=imu_sample_generation_;
            motion_error_="none";
            return {true,"Live pose atomically applied; smooth Stand retarget waits for the next fresh IMU sample"};
        }
        return {true,"Live pose atomically applied in RAM while outputs are OFF; no movement and no implicit ARM"};
    }
    case CommandType::BodySet: {
        if(!motion_config_)return {false,"V10 motion configuration unavailable"};
        if(!motionTuningAllowed())return {false,"Body settings require OFF or stable Hold/Stand with balance disabled"};
        const float cg_flag=cmd.body_values[3];
        if((cg_flag!=0.0f&&cg_flag!=1.0f))return {false,"CG enable must be exactly 0 or 1"};
        V10ControlProfile candidate=control_profile_;
        candidate.body.pitch_deg=cmd.body_values[0];candidate.body.roll_deg=cmd.body_values[1];
        candidate.body.body_x_mm=cmd.body_values[2];candidate.cg_walk_enabled=cg_flag==1.0f;
        candidate.cg_shift_mm=cmd.body_values[4];
        if(!validateControlProfile(candidate))
            return {false,"Body pose/CG settings fail range or Stand geometry; unchanged"};
        const bool saved=saved_control_profile_valid_&&sameControlProfile(candidate,saved_control_profile_);
        applyControlProfile(candidate,saved?"applied_matches_nvs":"applied_ram",saved);
        ++advanced_parameter_revision_;++control_profile_revision_;
        return {true,"Body pose/CG settings applied in RAM for next explicit motion; no PWM sent; motion_save remains six gait values only"};
    }
    case CommandType::ControlSet: {
        if(!motionTuningAllowed())return {false,"Control settings require OFF or stable Hold/Stand with feedback disabled"};
        V10ControlProfile candidate=control_profile_;
        candidate.planner.body_slew_mm_s=cmd.control_values[0];
        const float wait_s=cmd.control_values[1];
        if(!std::isfinite(wait_s)||wait_s<1||wait_s>30)return {false,"CG wait timeout must be 1..30 seconds"};
        candidate.planner.wait_timeout_ms=static_cast<std::uint32_t>(wait_s*1000.0f+0.5f);
        candidate.moving.angle_gain=cmd.control_values[2];
        candidate.moving.rate_gain_mm_per_rad_s=cmd.control_values[3];
        candidate.moving.filter_tau_s=cmd.control_values[4];
        candidate.moving.max_abs_shift_mm=cmd.control_values[5];
        candidate.moving.max_slew_mm_s=cmd.control_values[6];
        candidate.pitch_kp=cmd.control_values[7];candidate.pitch_kd_s=cmd.control_values[8];
        candidate.roll_kp=cmd.control_values[9];candidate.roll_kd_s=cmd.control_values[10];
        if(!validateControlProfile(candidate))return {false,"Control parameters out of safe bounds; unchanged"};
        const bool saved=saved_control_profile_valid_&&sameControlProfile(candidate,saved_control_profile_);
        applyControlProfile(candidate,saved?"applied_matches_nvs":"applied_ram",saved);
        ++control_profile_revision_;
        return {true,"Control profile applied in RAM; enable/reference/drive remain disabled; no PWM sent"};
    }
    case CommandType::TrotFeedforwardSet: {
        if(!motionTuningAllowed())return {false,"Trot feedforward settings require OFF or stable Hold/Stand with feedback disabled"};
        if(cmd.trot_ff_values[0]!=0.0f&&cmd.trot_ff_values[0]!=1.0f)
            return {false,"Trot feedforward enable must be exactly 0 or 1"};
        V10ControlProfile candidate=control_profile_;
        candidate.trot_feedforward.enabled=cmd.trot_ff_values[0]==1.0f;
        candidate.trot_feedforward.forward_gain=cmd.trot_ff_values[1];
        candidate.trot_feedforward.backward_gain=cmd.trot_ff_values[2];
        candidate.trot_feedforward.turn_gain=cmd.trot_ff_values[3];
        candidate.trot_feedforward.max_abs_mm=cmd.trot_ff_values[4];
        if(!validateControlProfile(candidate))return {false,"Trot feedforward values outside safe profile bounds"};
        const bool saved=saved_control_profile_valid_&&sameControlProfile(candidate,saved_control_profile_);
        applyControlProfile(candidate,saved?"applied_matches_nvs":"applied_ram",saved);
        ++control_profile_revision_;
        return {true,"Trot intent feedforward applied in RAM; default/off baseline preserved when disabled; no PWM sent"};
    }
    case CommandType::StraightTrimSet: {
        if(!motionTuningAllowed())
            return {false,"Straight trim requires OFF/idle or stable Stand/Hold"};
        V10DriveIntent effective{};
        if(!applyStraightTrim(V10DriveIntent{},cmd.straight_trim,effective))
            return {false,"Straight trim must be finite and within -0.30..+0.30"};
        straight_trim_=cmd.straight_trim;
        return {true,"Straight trim applied in RAM; no PWM sent; restart restores default 0.25"};
    }
    case CommandType::TrotPitchSet: {
        if(!motionTuningAllowed())
            return {false,"Trot Pitch offset requires OFF/idle or stable Stand/Hold"};
        if(!std::isfinite(cmd.trot_pitch_offset_deg)||cmd.trot_pitch_offset_deg<-0.5f||
           cmd.trot_pitch_offset_deg>0.0f||
           std::fabs(cmd.trot_pitch_offset_deg*10.0f-
                     std::round(cmd.trot_pitch_offset_deg*10.0f))>0.001f)
            return {false,"Trot Pitch offset must be -0.5..0.0 degrees in 0.1 degree steps"};
        V10AdvancedMotionParameters candidate=advanced_parameters_;
        candidate.trot_pitch_offset_deg=cmd.trot_pitch_offset_deg;
        candidate.dynamic_body_x_mm=0.0f;
        if(!effectiveDrive(V10DriveIntent{},candidate.drive))
            return {false,"Straight trim invalid"};
        std::uint8_t c[8];centers(c);
        if(!motion_config_||!V10Gait(*motion_config_).preflightAdvanced(
             MotionMode::Trot,motion_parameters_,candidate,c).ok)
            return {false,"Trot Pitch offset fails current Trot geometry; applied value unchanged"};
        advanced_parameters_.trot_pitch_offset_deg=cmd.trot_pitch_offset_deg;
        ++advanced_parameter_revision_;++trot_pitch_revision_;
        return {true,"Trot Pitch offset applied in RAM after full Trot preflight; no PWM sent; restart resets it to zero"};
    }
    case CommandType::ProfileSave: {
        if(!profile_store_)return {false,"Control profile storage unavailable"};
        if(servos_.armed()||servos_.faulted()||motionActive())
            return {false,"profile_save requires all outputs OFF, disarmed, and no fault"};
        if(!validateControlProfile(control_profile_))return {false,"Current control profile invalid; NVS unchanged"};
        const ControlProfileRecord record=makeProfileRecord(control_profile_);
        if(profile_store_->save(record)!=ControlProfileStoreResult::Ok){control_profile_state_="nvs_save_failed";
            return {false,"Control profile NVS save failed"};}
        ControlProfileRecord verified{};
        V10ControlProfile verified_profile{};
        if(profile_store_->load(verified)!=ControlProfileStoreResult::Ok||
           verified.checksum!=record.checksum||checksumProfileRecord(verified)!=verified.checksum||
           !decodeProfileRecord(verified,verified_profile)||
           !sameControlProfile(verified_profile,control_profile_)){
            control_profile_state_="nvs_save_verify_failed";return {false,"Control profile readback verification failed"};}
        saved_control_profile_=control_profile_;saved_control_profile_valid_=true;
        control_profile_saved_=true;control_profile_state_="saved_nvs";
        return {true,"Body/CG/planner/moving-CG control profile saved and read back; no enable/reference/drive state stored"};
    }
    case CommandType::ProfileLoad: {
        if(!profile_store_)return {false,"Control profile storage unavailable"};
        if(servos_.armed()||servos_.faulted()||motionActive())
            return {false,"profile_load requires all outputs OFF, disarmed, and no fault"};
        ControlProfileRecord record{};V10ControlProfile loaded{};
        const auto result=profile_store_->load(record);
        if(result!=ControlProfileStoreResult::Ok||!decodeProfileRecord(record,loaded)||
           !validateControlProfile(loaded)){control_profile_state_=result==ControlProfileStoreResult::Missing?
               "nvs_missing_using_current":"nvs_invalid_or_failed_using_current";
            return {false,"Saved control profile missing/corrupt/incompatible; RAM unchanged"};}
        saved_control_profile_=loaded;saved_control_profile_valid_=true;
        applyControlProfile(loaded,"loaded_nvs",true);++control_profile_revision_;
        return {true,"Control profile restored to RAM; robot remains OFF and feedback disabled"};
    }
    case CommandType::BalanceRollSign:
        if(balanceBusy()||moving_cg_enabled_||moving_cg_enable_pending_)
            return {false,"Disable static/moving feedback before changing Roll sensor sign"};
        if(!((!servos_.armed()&&motion_mode_==MotionMode::Idle)||
             (motion_mode_==MotionMode::Hold&&servos_.coordinated())))
            return {false,"Roll sign may change only while OFF/idle or stable Hold"};
        balance_roll_sensor_sign_=cmd.axis_sign;
        disableBalance(true,"roll_sign_changed_recapture_required");
        return {true,cmd.axis_sign==0 ? "Roll balance explicitly disabled by sign 0; reference cleared" :
              "Roll sensor sign applied in RAM; capture a new reference before balance_on"};
    case CommandType::BalanceCapture:
        if(balanceBusy())return {false,"Balance capture/enable already active"};
        if(moving_cg_enabled_||moving_cg_enable_pending_)
            return {false,"Disable moving CG feedback before capturing shared attitude reference"};
        if(!attitude_ready_||!imuFresh()||!attitude_estimator_.calibrated())
            return {false,"Fresh calibrated attitude required for reference capture"};
        if(motion_mode_!=MotionMode::Stand||motion_pending_||motion_transition_||
           !servos_.armed()||!servos_.coordinated())
            return {false,"Reference capture requires stable Stand"};
        if(!currentStandMatchesApplied())
            return {false,"Applied Stand/body parameters are not at the current frame; command Stand and wait before capture"};
        disableBalance(true,"capture_pending");
        balance_capture_pending_=true;balance_capture_last_ms_=0;
        balance_capture_pitch_sum_=balance_capture_roll_sum_=0;
        return {true,"Balance reference capture started; requires 20 stable fresh samples; no PWM sent"};
    case CommandType::BalanceOn:
        if(moving_cg_enabled_||moving_cg_enable_pending_)
            return {false,"Disable moving CG feedback before static balance"};
        if(balanceBusy())return {false,"Balance capture/enable already active"};
        if(!balance_reference_ready_)return {false,"Capture a stable reference first"};
        if(motion_mode_!=MotionMode::Stand||motion_pending_||motion_transition_||
           !servos_.armed()||!servos_.coordinated()||!attitude_ready_||!imuFresh())
            return {false,"balance_on requires stable Stand and fresh attitude"};
        if(last_attitude_.yaw_reference_ms!=balance_reference_attitude_ms_)
            return {false,"Attitude reference changed; capture again before balance_on"};
        if(!currentStandMatchesApplied())
            return {false,"Applied Stand/body parameters are not at the current frame; command Stand, wait for transition, then capture again"};
        if(!balanceEnvelopeValid())return {false,"Balance +/-1.5 degree envelope fails Stand geometry/limits"};
        balance_enable_pending_=true;
        balance_enable_sample_generation_=imu_sample_generation_;
        balance_error_="enable_pending";
        return {true,balance_roll_sensor_sign_==0 ?
            "Pitch balance queued for next fresh sample; Roll is explicitly disabled by sign 0; no PWM sent" :
            "Pitch+Roll balance queued for next fresh sample; no PWM sent"};
    case CommandType::BalanceOff:
        cancelPreflight("cancelled_balance_off",true);
        disableBalance(false,"disabled_by_user");
        disableMovingCg("balance_off");stopDriveSession("balance_off");cg_planner_.clear();
        motion_pending_=false;motion_transition_=false;
        motion_mode_=motion_frame_valid_?MotionMode::Hold:MotionMode::Idle;
        return {true,"Balance stopped; last complete PWM frame holds and remains powered"};
    case CommandType::MotionSave: {
        if (!motion_store_) return {false,"Motion parameter storage unavailable"};
        if (servos_.armed() || servos_.faulted() || motionActive())
            return {false,"Save requires all outputs OFF, disarmed, and no fault"};
        if (!validateAppliedParameters(motion_parameters_))
            return {false,"Applied parameters invalid; NVS unchanged"};
        const MotionParameterRecord record=makeMotionRecord(motion_parameters_);
        if(motion_store_->save(record)!=MotionStoreResult::Ok) {
            motion_parameter_state_="nvs_save_failed";
            return {false,"Motion parameter NVS save failed; RAM values remain applied"};
        }
        MotionParameterRecord verified{};
        if(motion_store_->load(verified)!=MotionStoreResult::Ok ||
           verified.magic!=record.magic || verified.version!=record.version ||
           verified.length!=record.length || verified.checksum!=record.checksum ||
           checksumRecord(verified)!=verified.checksum) {
            motion_parameter_state_="nvs_save_verify_failed";
            return {false,"Motion NVS write could not be read back intact; RAM values remain applied"};
        }
        saved_motion_parameters_=motion_parameters_; saved_motion_parameters_valid_=true;
        motion_parameters_saved_=true; motion_parameter_state_="saved_nvs";
        return {true,"Applied motion parameters saved as one versioned NVS record; no PWM sent"};
    }
    case CommandType::MotionLoad: {
        if (!motion_store_) return {false,"Motion parameter storage unavailable"};
        if (servos_.armed() || servos_.faulted() || motionActive())
            return {false,"Load requires all outputs OFF, disarmed, and no fault"};
        MotionParameterRecord record{}; V10MotionParameters loaded{};
        const MotionStoreResult result=motion_store_->load(record);
        if(result!=MotionStoreResult::Ok) {
            motion_parameter_state_=result==MotionStoreResult::Missing ?
                "nvs_missing_using_current" : result==MotionStoreResult::Invalid ?
                "nvs_invalid_using_current" : "nvs_load_failed_using_current";
            return {false,"Saved motion parameters unavailable; current RAM values unchanged"};
        }
        if(!decodeMotionRecord(record,loaded) || !validateAppliedParameters(loaded)) {
            motion_parameter_state_="nvs_invalid_using_current";
            return {false,"Saved motion record corrupt/incompatible/unsafe; current RAM values unchanged"};
        }
        motion_parameters_=loaded; saved_motion_parameters_=loaded;
        disableBalance(true,"motion_parameters_loaded_recapture_required");
        saved_motion_parameters_valid_=true; motion_parameters_saved_=true;
        motion_parameter_state_="loaded_nvs"; ++motion_parameter_revision_;
        return {true,"Saved motion parameters restored to applied RAM; no PWM sent"};
    }
    case CommandType::RangeNormal:
    case CommandType::RangeExtended:
        switch (servos_.setExtendedRange(cmd.type == CommandType::RangeExtended)) {
        case ServoResult::Ok:
            return cmd.type == CommandType::RangeExtended
                ? DebugReply{true, "Extended bench range unlocked; no PWM sent; NOT a verified mechanical range"}
                : DebugReply{true, "Normal bench range restored; no PWM sent"};
        case ServoResult::ModeConflict:
            return {false, "Use global OFF before changing pulse range"};
        default:
            return {false, "Pulse range change rejected; check PCA/configuration fault"};
        }
    case CommandType::CalStart:
        if (!imuFresh()) return {false, "IMU not healthy; calibration denied"};
        switch (calibration_.startAll()) {
        case ServoResult::Ok:
            calibration_session_pending_ = true;
            return {true, "Calibration active: 8 channels at current RAM centers (loaded NVS or default 90 deg); OFF before saving"};
        case ServoResult::ModeConflict:
            return {false, "Another output mode active; use global OFF before calibration"};
        case ServoResult::OutOfRange:
            return {false, "Calibration center/channel outside bench limits; no PWM started"};
        default:
            return {false, "Calibration PWM not confirmed; physically disconnect servo power NOW"};
        }
    case CommandType::CalSaveAll: {
        if (!calibration_session_pending_)
            return {false, "No calibration session pending; start calibration first"};
        if (servos_.armed()) {
            if (!servos_.calibrating())
                return {false, "Single-output mode active; use global OFF before saving"};
            if (!servos_.off())
                return {false, "Calibration shutdown not confirmed; disconnect servo power NOW; NVS unchanged"};
        }
        std::size_t saved = 0;
        switch (calibration_.saveAll(saved)) {
        case CalibrationResult::Ok:
            calibration_session_pending_ = false;
            return {true, "All eight centers saved to NVS after PWM OFF"};
        case CalibrationResult::HardwareFault:
            return {false, "PWM fault; output may persist; disconnect servo power NOW; NVS not saved"};
        default:
            return {false, "NVS save failed or partial; outputs OFF; retry cal_save_all after storage check"};
        }
    }
    case CommandType::Arm:
        if (!imuFresh()) return {false, "IMU not healthy; ARM denied"};
        if (servos_.armed()) return {false, "Already armed; use OFF to disarm"};
        return servos_.arm() ? DebugReply{true, "ARMED; no pulse sent"}
                             : DebugReply{false, "PCA fault; ARM denied"};
    case CommandType::Off:
        cancelPreflight("cancelled_global_off",true);
        disableBalance(true,"cleared_by_global_off");
        disableMovingCg("cleared_by_global_off");stopDriveSession("global_off");
        cg_planner_.clear();
        cancelMotion("none",MotionMode::Idle);
        motion_frame_valid_=false;
        return servos_.off() ? DebugReply{true, "All 16 PWM channels OFF; motion plan cancelled"}
                             : DebugReply{false, "PWM shutdown failed; disconnect servo power NOW"};
    case CommandType::OffChannel:
        switch (servos_.offChannel(cmd.channel)) {
        case ServoResult::Ok:
            return {true, "Selected PWM channel OFF (I2C acknowledged); ARM remains on"};
        case ServoResult::NotActive:
            return {false, "Selected channel has no active PWM; other output unchanged"};
        case ServoResult::ModeConflict:
            return {false, "Eight-channel calibration active; use global OFF"};
        case ServoResult::OutOfRange: return {false, "Channel not selected for this board"};
        default:
            return {false, "Channel OFF not confirmed; use global OFF and physically disconnect servo power if output persists"};
        }
    case CommandType::KeepAlive:
        if (servos_.manualHold())
            return imuFresh() && servos_.armed() && !servos_.faulted()
                ? DebugReply{true, "Manual hold active; keepalive not needed"}
                : DebugReply{false, "Not armed or IMU fault"};
        return imuFresh() && servos_.keepAlive() ? DebugReply{true, "OK"}
                                                 : DebugReply{false, "Not armed or IMU fault"};
    case CommandType::Pwm:
    case CommandType::Offset:
    case CommandType::Shank:
    case CommandType::Nudge:
        if (!imuFresh()) {
            const bool off_ok = servos_.off();
            return off_ok ? DebugReply{false, "IMU unavailable; PWM off acknowledged"}
                          : DebugReply{false, "Cannot shut PWM off; disconnect servo power NOW"};
        }
        if (cmd.type == CommandType::Pwm || cmd.type == CommandType::Offset ||
            cmd.type == CommandType::Shank) {
            std::uint16_t pulse = cmd.pulse_us;
            if (cmd.type == CommandType::Offset) {
                const RelativeServoLimit* limit = relativeLimit(cmd.channel);
                if (!calibration_.selected(cmd.channel) || !limit)
                    return {false, "Relative joint not configured for this channel"};
                if (!relativeServoPulse(calibration_.centerDeg(cmd.channel), cmd.offset_deg,
                    limit->min_offset_deg, limit->max_offset_deg, pulse))
                    return {false, "Offset outside this joint range or center+offset outside 0..180; no PWM sent"};
            } else if (cmd.type == CommandType::Shank) {
                const V10ShankMapConfig* config = shankMapConfig(cmd.channel);
                std::int8_t direction = 0;
                float max_u = 0, max_q = 0;
                if (!calibration_.selected(cmd.channel) || !config ||
                    !shankRange(cmd.channel, calibration_.centerDeg(cmd.channel),
                                direction, max_u, max_q))
                    return {false, "Shank q photo curve is not configured for this channel"};
                if (!std::isfinite(cmd.shank_q_deg) || cmd.shank_q_deg < 0.0f ||
                    cmd.shank_q_deg > max_q)
                    return {false, "Shank q outside current center/curve range; no PWM sent"};
                float u = 0;
                const V10ShankLinearMap map(config->points, config->point_count);
                if (!map.uFromQ(cmd.shank_q_deg, u))
                    return {false, "Shank q conversion failed; no PWM sent"};
                // q/u are unsigned magnitudes. The existing one-sided relative
                // configuration owns actuator direction; apply it exactly once.
                if (u > max_u && u - max_u <= 0.0001f) u = max_u;
                const RelativeServoLimit* limit = relativeLimit(cmd.channel);
                if (u > max_u || !limit ||
                    !relativeServoPulse(calibration_.centerDeg(cmd.channel),
                                        static_cast<float>(direction) * u,
                                        limit->min_offset_deg, limit->max_offset_deg,
                                        pulse))
                    return {false, "Shank q outside current center/curve range; no PWM sent"};
            }
            switch (servos_.pulse(cmd.channel, pulse)) {
            case ServoResult::Ok:
                if (cmd.type == CommandType::Offset)
                    return {true, "Current calibrated center + offset output; center/NVS unchanged"};
                if (cmd.type == CommandType::Shank)
                    return {true, "Shared photo-candidate shank q output; current center/NVS unchanged"};
                return {true, "Raw calibration PWM updated; bypasses relative joint envelope"};
            case ServoResult::ModeConflict:
                return {false, "Eight-channel calibration active; use global OFF"};
            case ServoResult::NotArmed: return {false, "Use ARM first"};
            case ServoResult::OutOfRange: return {false, "Channel/pulse outside bench limits"};
            default: return {false, "PCA write failed; disconnect servo power NOW"};
            }
        }
        switch (calibration_.nudge(cmd.channel, cmd.delta_deg)) {
        case CalibrationResult::Ok:
            return servos_.calibrating()
                ? DebugReply{true, "Calibration center changed by 1 degree; other channels hold; OFF then SAVE"}
                : DebugReply{true, "Center changed by 1 degree; OFF then SAVE"};
        case CalibrationResult::NotArmed: return {false, "Use ARM first"};
        case CalibrationResult::ManualPulseActive:
            return {false, "Manual pulse active: OFF and ARM before center nudge"};
        case CalibrationResult::InvalidChannel: return {false, "Channel not selected"};
        case CalibrationResult::OutOfRange: return {false, "Center outside active electrical limits or absolute 0..180 degree range"};
        default: return {false, "Calibration/PCA fault; disconnect servo power if output persists"};
        }
    case CommandType::Save:
        switch (calibration_.save(cmd.channel)) {
        case CalibrationResult::Ok: return {true, "Channel center saved to NVS (no PWM sent)"};
        case CalibrationResult::DisarmBeforeSave:
            return {false, "Use OFF (all channels) before saving"};
        case CalibrationResult::HardwareFault:
            return {false, "PWM fault: OFF not confirmed; disconnect servo power NOW"};
        case CalibrationResult::InvalidChannel: return {false, "Channel not selected"};
        default: return {false, "NVS save failed"};
        }
    default: return {false, "Invalid command; type help"};
    }
}

DebugSnapshot RobotApp::snapshot() const {
    DebugSnapshot s{};
    s.pca = servos_.ready();
    s.armed = servos_.armed();
    s.calibrating = servos_.calibrating();
    s.calibration_pending = calibration_session_pending_;
    s.calibration_mask = servos_.calibrationMask();
    s.fault = servos_.faulted();
    s.imu_ok = imuFresh();
    if (s.imu_ok) {
        s.imu = last_sample_;
        const float ax = last_sample_.accel_mps2[0];
        const float ay = last_sample_.accel_mps2[1];
        const float az = last_sample_.accel_mps2[2];
        s.accel_norm_mps2 = std::sqrt(ax*ax + ay*ay + az*az);
    }
    s.attitude_calibration_samples = attitude_estimator_.calibrationSamples();
    s.gyro_calibrated = attitude_estimator_.calibrated();
    s.attitude_ok = s.imu_ok && attitude_ready_ &&
        static_cast<std::uint32_t>(clock_.millis() - last_attitude_.timestamp_ms) <= 100;
    if (s.attitude_ok) s.attitude = last_attitude_;
    s.active_channel = servos_.activeChannel();
    s.active_pulse_us = servos_.activePulseUs();
    s.remaining_ms = servos_.remainingMs();
    s.manual_hold = servos_.manualHold();
    s.extended_range = servos_.extendedRange();
    s.pulse_min_us = servos_.minPulseUs();
    s.pulse_max_us = servos_.maxPulseUs();
    s.motion_mode=V10Gait::modeName(motion_mode_);
    s.motion_error=motion_error_;
    s.motion_pending=motion_pending_;
    s.motion_transition=motion_transition_;
    s.motion_parameters=motion_parameters_;
    s.motion_parameter_state=motion_parameter_state_;
    s.motion_parameters_saved=motion_parameters_saved_;
    s.motion_parameter_revision=motion_parameter_revision_;
    s.motion_frame_count=motion_frame_count_;
    s.motion_last_update_gap_ms=motion_last_update_gap_ms_;
    s.motion_max_update_gap_ms=motion_max_update_gap_ms_;
    s.motion_missed_deadlines=motion_missed_deadlines_;
    s.advanced_parameters=advanced_parameters_;
    s.advanced_parameter_revision=advanced_parameter_revision_;
    if(motion_config_) {
        s.body_length_mm=motion_config_->body_length_mm;
        s.body_width_mm=motion_config_->body_width_mm;
    }
    s.body_dimensions_measured=false;
    s.balance_capture_pending=balance_capture_pending_;
    s.balance_capture_samples=balance_capture_count_;
    s.balance_reference_ready=balance_reference_ready_;
    s.balance_enabled=balance_enabled_;
    s.balance_enable_pending=balance_enable_pending_;
    s.balance_reference_pitch_deg=balance_reference_pitch_deg_;
    s.balance_reference_roll_deg=balance_reference_roll_deg_;
    s.balance_pitch_correction_deg=balance_pitch_correction_deg_;
    s.balance_roll_correction_deg=balance_roll_correction_deg_;
    s.balance_roll_sensor_sign=balance_roll_sensor_sign_;
    s.balance_error=balance_error_;
    s.preview_error=preview_error_;
    s.preview_leg=last_preview_.leg;
    s.preview_joint=last_preview_.joint;
    s.preview_phase=last_preview_.phase;
    s.preview_min_support_margin_mm=last_preview_.min_support_margin_mm;
    s.preview_pending=preflight_job_.active;
    s.preview_stage=preflight_stage_;
    s.preview_result=preflight_result_;
    s.preview_frames_checked=preflight_frames_checked_;
    s.preview_max_slice_ms=preflight_max_slice_ms_;
    s.preflight_duration_ms=preflight_duration_ms_;
    s.preflight_actual_cycle_s=preflight_actual_cycle_s_;
    float cg_front=advanced_parameters_.body.body_x_mm;
    float cg_rear=advanced_parameters_.body.body_x_mm;
    (void)cgTargets(cg_front,cg_rear);
    s.cg_actual_body_x_min_mm=advanced_parameters_.cg_walk_enabled ?
        std::fmin(advanced_parameters_.body.body_x_mm,std::fmin(cg_front,cg_rear)) :
        advanced_parameters_.body.body_x_mm;
    s.cg_actual_body_x_max_mm=advanced_parameters_.cg_walk_enabled ?
        std::fmax(advanced_parameters_.body.body_x_mm,std::fmax(cg_front,cg_rear)) :
        advanced_parameters_.body.body_x_mm;
    s.drive_intent=preflight_job_.active ? preflight_job_.advanced.drive :
        advanced_parameters_.drive;
    s.straight_trim=straight_trim_;
    s.effective_drive_turn=s.drive_intent.turn;
    s.trot_pitch_offset_deg=advanced_parameters_.trot_pitch_offset_deg;
    s.trot_effective_pitch_deg=advanced_parameters_.body.pitch_deg+
        advanced_parameters_.trot_pitch_offset_deg;
    s.trot_pitch_revision=trot_pitch_revision_;
    s.drive_session_active=drive_session_.active();
    s.drive_epoch=drive_session_.epoch();s.drive_seq=drive_session_.lastSeq();
    s.drive_lease_remaining_ms=drive_session_.remainingMs(clock_.millis());
    s.drive_session_reason=drive_session_reason_;
    s.moving_cg_enabled=moving_cg_enabled_;
    s.moving_cg_enable_pending=moving_cg_enable_pending_;
    s.moving_cg_shift_mm=moving_cg_shift_mm_;
    s.moving_cg_error=moving_cg_error_;
    s.cg_phase_waiting=cg_plan_.waiting&&cg_planner_.active();
    s.cg_stage=cg_plan_.stage;
    s.motion_nominal_elapsed_s=motion_nominal_elapsed_s_;
    s.motion_actual_elapsed_s=motion_actual_elapsed_s_;
    (void)trotFeedforward(motion_parameters_.step_length_mm,advanced_parameters_.drive,
                          control_profile_.trot_feedforward,s.trot_feedforward_mm);
    s.control_profile=control_profile_;
    s.control_profile_state=control_profile_state_;
    s.control_profile_saved=control_profile_saved_;
    s.control_profile_revision=control_profile_revision_;
    if (motion_config_ && motion_frame_valid_) {
        const bool periodic = !motion_pending_ && !motion_transition_ &&
            (motion_mode_ == MotionMode::Walk || motion_mode_ == MotionMode::Trot ||
             (motion_mode_ >= MotionMode::Leg1 && motion_mode_ <= MotionMode::Leg4));
        if (periodic) {
            if(cg_planner_.active())s.motion_phase=cg_plan_.phase;
            else {const std::uint32_t elapsed=clock_.millis()-motion_phase_start_ms_;
                s.motion_phase=motion_parameters_.period_s>0 ?
                    std::fmod(static_cast<float>(elapsed)/(motion_parameters_.period_s*1000),1.0f) : 0;}
        }
        for (int i=0;i<4;++i) {
            s.target_forward_mm[i]=motion_frame_.forward_mm[i];
            s.target_down_mm[i]=motion_frame_.down_mm[i];
        }
        for (int i=0;i<8;++i) s.command_deg[i]=motion_frame_.servo_deg[i];
    }
    const std::size_t count = calibration_.count() < 8 ? calibration_.count() : 8;
    s.channel_count = static_cast<std::uint8_t>(count);
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint8_t ch = calibration_.channelAt(i);
        s.channels[i] = {static_cast<std::uint8_t>(ch + 1), ch,
                         calibration_.centerUs(ch)};
        s.channels[i].center_deg = calibration_.centerDeg(ch);
        const RelativeServoLimit* limit = relativeLimit(ch);
        if (limit) {
            s.channels[i].joint = limit->joint;
            s.channels[i].shank = limit->shank;
            s.channels[i].relative_available = relativeServoBounds(
                s.channels[i].center_deg, limit->min_offset_deg, limit->max_offset_deg,
                s.channels[i].min_offset_deg, s.channels[i].max_offset_deg);
        }
        const V10ShankMapConfig* config = shankMapConfig(ch);
        std::int8_t direction = 0;
        float max_u = 0, max_q = 0;
        if (config && shankRange(ch, s.channels[i].center_deg, direction,
                                max_u, max_q)) {
            s.channels[i].shank_q_available = true;
            s.channels[i].shank_q_photo_candidate = true;
            s.channels[i].shank_direction = direction;
            s.channels[i].max_shank_q_deg = max_q;
            s.channels[i].shank_map_point_count =
                static_cast<std::uint8_t>(config->point_count);
            for (std::size_t n = 0; n < config->point_count; ++n)
                s.channels[i].shank_map_points[n] = config->points[n];
        }
    }
    return s;
}

} // namespace dog
