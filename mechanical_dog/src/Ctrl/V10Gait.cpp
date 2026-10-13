#include "Ctrl/V10Gait.h"
#include "Ctrl/PyAppleFootTrajectory.h"

#include <cmath>
#include <cstring>

namespace dog {
namespace {
constexpr float kPi = 3.14159265358979323846f;
float wrap(float v) { v -= std::floor(v); return v; }
float smooth(float t) { return t*t*(3.0f-2.0f*t); }
std::uint16_t pulseFor(float deg) {
    return static_cast<std::uint16_t>(500.0f + std::floor(2000.0f*deg/180.0f + 0.5f));
}
bool finite(float v) { return std::isfinite(v); }
struct CgState {
    float body_x=0;
    float offsets[4]={};
    int swing_leg=-1;
    float swing_t=0;
};
bool cgTargets(const V10MotionParameters& p,const V10AdvancedMotionParameters& a,
               float& front,float& rear){
    float left=0,right=0;if(!driveScales(a.drive,left,right))return false;
    const float s_left=p.step_length_mm*left,s_right=p.step_length_mm*right;
    front=p.stand_forward_mm+std::fmin(0.0f,s_left*0.5f)-a.cg_shift_mm;
    rear=p.stand_forward_mm+std::fmax(s_right*0.5f,(s_left+s_right)*0.5f)+a.cg_shift_mm;
    return finite(front)&&finite(rear);
}
CgState cgState(float phase,const V10MotionParameters& p,
                const V10AdvancedMotionParameters& a) {
    CgState s{};s.body_x=a.body.body_x_mm;
    float left=0,right=0;driveScales(a.drive,left,right);
    const float steps[4]={p.step_length_mm*left,p.step_length_mm*right,
                          p.step_length_mm*right,p.step_length_mm*left};
    auto completed=[&](int count){for(int i=0;i<count;++i)s.offsets[i]=steps[i];};
    auto swing=[&](int leg,float t){completed(leg);s.offsets[leg]=steps[leg]*smooth(t);s.swing_leg=leg;s.swing_t=t;};
    float front_target=0,rear_target=0;(void)cgTargets(p,a,front_target,rear_target);
    if(phase<0.10f)s.body_x=a.body.body_x_mm+
        (front_target-a.body.body_x_mm)*smooth(phase/0.10f);
    else if(phase<0.25f){s.body_x=front_target;swing(0,(phase-0.10f)/0.15f);}
    else if(phase<0.275f){s.body_x=front_target;completed(1);}
    else if(phase<0.425f){s.body_x=front_target;swing(1,(phase-0.275f)/0.15f);}
    else if(phase<0.525f){completed(2);s.body_x=front_target+
        (rear_target-front_target)*smooth((phase-0.425f)/0.10f);}
    else if(phase<0.675f){s.body_x=rear_target;completed(2);swing(2,(phase-0.525f)/0.15f);}
    else if(phase<0.70f){s.body_x=rear_target;completed(3);}
    else if(phase<0.85f){s.body_x=rear_target;completed(3);swing(3,(phase-0.70f)/0.15f);}
    else {const float t=smooth((phase-0.85f)/0.15f);s.body_x=rear_target+
        (a.body.body_x_mm-rear_target)*t;for(int i=0;i<4;++i)s.offsets[i]=steps[i]*(1.0f-t);}
    return s;
}
float supportMargin(const CgState& state,const V10MotionFrame& frame,
                    const V10AdvancedMotionParameters& advanced,
                    const V10MotionConfig& config) {
    if(state.swing_leg<0)return 1.0e6f;
    struct Point{float x,y;};Point support[3];int used=0;
    constexpr float rad=0.01745329251994329577f;
    const float pitch=advanced.body.pitch_deg*rad,roll=advanced.body.roll_deg*rad;
    const float cp=std::cos(pitch),sp=std::sin(pitch),cr=std::cos(roll),sr=std::sin(roll);
    const float r00=cp,r01=-sp*sr,r02=-sp*cr;
    const float r10=0.0f,r11=cr,r12=-sr;
    const float hx[4]={config.body_length_mm*0.5f,config.body_length_mm*0.5f,
                       -config.body_length_mm*0.5f,-config.body_length_mm*0.5f};
    const float hy[4]={config.body_width_mm*0.5f,-config.body_width_mm*0.5f,
                       -config.body_width_mm*0.5f,config.body_width_mm*0.5f};
    for(int leg=0;leg<4;++leg)if(leg!=state.swing_leg) {
        const float bx=hx[leg]+frame.forward_mm[leg];
        const float by=hy[leg];
        const float bz=-frame.down_mm[leg];
        support[used++]={state.body_x+r00*bx+r01*by+r02*bz,
                        r10*bx+r11*by+r12*bz};
    }
    float orientation=0,margin=1.0e6f;
    for(int i=0;i<3;++i){const Point a=support[i],b=support[(i+1)%3];
        const float cross=(b.x-a.x)*(0.0f-a.y)-(b.y-a.y)*(state.body_x-a.x);
        if(i==0)orientation=cross;else if(cross*orientation<0)return -1.0f;
        margin=std::fmin(margin,std::fabs(cross)/std::hypot(b.x-a.x,b.y-a.y));}
    return margin;
}
}

const char* V10Gait::modeName(MotionMode mode) {
    switch (mode) {
    case MotionMode::Idle: return "idle"; case MotionMode::Hold: return "hold";
    case MotionMode::Stand: return "stand"; case MotionMode::Walk: return "walk";
    case MotionMode::Trot: return "trot"; case MotionMode::Leg1: return "leg_test_1";
    case MotionMode::Leg2: return "leg_test_2"; case MotionMode::Leg3: return "leg_test_3";
    case MotionMode::Leg4: return "leg_test_4"; case MotionMode::HoldError: return "hold_error";
    }
    return "invalid";
}

bool V10Gait::configValid() const {
    if (!finite(config_.geometry.upper_mm) || !finite(config_.geometry.lower_mm) ||
        config_.geometry.upper_mm <= 0 || config_.geometry.lower_mm <= 0 ||
        !finite(config_.hip_geometry_neutral_deg) ||
        !finite(config_.knee_fold_neutral_deg) ||
        !finite(config_.hip_window_deg) || config_.hip_window_deg <= 0 ||
        !finite(config_.transition_s) || config_.transition_s <= 0 ||
        config_.update_ms == 0 || config_.max_tick_gap_ms <= config_.update_ms ||
        !V10BodyPose(config_.body_length_mm,config_.body_width_mm).valid() ||
        !config_.shank_points ||
        !V10ShankLinearMap(config_.shank_points, config_.shank_point_count).valid()) return false;
    std::uint16_t mask = 0;
    for (int i=0;i<8;++i) {
        if (config_.channels[i] >= 16 || (mask & (1U << config_.channels[i]))) return false;
        mask = static_cast<std::uint16_t>(mask | (1U << config_.channels[i]));
    }
    for (int i=0;i<4;++i)
        if ((config_.hip_direction[i] != -1 && config_.hip_direction[i] != 1) ||
            (config_.shank_direction[i] != -1 && config_.shank_direction[i] != 1)) return false;
    return true;
}

bool V10Gait::parametersValid(const V10MotionParameters& p) const {
    return configValid() && finite(p.step_length_mm) && p.step_length_mm >= 0 && p.step_length_mm <= 30 &&
        finite(p.lift_mm) && p.lift_mm >= 0 && p.lift_mm <= 30 &&
        finite(p.period_s) && p.period_s >= 0.4f && p.period_s <= 10 &&
        finite(p.stand_down_mm) && p.stand_down_mm >= 100 && p.stand_down_mm <= 115 &&
        finite(p.stand_forward_mm) && std::fabs(p.stand_forward_mm) <= 5 &&
        finite(p.walk_body_shift_mm) && p.walk_body_shift_mm >= 0 && p.walk_body_shift_mm <= 3;
}

bool V10Gait::advancedValid(const V10AdvancedMotionParameters& a) const {
    float left=0,right=0;
    float ff=0;
    return driveScales(a.drive,left,right) && trotFeedforward(1.0f,a.drive,a.trot_feedforward,ff) &&
        finite(a.body.pitch_deg) && a.body.pitch_deg>=kBodyPitchMinDeg &&
        a.body.pitch_deg<=kBodyPitchMaxDeg &&
        finite(a.trot_pitch_offset_deg) && a.trot_pitch_offset_deg>=-0.5f &&
        a.trot_pitch_offset_deg<=0.0f &&
        std::fabs(a.trot_pitch_offset_deg*10.0f-
                  std::round(a.trot_pitch_offset_deg*10.0f))<=0.001f &&
        finite(a.body.roll_deg) && a.body.roll_deg>=kBodyRollMinDeg &&
        a.body.roll_deg<=kBodyRollMaxDeg &&
        finite(a.body.body_x_mm) && std::fabs(a.body.body_x_mm)<=8.0f &&
        finite(a.cg_shift_mm) && a.cg_shift_mm>=0.0f && a.cg_shift_mm<=3.0f &&
        finite(a.dynamic_body_x_mm) && std::fabs(a.dynamic_body_x_mm)<=0.5f &&
        finite(a.cg_body_x_override_mm) && std::fabs(a.cg_body_x_override_mm)<=40.0f;
}

bool V10Gait::frame(MotionMode mode, float phase, const V10MotionParameters& p,
                    const std::uint8_t centers[8], V10MotionFrame& out) const {
    return frameAdvanced(mode,phase,p,V10AdvancedMotionParameters{},centers,out);
}

bool V10Gait::frameAdvanced(MotionMode mode,float phase,const V10MotionParameters& p,
                            const V10AdvancedMotionParameters& advanced,
                            const std::uint8_t centers[8],V10MotionFrame& out) const {
    V10PreflightError error=V10PreflightError::None; int leg=-1;
    return frameDetailed(mode,phase,p,advanced,centers,out,error,leg);
}

V10PreflightReport V10Gait::frameAdvancedReport(MotionMode mode,float phase,
                            const V10MotionParameters& p,
                            const V10AdvancedMotionParameters& advanced,
                            const std::uint8_t centers[8],V10MotionFrame& out) const {
    V10PreflightReport report{};V10PreflightError error=V10PreflightError::None;int leg=-1;
    if(!frameDetailed(mode,phase,p,advanced,centers,out,error,leg)){
        report.error=error;report.leg=leg;report.phase=phase;return report;}
    if(mode==MotionMode::Walk&&advanced.cg_walk_enabled){
        CgState cg=cgState(wrap(phase),p,advanced);
        if(advanced.cg_body_x_override_valid)cg.body_x=advanced.cg_body_x_override_mm;
        cg.body_x+=advanced.dynamic_body_x_mm;
        const float margin=supportMargin(cg,out,advanced,config_);
        report.min_support_margin_mm=margin;
        if(margin<0.5f){report.error=V10PreflightError::SupportMargin;report.leg=cg.swing_leg;report.phase=phase;return report;}
    }
    report.ok=true;report.error=V10PreflightError::None;return report;
}

bool V10Gait::frameDetailed(MotionMode mode,float phase,const V10MotionParameters& p,
                            const V10AdvancedMotionParameters& advanced,
                            const std::uint8_t centers[8],V10MotionFrame& out,
                            V10PreflightError& error,int& failed_leg) const {
    error=V10PreflightError::InvalidParameters; failed_leg=-1;
    if (!parametersValid(p) || !advancedValid(advanced) || !centers || !finite(phase) ||
        mode == MotionMode::Idle || mode == MotionMode::Hold || mode == MotionMode::HoldError)
        return false;
    V10MotionFrame next{};
    phase = wrap(phase);
    float left_scale=0,right_scale=0;
    if(!driveScales(advanced.drive,left_scale,right_scale)){
        error=V10PreflightError::DriveIntent;return false;
    }
    float nominal_forward[4]={p.stand_forward_mm,p.stand_forward_mm,
                              p.stand_forward_mm,p.stand_forward_mm};
    float nominal_down[4]={p.stand_down_mm,p.stand_down_mm,
                           p.stand_down_mm,p.stand_down_mm};
    V10BodyPoseCommand body=advanced.body;
    if(mode==MotionMode::Trot) {
        body.pitch_deg+=advanced.trot_pitch_offset_deg;
        if(!finite(body.pitch_deg)||body.pitch_deg<kBodyPitchMinDeg||
           body.pitch_deg>kBodyPitchMaxDeg){error=V10PreflightError::BodyTransform;return false;}
    }

    if(mode==MotionMode::Walk && advanced.cg_walk_enabled) {
        // Explicit all-support transfers around four sequential swing stages.
        // The legacy per-leg stride bias is deliberately ignored in this mode.
        CgState cg=cgState(phase,p,advanced);
        if(advanced.cg_body_x_override_valid)cg.body_x=advanced.cg_body_x_override_mm;
        cg.body_x+=advanced.dynamic_body_x_mm;
        body.body_x_mm=cg.body_x;
        for(int leg=0;leg<4;++leg)nominal_forward[leg]+=cg.offsets[leg];
        if(cg.swing_leg>=0){nominal_down[cg.swing_leg]-=p.lift_mm*0.5f*
            (1.0f-std::cos(2.0f*kPi*cg.swing_t));next.swing[cg.swing_leg]=true;}
    } else {
    for (int leg=0; leg<4; ++leg) {
        float x = p.stand_forward_mm, down = p.stand_down_mm;
        bool moving = mode == MotionMode::Walk || mode == MotionMode::Trot ||
                      mode == static_cast<MotionMode>(static_cast<int>(MotionMode::Leg1)+leg);
        const bool leg_test=mode>=MotionMode::Leg1&&mode<=MotionMode::Leg4;
        const float scale=leg_test?1.0f:((leg==0||leg==3)?left_scale:right_scale);
        moving=moving&&(leg_test||advanced.drive.stepping||std::fabs(scale)>0.0001f);
        if (moving) {
            float lp = phase;
            if (mode == MotionMode::Trot) {
                lp = wrap(phase + ((leg==1 || leg==3) ? 0.5f : 0.0f));
                const bool swing_phase=lp<0.5f;
                const float u=swing_phase?lp*2.0f:(lp-0.5f)*2.0f;
                PyAppleFootPoint point{};
                const bool generated=swing_phase?pyAppleSwingFoot(u,point):
                                                   pyAppleSupportFoot(u,point);
                if(!generated){error=V10PreflightError::InvalidParameters;return false;}
                // Original Py-Apple emits x in 0..S. Translate by S/2 so
                // stand_forward/body_x and total stroke retain their meanings.
                // Robot forward is still negated exactly once at the IK seam.
                x+=p.step_length_mm*scale*(point.x-0.5f);
                if(swing_phase){down-=p.lift_mm*point.z;next.swing[leg]=true;}
            } else {
            float swing_fraction = 0.5f;
            if (mode == MotionMode::Walk) {
                lp = wrap(phase - 0.25f*leg);
                swing_fraction = 0.2f;
            }
            if (mode >= MotionMode::Leg1 && mode <= MotionMode::Leg4) lp = phase;
            const float half = p.step_length_mm*scale*0.5f;
            if (lp < swing_fraction) {
                const float t = smooth(lp/swing_fraction);
                x += -half + 2.0f*half*t;
                // Raised cosine: height and vertical velocity are both
                // continuous at lift-off and touch-down.
                const float swing_t = lp/swing_fraction;
                down -= p.lift_mm*0.5f*(1.0f-std::cos(2.0f*kPi*swing_t));
                next.swing[leg] = true;
            } else {
                const float t = smooth((lp-swing_fraction)/(1.0f-swing_fraction));
                x += half - 2.0f*half*t; // support foot moves rearward.
            }
            if (mode == MotionMode::Walk) {
                // This is an open-loop trajectory bias, not an observed body
                // translation.  Tying it to each leg's local gait phase keeps
                // every planted foot moving rearward; a shared sinusoid can
                // otherwise pull a support foot forward near a phase edge.
                if (lp < swing_fraction) {
                    const float t = smooth(lp/swing_fraction);
                    x += scale*(-p.walk_body_shift_mm + 2.0f*p.walk_body_shift_mm*t);
                } else {
                    const float t = smooth((lp-swing_fraction)/(1.0f-swing_fraction));
                    x += scale*(p.walk_body_shift_mm - 2.0f*p.walk_body_shift_mm*t);
                }
            }
            }
        }
        nominal_forward[leg] = x;
        nominal_down[leg] = down;
    }
    }
    if(!(mode==MotionMode::Walk&&advanced.cg_walk_enabled))body.body_x_mm+=advanced.dynamic_body_x_mm;
    if(mode==MotionMode::Trot){float feedforward=0;
        if(!trotFeedforward(p.step_length_mm,advanced.drive,advanced.trot_feedforward,feedforward)){
            error=V10PreflightError::DriveIntent;return false;}
        body.body_x_mm+=feedforward;
    }
    float transformed_forward[4]={},transformed_down[4]={};
    if(!V10BodyPose(config_.body_length_mm,config_.body_width_mm).apply(
       nominal_forward,nominal_down,body,transformed_forward,transformed_down)) {
        error=V10PreflightError::BodyTransform; return false;
    }
    for (int leg=0; leg<4; ++leg) {
        const float x=transformed_forward[leg],down=transformed_down[leg];
        next.forward_mm[leg] = x; next.down_mm[leg] = down;
        V10LegAngles a{};
        // Sole body->IK adapter: robot forward is the local negative X axis.
        if (!V10LegIk(config_.geometry).solve(-x, down, KneeFold::Clockwise, a)) {
            error=V10PreflightError::Ik; failed_leg=leg; return false;
        }
        const float hip_delta = config_.hip_direction[leg] *
            (a.hip_deg-config_.hip_geometry_neutral_deg);
        const float q = config_.knee_fold_neutral_deg-std::fabs(a.knee_turn_deg);
        float u = 0;
        if (!finite(hip_delta) || std::fabs(hip_delta) > config_.hip_window_deg) {
            error=V10PreflightError::HipWindow; failed_leg=leg; return false;
        }
        if(!finite(q) || q < 0 ||
           !V10ShankLinearMap(config_.shank_points, config_.shank_point_count).uFromQ(q,u)) {
            error=V10PreflightError::ShankMap; failed_leg=leg; return false;
        }
        const float hip = centers[2*leg] + hip_delta;
        const float shank = centers[2*leg+1] + config_.shank_direction[leg]*u;
        if (!finite(hip) || !finite(shank) || hip < 0 || hip > 180 || shank < 0 || shank > 180) {
            error=V10PreflightError::ServoRange; failed_leg=leg; return false;
        }
        next.servo_deg[2*leg] = hip;
        next.servo_deg[2*leg+1] = shank;
        next.pulse_us[2*leg] = pulseFor(hip);
        next.pulse_us[2*leg+1] = pulseFor(shank);
    }
    out = next;
    error=V10PreflightError::None;
    return true;
}

void V10PreflightCursor::finish(const V10PreflightReport& report) {
    report_ = report;
    finished_ = true;
}

bool V10PreflightCursor::beginAdvanced(
        const V10MotionConfig& config, MotionMode mode,
        const V10MotionParameters& parameters,
        const V10AdvancedMotionParameters& advanced,
        const std::uint8_t centers[8]) {
    *this = V10PreflightCursor{};
    started_ = true;
    config_ = config;
    mode_ = mode;
    parameters_ = parameters;
    advanced_ = advanced;
    if (centers) std::memcpy(centers_, centers, sizeof(centers_));
    const V10Gait gait(config_);
    if (!centers || !gait.parametersValid(parameters_) ||
        !gait.advancedValid(advanced_) || mode_ == MotionMode::Idle ||
        mode_ == MotionMode::Hold || mode_ == MotionMode::HoldError) {
        finished_ = true;
        return false;
    }
    kind_ = Kind::Advanced;
    const float sample_ms = static_cast<float>(config_.update_ms < 5U ?
                                                config_.update_ms : 5U);
    samples_ = mode_ == MotionMode::Stand ? 1 :
        static_cast<int>(std::ceil(parameters_.period_s * 1000.0f / sample_ms));
    if (mode_ == MotionMode::Walk && advanced_.cg_walk_enabled)
        report_.min_support_margin_mm = 1.0e6f;
    finished_ = false;
    return true;
}

bool V10PreflightCursor::beginPlannedCg(
        const V10MotionConfig& config, const V10MotionParameters& parameters,
        const V10AdvancedMotionParameters& advanced,
        const std::uint8_t centers[8],
        const V10CgPlannerConfig& planner_config, float dynamic_envelope,
        float dynamic_slew) {
    *this = V10PreflightCursor{};
    started_ = true;
    config_ = config;
    mode_ = MotionMode::Walk;
    parameters_ = parameters;
    advanced_ = advanced;
    planner_config_ = planner_config;
    dynamic_envelope_ = dynamic_envelope;
    dynamic_slew_ = dynamic_slew;
    if (centers) std::memcpy(centers_, centers, sizeof(centers_));
    const V10Gait gait(config_);
    if (!centers || !gait.parametersValid(parameters_) ||
        !gait.advancedValid(advanced_) || !advanced_.cg_walk_enabled ||
        !finite(dynamic_envelope_) || dynamic_envelope_ < 0 ||
        dynamic_envelope_ > 0.5f || !finite(dynamic_slew_) ||
        dynamic_slew_ < 0 || dynamic_slew_ > 10 ||
        !cgTargets(parameters_, advanced_, front_target_, rear_target_)) {
        finished_ = true;
        return false;
    }
    kind_ = Kind::PlannedCg;
    scenario_count_ = dynamic_envelope_ > 0 ? 3 : 1;
    report_.min_support_margin_mm = 1.0e6f;
    finished_ = false;
    beginPlannedScenario();
    if (!planner_.active()) {
        finished_ = true;
        return false;
    }
    return true;
}

void V10PreflightCursor::beginPlannedScenario() {
    const float scenarios[3] = {-dynamic_envelope_, dynamic_envelope_, 0.0f};
    planner_ = V10CgPlanner(planner_config_);
    planner_.begin(advanced_.body.body_x_mm, front_target_, rear_target_);
    dynamic_ = scenarios[scenario_];
    dynamic_direction_ = 1.0f;
    sample_index_ = 0;
    plan_ = V10CgPlanOutput{};
}

bool V10PreflightCursor::stepAdvanced() {
    const V10Gait gait(config_);
    if (sample_index_ < samples_) {
        const float phase = samples_ == 1 ? 0.0f :
            static_cast<float>(sample_index_) / samples_;
        V10MotionFrame current{};
        const V10PreflightReport frame_report = gait.frameAdvancedReport(
            mode_, phase, parameters_, advanced_, centers_, current);
        ++processed_frames_;
        if (!frame_report.ok) {
            finish(frame_report);
            return true;
        }
        if (mode_ == MotionMode::Walk && advanced_.cg_walk_enabled &&
            frame_report.min_support_margin_mm < report_.min_support_margin_mm)
            report_.min_support_margin_mm = frame_report.min_support_margin_mm;
        ++sample_index_;
        return false;
    }
    report_.ok = true;
    report_.error = V10PreflightError::None;
    finish(report_);
    return true;
}

bool V10PreflightCursor::stepPlannedCg() {
    const V10Gait gait(config_);
    if (!planner_.tick(0.02f, parameters_.period_s,
                       advanced_.body.body_x_mm, front_target_, rear_target_, plan_)) {
        V10PreflightReport failed = report_;
        failed.error = V10PreflightError::InvalidParameters;
        finish(failed);
        return true;
    }
    if (scenario_ == 2 && dynamic_envelope_ > 0) {
        dynamic_ += dynamic_direction_ * dynamic_slew_ * 0.02f;
        if (dynamic_ >= dynamic_envelope_) {
            dynamic_ = dynamic_envelope_;
            dynamic_direction_ = -1;
        }
        if (dynamic_ <= -dynamic_envelope_) {
            dynamic_ = -dynamic_envelope_;
            dynamic_direction_ = 1;
        }
    }
    V10AdvancedMotionParameters runtime = advanced_;
    runtime.cg_body_x_override_valid = true;
    runtime.cg_body_x_override_mm = plan_.body_x_mm;
    runtime.dynamic_body_x_mm = dynamic_;
    V10MotionFrame frame{};
    const V10PreflightReport frame_report = gait.frameAdvancedReport(
        MotionMode::Walk, plan_.phase, parameters_, runtime, centers_, frame);
    ++processed_frames_;
    if (!frame_report.ok) {
        finish(frame_report);
        return true;
    }
    if (frame_report.min_support_margin_mm < report_.min_support_margin_mm)
        report_.min_support_margin_mm = frame_report.min_support_margin_mm;
    if (plan_.cycle_count >= 1) {
        actual_cycle_s_ = std::fmax(actual_cycle_s_, plan_.actual_elapsed_s);
        ++scenario_;
        if (scenario_ >= scenario_count_) {
            report_.ok = true;
            report_.error = V10PreflightError::None;
            finish(report_);
            return true;
        }
        beginPlannedScenario();
        if (!planner_.active()) {
            V10PreflightReport failed = report_;
            failed.error = V10PreflightError::InvalidParameters;
            finish(failed);
            return true;
        }
        return false;
    }
    if (sample_index_ == 1999) {
        V10PreflightReport failed = report_;
        failed.error = V10PreflightError::InvalidParameters;
        finish(failed);
        return true;
    }
    ++sample_index_;
    return false;
}

bool V10PreflightCursor::step() {
    if (finished_) return true;
    if (!started_) {
        finished_ = true;
        return true;
    }
    return kind_ == Kind::Advanced ? stepAdvanced() : stepPlannedCg();
}

bool V10Gait::preflight(MotionMode mode, const V10MotionParameters& p,
                        const std::uint8_t centers[8]) const {
    return preflightAdvanced(mode,p,V10AdvancedMotionParameters{},centers).ok;
}

V10PreflightReport V10Gait::preflightAdvanced(MotionMode mode,
                        const V10MotionParameters& p,
                        const V10AdvancedMotionParameters& advanced,
                        const std::uint8_t centers[8]) const {
    V10PreflightCursor cursor;
    if (!cursor.beginAdvanced(config_, mode, p, advanced, centers))
        return cursor.report();
    while (!cursor.step()) {}
    return cursor.report();
}

V10PreflightReport V10Gait::preflightPlannedCg(
        const V10MotionParameters& p,const V10AdvancedMotionParameters& advanced,
        const std::uint8_t centers[8],const V10CgPlannerConfig& planner_config,
        float dynamic_envelope,float dynamic_slew,float& actual_cycle_s) const {
    V10PreflightCursor cursor;
    actual_cycle_s = 0;
    if (!cursor.beginPlannedCg(config_, p, advanced, centers, planner_config,
                               dynamic_envelope, dynamic_slew))
        return cursor.report();
    while (!cursor.step()) {}
    actual_cycle_s = cursor.actualCycleS();
    return cursor.report();
}

} // namespace dog
