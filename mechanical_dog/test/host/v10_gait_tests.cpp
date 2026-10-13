#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include "Board/ApplepyV1.h"
#include "Ctrl/ServoController.h"
#include "Ctrl/V10ControlProfile.h"
#include "Ctrl/V10Gait.h"

namespace {
struct Clock : dog::IClock {
    std::uint32_t now=0;
    std::uint32_t millis() const override { return now; }
    void delayMs(std::uint32_t ms) override { now+=ms; }
};
struct Pwm : dog::IServoPwm {
    int off_count=0, calls=0, fail_on=-1;
    std::vector<int> events;
    bool begin() override { return true; }
    bool allOff() override { ++off_count; events.push_back(-1); return true; }
    bool offChannel(std::uint8_t) override { return true; }
    bool setPulseUs(std::uint8_t ch,std::uint16_t) override {
        ++calls; if (calls==fail_on) return false; events.push_back(ch); return true;
    }
};
bool close(float a,float b,float e=0.03f) { return std::fabs(a-b)<e; }
struct Point {float x,y;};
float supportMargin(const dog::V10MotionFrame& frame,int swing_leg) {
    const float hx[4]={71,71,-71,-71},hy[4]={46.4f,-46.4f,-46.4f,46.4f};
    Point p[3];int used=0;
    for(int leg=0;leg<4;++leg)if(leg!=swing_leg)p[used++]={hx[leg]+frame.forward_mm[leg],hy[leg]};
    float sign=0,margin=1.0e9f;
    for(int i=0;i<3;++i) {
        const Point a=p[i],b=p[(i+1)%3];
        const float cross=(b.x-a.x)*(-a.y)-(b.y-a.y)*(-a.x);
        if(i==0)sign=cross;
        else if(cross*sign<0)return -1;
        margin=std::fmin(margin,std::fabs(cross)/std::hypot(b.x-a.x,b.y-a.y));
    }
    return margin;
}
std::uint64_t frameHash(dog::V10Gait& gait,dog::MotionMode mode,
                        const dog::V10MotionParameters& p,
                        const dog::V10AdvancedMotionParameters& advanced,
                        const std::uint8_t centers[8]) {
    std::uint64_t hash=1469598103934665603ULL;
    const auto mix=[&](const void* data,std::size_t size){const auto* bytes=
        static_cast<const std::uint8_t*>(data);for(std::size_t i=0;i<size;++i){hash^=bytes[i];hash*=1099511628211ULL;}};
    const int samples=static_cast<int>(p.period_s*200.0f+0.5f);
    for(int i=0;i<samples;++i){dog::V10MotionFrame frame{};
        assert(gait.frameAdvanced(mode,static_cast<float>(i)/samples,p,advanced,centers,frame));
        mix(frame.forward_mm,sizeof(frame.forward_mm));mix(frame.down_mm,sizeof(frame.down_mm));
        mix(frame.servo_deg,sizeof(frame.servo_deg));mix(frame.pulse_us,sizeof(frame.pulse_us));
        mix(frame.swing,sizeof(frame.swing));}
    return hash;
}
}

int main() {
    using namespace dog;
    V10Gait gait(board::kV10MotionConfig);
    const V10ControlProfile built_in_profile{};
    V10MotionParameters built_in{};
    assert(close(built_in.step_length_mm,17.9f)&&close(built_in.lift_mm,12.8f)&&
           close(built_in.period_s,0.5f)&&close(built_in.stand_down_mm,114.0f)&&
           close(built_in.stand_forward_mm,0.0f)&&close(built_in.walk_body_shift_mm,2.0f));
    V10MotionParameters p=built_in;
    p.step_length_mm=6;p.lift_mm=2;p.period_s=1.2f;p.stand_down_mm=108;
    std::uint8_t centers[8]={90,90,90,90,90,90,90,90};
    assert(gait.parametersValid(p));
    assert(gait.preflight(MotionMode::Stand,p,centers));
    assert(gait.preflight(MotionMode::Walk,p,centers));
    assert(gait.preflight(MotionMode::Trot,p,centers));
    for (int m=static_cast<int>(MotionMode::Leg1);m<=static_cast<int>(MotionMode::Leg4);++m)
        assert(gait.preflight(static_cast<MotionMode>(m),p,centers));
    V10MotionParameters walk_p=p; walk_p.period_s=2.5f;
    assert(gait.preflight(MotionMode::Walk,walk_p,centers));

    // 0.8.5 red test: the original Py-Apple Trot holds swing X through the
    // first quarter and uses linear support return. The pre-0.8.5 smoothstep
    // kernel moves both at these samples, so these assertions must fail first.
    V10MotionParameters original_trot=p;
    V10MotionFrame original_sample{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.0625f,original_trot,{},centers,
                              original_sample)); // swing u=.125
    assert(close(original_sample.forward_mm[0],-3.0f,0.001f));
    assert(close(original_sample.down_mm[0],107.6875f,0.001f));
    assert(gait.frameAdvanced(MotionMode::Trot,0.125f,original_trot,{},centers,
                              original_sample)); // support u=.25
    assert(close(original_sample.forward_mm[1],1.5f,0.001f));
    V10MotionFrame trot_start{},trot_quarter{},trot_swap{},trot_end{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.0f,original_trot,{},centers,trot_start));
    assert(gait.frameAdvanced(MotionMode::Trot,0.25f,original_trot,{},centers,trot_quarter));
    assert(gait.frameAdvanced(MotionMode::Trot,0.5f,original_trot,{},centers,trot_swap));
    assert(gait.frameAdvanced(MotionMode::Trot,1.0f,original_trot,{},centers,trot_end));
    assert(trot_start.swing[0]&&!trot_start.swing[1]&&trot_start.swing[2]&&!trot_start.swing[3]);
    assert(!trot_swap.swing[0]&&trot_swap.swing[1]&&!trot_swap.swing[2]&&trot_swap.swing[3]);
    assert(close(trot_start.forward_mm[0],-3)&&close(trot_start.forward_mm[1],3));
    assert(close(trot_quarter.forward_mm[0],0)&&close(trot_quarter.down_mm[0],106));
    for(int leg=0;leg<4;++leg){assert(close(trot_start.forward_mm[leg],trot_end.forward_mm[leg]));
        assert(close(trot_start.down_mm[leg],trot_end.down_mm[leg]));}
    V10MotionFrame support_early{},support_late{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.55f,original_trot,{},centers,support_early));
    assert(gait.frameAdvanced(MotionMode::Trot,0.70f,original_trot,{},centers,support_late));
    assert(support_late.forward_mm[0]<support_early.forward_mm[0]);
    V10AdvancedMotionParameters reverse_trot{};reverse_trot.drive={-1,0,false};
    V10MotionFrame reverse_sample{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.125f,original_trot,reverse_trot,centers,reverse_sample));
    assert(close(reverse_sample.forward_mm[0],3.0f)&&close(original_sample.forward_mm[0],-3.0f));
    V10AdvancedMotionParameters turn_trot{};turn_trot.drive={0,1,false};
    V10MotionFrame turn_sample{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.125f,original_trot,turn_trot,centers,turn_sample));
    assert(turn_sample.forward_mm[0]*turn_sample.forward_mm[2]<0);
    V10AdvancedMotionParameters step_trot{};step_trot.drive={0,0,true};
    V10MotionFrame step_sample{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.25f,original_trot,step_trot,centers,step_sample));
    for(int leg=0;leg<4;++leg)assert(close(step_sample.forward_mm[leg],0));
    assert(step_sample.down_mm[0]<original_trot.stand_down_mm);

    const std::uint8_t asymmetric[8]={95,100,85,80,100,80,85,100};
    const std::uint8_t user_centers[8]={90,100,90,90,90,97,90,102};
    const V10DriveIntent candidate_intents[5]={{1,0,false},{-1,0,false},{0,1,false},
                                                {0,-1,false},{0,0,true}};
    for(const std::uint8_t* set:{user_centers,asymmetric})for(const auto& intent:candidate_intents){
        V10AdvancedMotionParameters candidate{};candidate.drive=intent;
        candidate.body.body_x_mm=8.0f;
        const auto stand_ok=gait.preflightAdvanced(MotionMode::Stand,built_in,candidate,set);
        assert(stand_ok.ok);
        const auto trot_ok=gait.preflightAdvanced(MotionMode::Trot,built_in,candidate,set);
        assert(trot_ok.ok);
        V10MotionParameters step_edge=built_in;step_edge.step_length_mm=17.9f;
        assert(gait.preflightAdvanced(MotionMode::Trot,step_edge,candidate,set).ok);
        V10MotionParameters walk_step_edge=built_in;walk_step_edge.step_length_mm=25.3f;
        walk_step_edge.lift_mm=7.1f;walk_step_edge.stand_down_mm=108;
        assert(gait.preflightAdvanced(MotionMode::Walk,walk_step_edge,candidate,set).ok);
        V10MotionParameters next_step=walk_step_edge;next_step.step_length_mm=25.4f;
        const auto walk_step_next=gait.preflightAdvanced(MotionMode::Walk,next_step,candidate,set);
        if(intent.stepping)assert(walk_step_next.ok);
        else assert(!walk_step_next.ok&&walk_step_next.error==V10PreflightError::HipWindow&&
                    walk_step_next.leg==0&&
                    close(walk_step_next.phase,intent.forward<0||intent.turn>0?0.16f:0.04f,0.0001f));
        V10MotionParameters trot_next=built_in;trot_next.step_length_mm=16;
        trot_next.lift_mm=13.2f;
        assert(gait.preflightAdvanced(MotionMode::Trot,trot_next,candidate,set).ok);
        V10AdvancedMotionParameters boundary_candidate{};
        boundary_candidate.drive=intent;
        const auto trot_reject=gait.preflightAdvanced(MotionMode::Trot,trot_next,boundary_candidate,set);
        assert(!trot_reject.ok&&trot_reject.error==V10PreflightError::ShankMap&&
               close(trot_reject.phase,0.25f,0.0001f)&&trot_reject.leg==0);
        V10MotionParameters trot_edge=built_in;trot_edge.step_length_mm=16;
        trot_edge.lift_mm=13.4f;
        assert(gait.preflightAdvanced(MotionMode::Trot,trot_edge,candidate,set).ok);
        trot_edge.lift_mm=13.5f;
        assert(gait.preflightAdvanced(MotionMode::Trot,trot_edge,candidate,set).error==
               V10PreflightError::ShankMap);
        V10MotionParameters walk_candidate=built_in;
        walk_candidate.step_length_mm=6;walk_candidate.lift_mm=7.1f;
        walk_candidate.period_s=0.5f;walk_candidate.stand_down_mm=108;
        walk_candidate.stand_forward_mm=0;walk_candidate.walk_body_shift_mm=2;
        const auto walk_ok=gait.preflightAdvanced(MotionMode::Walk,walk_candidate,candidate,set);
        assert(walk_ok.ok);
        walk_candidate.lift_mm=7.2f;
        assert(gait.preflightAdvanced(MotionMode::Walk,walk_candidate,candidate,set).ok);
        const auto walk_reject=gait.preflightAdvanced(MotionMode::Walk,walk_candidate,boundary_candidate,set);
        assert(!walk_reject.ok&&walk_reject.error==V10PreflightError::ShankMap&&
               close(walk_reject.phase,0.10f,0.0001f)&&walk_reject.leg==0);
        walk_candidate.lift_mm=7.4f;
        assert(gait.preflightAdvanced(MotionMode::Walk,walk_candidate,candidate,set).ok);
        walk_candidate.lift_mm=7.5f;
        assert(gait.preflightAdvanced(MotionMode::Walk,walk_candidate,candidate,set).error==
               V10PreflightError::ShankMap);
    }
    // 0.8.10: Trot-only open-loop Pitch offset is applied before BodyPose.
    // The diagnosed -0.5 degree endpoint is valid only after lowering the
    // built-in Trot lift from 13.1 to 12.8 mm. It must not alter other modes.
    for(const std::uint8_t* set:{user_centers,asymmetric})for(const auto& intent:candidate_intents){
        V10AdvancedMotionParameters pitch{};pitch.body.body_x_mm=8.0f;
        pitch.drive=intent;pitch.trot_pitch_offset_deg=-0.5f;
        assert(gait.preflightAdvanced(MotionMode::Trot,built_in,pitch,set).ok);
    }
    V10AdvancedMotionParameters limiting_pitch{};limiting_pitch.body.body_x_mm=8.0f;
    limiting_pitch.trot_pitch_offset_deg=-0.5f;
    V10MotionParameters pitch_next_lift=built_in;pitch_next_lift.lift_mm=12.9f;
    const auto pitch_next_report=gait.preflightAdvanced(
        MotionMode::Trot,pitch_next_lift,limiting_pitch,user_centers);
    assert(!pitch_next_report.ok&&pitch_next_report.error==V10PreflightError::ShankMap&&
           close(pitch_next_report.phase,0.25f,0.0001f)&&pitch_next_report.leg==0);
    V10AdvancedMotionParameters explicit_zero{};explicit_zero.body.body_x_mm=8.0f;
    V10AdvancedMotionParameters negative_pitch=explicit_zero;
    negative_pitch.trot_pitch_offset_deg=-0.5f;
    for(MotionMode unaffected:{MotionMode::Stand,MotionMode::Walk,MotionMode::Leg1,
                               MotionMode::Leg2,MotionMode::Leg3,MotionMode::Leg4})
        assert(frameHash(gait,unaffected,built_in,explicit_zero,user_centers)==
               frameHash(gait,unaffected,built_in,negative_pitch,user_centers));
    assert(frameHash(gait,MotionMode::Trot,built_in,explicit_zero,user_centers)!=
           frameHash(gait,MotionMode::Trot,built_in,negative_pitch,user_centers));
    V10MotionFrame pitch_zero_frame{},pitch_negative_frame{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,built_in,explicit_zero,
                              user_centers,pitch_zero_frame));
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,built_in,negative_pitch,
                              user_centers,pitch_negative_frame));
    assert(!close(pitch_zero_frame.down_mm[0],pitch_negative_frame.down_mm[0],0.001f));
    V10AdvancedMotionParameters combined_out_of_range=negative_pitch;
    combined_out_of_range.body.pitch_deg=kBodyPitchMinDeg;
    const auto combined_reject=gait.preflightAdvanced(
        MotionMode::Trot,built_in,combined_out_of_range,user_centers);
    assert(!combined_reject.ok&&combined_reject.error==V10PreflightError::BodyTransform);
    V10AdvancedMotionParameters base_out_of_range=explicit_zero;
    base_out_of_range.body.pitch_deg=kBodyPitchMinDeg-0.1f;
    assert(!gait.preflightAdvanced(MotionMode::Trot,built_in,base_out_of_range,user_centers).ok);
    V10AdvancedMotionParameters positive_offset=explicit_zero;
    positive_offset.trot_pitch_offset_deg=0.1f;
    assert(!gait.preflightAdvanced(MotionMode::Trot,built_in,positive_offset,user_centers).ok);
    V10AdvancedMotionParameters off_grid_offset=explicit_zero;
    off_grid_offset.trot_pitch_offset_deg=-0.25f;
    assert(!gait.preflightAdvanced(MotionMode::Trot,built_in,off_grid_offset,user_centers).ok);
    V10MotionParameters hard_step_limit=built_in;hard_step_limit.step_length_mm=30.1f;
    assert(!gait.parametersValid(hard_step_limit));
    assert(close(built_in_profile.body.pitch_deg,0.0f)&&
           close(built_in_profile.body.roll_deg,0.0f)&&
           close(built_in_profile.body.body_x_mm,8.0f));
    assert(!built_in_profile.cg_walk_enabled&&
           !built_in_profile.trot_feedforward.enabled);
    // This valid trajectory was rejected in 0.8.5 only by the coordinated
    // servo-speed threshold. Geometry, mapping and electrical range now decide.
    V10MotionParameters formerly_speed_limited=built_in;
    formerly_speed_limited.step_length_mm=20;
    formerly_speed_limited.lift_mm=6;
    formerly_speed_limited.period_s=0.7f;
    formerly_speed_limited.stand_down_mm=115;
    assert(gait.preflightAdvanced(MotionMode::Trot,formerly_speed_limited,
                                  {},user_centers).ok);

    V10MotionParameters original_h30=built_in;original_h30.step_length_mm=30;
    original_h30.lift_mm=30;original_h30.period_s=10;original_h30.stand_down_mm=110;
    auto original_reject=gait.preflightAdvanced(MotionMode::Trot,original_h30,{},user_centers);
    assert(!original_reject.ok&&original_reject.error==V10PreflightError::HipWindow);
    original_h30.step_length_mm=0;
    original_reject=gait.preflightAdvanced(MotionMode::Trot,original_h30,{},user_centers);
    assert(!original_reject.ok&&original_reject.error==V10PreflightError::ShankMap);
    original_h30.step_length_mm=30;original_h30.period_s=0.1f;
    original_reject=gait.preflightAdvanced(MotionMode::Trot,original_h30,{},user_centers);
    assert(!original_reject.ok&&original_reject.error==V10PreflightError::InvalidParameters);
    assert(gait.preflight(MotionMode::Trot,p,asymmetric));
    assert(gait.preflight(MotionMode::Walk,walk_p,asymmetric));
    V10MotionParameters slow=p;
    slow.step_length_mm=3; slow.lift_mm=1; slow.period_s=3; slow.walk_body_shift_mm=1;
    assert(gait.preflight(MotionMode::Walk,slow,centers));
    assert(gait.preflight(MotionMode::Trot,slow,centers));
    assert(gait.preflight(MotionMode::Walk,slow,asymmetric));
    assert(gait.preflight(MotionMode::Trot,slow,asymmetric));
    V10MotionParameters non_multiple=p; non_multiple.period_s=1.23f;
    assert(gait.preflight(MotionMode::Trot,non_multiple,centers));

    // Advanced body pose is zero-compatible and applies one global body
    // translation before the existing sole body->IK X sign conversion.
    V10AdvancedMotionParameters advanced{};
    V10MotionFrame legacy_zero{},advanced_zero{},body_x_frame{};
    assert(gait.frame(MotionMode::Stand,0,p,centers,legacy_zero));
    assert(gait.frameAdvanced(MotionMode::Stand,0,p,advanced,centers,advanced_zero));
    for(int i=0;i<8;++i) assert(close(legacy_zero.servo_deg[i],advanced_zero.servo_deg[i]));
    advanced.body.body_x_mm=3;
    assert(gait.frameAdvanced(MotionMode::Stand,0,p,advanced,centers,body_x_frame));
    for(int leg=0;leg<4;++leg)
        assert(close(body_x_frame.forward_mm[leg],legacy_zero.forward_mm[leg]-3));

    // New CG Walk is opt-in. Transfer stages keep all feet down and apply one
    // equal global body shift; swing order is 1,2,3,4.
    advanced={}; advanced.cg_walk_enabled=true; advanced.cg_shift_mm=2.5f;
    V10MotionFrame cg{};
    assert(gait.frameAdvanced(MotionMode::Walk,0.05f,walk_p,advanced,centers,cg));
    for(bool swinging:cg.swing) assert(!swinging);
    for(int leg=1;leg<4;++leg)
        assert(close(cg.forward_mm[leg]-walk_p.stand_forward_mm,
                     cg.forward_mm[0]-walk_p.stand_forward_mm));
    const float cg_swing_phases[4]={0.17f,0.34f,0.60f,0.77f};
    for(int selected=0;selected<4;++selected) {
        assert(gait.frameAdvanced(MotionMode::Walk,cg_swing_phases[selected],walk_p,
                                  advanced,centers,cg));
        for(int leg=0;leg<4;++leg) assert(cg.swing[leg]==(leg==selected));
        assert(supportMargin(cg,selected)>1.0f); // nominal body-centre COM only
    }
    V10MotionFrame transfer_a{},transfer_b{};
    assert(gait.frameAdvanced(MotionMode::Walk,0.435f,walk_p,advanced,centers,transfer_a));
    assert(gait.frameAdvanced(MotionMode::Walk,0.515f,walk_p,advanced,centers,transfer_b));
    for(int leg=0;leg<4;++leg) assert(!transfer_a.swing[leg]&&!transfer_b.swing[leg]);
    auto transfer_x=[](float phase){const float t=(phase-0.425f)/0.10f;
        return -2.5f+11.0f*t*t*(3.0f-2.0f*t);};
    for(int leg=0;leg<4;++leg)
        assert(close(transfer_x(0.435f)+transfer_a.forward_mm[leg],
                     transfer_x(0.515f)+transfer_b.forward_mm[leg],0.02f));
    V10MotionParameters bias_changed=walk_p; bias_changed.walk_body_shift_mm=0;
    V10MotionFrame no_bias{},with_bias{};
    assert(gait.frameAdvanced(MotionMode::Walk,0.34f,walk_p,advanced,centers,with_bias));
    assert(gait.frameAdvanced(MotionMode::Walk,0.34f,bias_changed,advanced,centers,no_bias));
    for(int i=0;i<8;++i) assert(close(no_bias.servo_deg[i],with_bias.servo_deg[i]));
    V10MotionParameters cg_walk_p=walk_p; cg_walk_p.period_s=3.2f;
    const V10PreflightReport cg_ok=gait.preflightAdvanced(MotionMode::Walk,cg_walk_p,advanced,centers);
    assert(cg_ok.ok && cg_ok.min_support_margin_mm>1.0f);
    V10MotionParameters shifted_walk=cg_walk_p;shifted_walk.stand_forward_mm=2;
    advanced.body.body_x_mm=3;
    const V10PreflightReport shifted_ok=gait.preflightAdvanced(
        MotionMode::Walk,shifted_walk,advanced,centers);
    assert(shifted_ok.ok && shifted_ok.min_support_margin_mm>1.0f);
    advanced.body.body_x_mm=0;advanced.body.roll_deg=2;
    const V10PreflightReport rolled_cg=gait.preflightAdvanced(
        MotionMode::Walk,cg_walk_p,advanced,centers);
    assert(!rolled_cg.ok && rolled_cg.error==V10PreflightError::SupportMargin &&
           rolled_cg.min_support_margin_mm<0.0f);

    // Signed drive intent is RAM-only and does not reinterpret stored step.
    V10AdvancedMotionParameters drive{};
    V10MotionFrame forward_drive{},reverse_drive{},turn_drive{},step_drive{};
    drive.drive={1,0,false};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,drive,centers,forward_drive));
    drive.drive={-1,0,false};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,drive,centers,reverse_drive));
    assert((forward_drive.forward_mm[0]-p.stand_forward_mm)*
           (reverse_drive.forward_mm[0]-p.stand_forward_mm)<0);
    drive.drive={0,1,false};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,drive,centers,turn_drive));
    assert((turn_drive.forward_mm[0]-p.stand_forward_mm)*
           (turn_drive.forward_mm[2]-p.stand_forward_mm)<0); // left/right opposite
    drive.drive={0,0,true};
    assert(gait.frameAdvanced(MotionMode::Walk,0.05f,walk_p,drive,centers,step_drive));
    assert(close(step_drive.forward_mm[0],walk_p.stand_forward_mm));
    assert(step_drive.down_mm[0]<walk_p.stand_down_mm); // lift with no fore/aft or bias
    drive.cg_walk_enabled=true;drive.cg_shift_mm=2.5f;drive.drive={-1,0,false};
    assert(gait.preflightAdvanced(MotionMode::Walk,cg_walk_p,drive,centers).ok);
    V10MotionParameters turn_cg_p=cg_walk_p;turn_cg_p.period_s=10.0f;
    drive.drive={0,1,false};
    assert(gait.preflightAdvanced(MotionMode::Walk,turn_cg_p,drive,centers).ok);
    drive.drive={0,-1,false};
    assert(gait.preflightAdvanced(MotionMode::Walk,turn_cg_p,drive,centers).ok);
    drive.drive={1,0,false};drive.dynamic_body_x_mm=0.5f;drive.body.roll_deg=0;
    assert(gait.preflightAdvanced(MotionMode::Walk,cg_walk_p,drive,centers).ok);
    drive.dynamic_body_x_mm=0;
    float actual_cycle_s=0;
    const auto planned=gait.preflightPlannedCg(cg_walk_p,drive,centers,
        {20.0f,0.05f,15000},0.5f,2.0f,actual_cycle_s);
    assert(planned.ok&&actual_cycle_s>3.2f&&actual_cycle_s<5.0f);
    V10AdvancedMotionParameters hash_zero{};
    assert(frameHash(gait,MotionMode::Walk,walk_p,hash_zero,centers)==
           7935719014511397748ULL);
    V10AdvancedMotionParameters hash_cg{};hash_cg.cg_walk_enabled=true;hash_cg.cg_shift_mm=2.5f;
    assert(frameHash(gait,MotionMode::Walk,cg_walk_p,hash_cg,centers)==
           13541710352139227498ULL);

    // Geometry and support-report fixtures remain pinned while speed telemetry
    // is intentionally absent from the report.
    const auto fixture=[](const V10PreflightReport& r,bool ok,V10PreflightError error,
                           int leg,int joint,float phase,float margin){
        assert(r.ok==ok&&r.error==error&&r.leg==leg&&r.joint==joint);
        assert(close(r.phase,phase,0.0002f));
        assert(close(r.min_support_margin_mm,margin,0.002f));
    };
    fixture(gait.preflightAdvanced(MotionMode::Stand,p,{},centers),true,
            V10PreflightError::None,-1,-1,0,0);
    fixture(gait.preflightAdvanced(MotionMode::Walk,p,{},centers),true,
            V10PreflightError::None,-1,-1,0,0);
    fixture(gait.preflightAdvanced(MotionMode::Trot,p,{},centers),true,
            V10PreflightError::None,-1,-1,0,0);
    fixture(gait.preflightAdvanced(MotionMode::Walk,walk_p,{},centers),true,
            V10PreflightError::None,-1,-1,0,0);
    fixture(gait.preflightAdvanced(MotionMode::Walk,walk_p,{},asymmetric),true,
            V10PreflightError::None,-1,-1,0,0);
    V10AdvancedMotionParameters fixture_drive{};fixture_drive.drive={-1,0,false};
    fixture(gait.preflightAdvanced(MotionMode::Trot,p,fixture_drive,centers),true,
            V10PreflightError::None,-1,-1,0,0);
    fixture_drive.drive={0,1,false};
    fixture(gait.preflightAdvanced(MotionMode::Trot,p,fixture_drive,centers),true,
            V10PreflightError::None,-1,-1,0,0);
    fixture_drive.drive={0,0,true};
    fixture(gait.preflightAdvanced(MotionMode::Walk,walk_p,fixture_drive,centers),true,
            V10PreflightError::None,-1,-1,0,0);
    V10AdvancedMotionParameters fixture_cg{};fixture_cg.cg_walk_enabled=true;
    fixture_cg.cg_shift_mm=2.5f;
    fixture(gait.preflightAdvanced(MotionMode::Walk,cg_walk_p,fixture_cg,centers),true,
            V10PreflightError::None,-1,-1,0,1.36764693f);
    fixture_cg.body.roll_deg=2;
    fixture(gait.preflightAdvanced(MotionMode::Walk,cg_walk_p,fixture_cg,centers),false,
            V10PreflightError::SupportMargin,1,-1,0.275000006f,-1);
    fixture_cg.body.roll_deg=0;float fixture_actual=0;
    const auto fixture_planned=gait.preflightPlannedCg(cg_walk_p,fixture_cg,centers,
        {20.0f,0.05f,15000},0.5f,2.0f,fixture_actual);
    fixture(fixture_planned,true,V10PreflightError::None,-1,-1,0,1.09411812f);
    assert(close(fixture_actual,3.81999707f,0.002f));
    std::uint8_t fixture_edge[8]={0,0,180,180,180,180,0,0};
    fixture(gait.preflightAdvanced(MotionMode::Stand,p,{},fixture_edge),false,
            V10PreflightError::ServoRange,0,-1,0,0);
    V10MotionConfig fixture_config=board::kV10MotionConfig;
    fixture_config.geometry={40,40};
    fixture(V10Gait(fixture_config).preflightAdvanced(MotionMode::Stand,p,{},centers),false,
            V10PreflightError::Ik,0,-1,0,0);
    const V10ShankMapPoint fixture_short_map[2]={{0,0},{1,1}};
    fixture_config=board::kV10MotionConfig;fixture_config.shank_points=fixture_short_map;
    fixture_config.shank_point_count=2;
    fixture(V10Gait(fixture_config).preflightAdvanced(MotionMode::Stand,p,{},centers),false,
            V10PreflightError::ShankMap,0,-1,0,0);
    V10PreflightCursor ordinary_cursor;
    assert(ordinary_cursor.beginAdvanced(board::kV10MotionConfig,MotionMode::Walk,
                                         walk_p,{},centers));
    while(!ordinary_cursor.finished()){
        const std::uint32_t before=ordinary_cursor.processedFrames();
        ordinary_cursor.step();
        assert(ordinary_cursor.processedFrames()-before<=1U);
    }
    fixture(ordinary_cursor.report(),true,V10PreflightError::None,-1,-1,0,
            0);
    V10PreflightCursor planned_cursor;
    assert(planned_cursor.beginPlannedCg(board::kV10MotionConfig,cg_walk_p,
        fixture_cg,centers,{20.0f,0.05f,15000},0.5f,2.0f));
    while(!planned_cursor.finished()){
        const std::uint32_t before=planned_cursor.processedFrames();
        planned_cursor.step();
        assert(planned_cursor.processedFrames()-before<=1U);
    }
    fixture(planned_cursor.report(),true,V10PreflightError::None,-1,-1,0,
            1.09411812f);
    assert(close(planned_cursor.actualCycleS(),3.81999707f,0.002f));

    V10AdvancedMotionParameters ff_disabled{},ff_enabled{};
    ff_enabled.trot_feedforward={true,4,4,2,2};
    V10MotionFrame no_ff{},forward_ff{},reverse_ff{},reverse_no_ff{},turn_ff{},turn_no_ff{},step_ff{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_disabled,centers,no_ff));
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_enabled,centers,forward_ff));
    for(int leg=0;leg<4;++leg)assert(forward_ff.forward_mm[leg]<no_ff.forward_mm[leg]);
    ff_enabled.drive={-1,0,false};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_enabled,centers,reverse_ff));
    ff_disabled.drive={-1,0,false};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_disabled,centers,reverse_no_ff));
    ff_enabled.drive={0,1,false};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_enabled,centers,turn_ff));
    ff_disabled.drive={0,1,false};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_disabled,centers,turn_no_ff));
    ff_enabled.drive={0,0,true};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_enabled,centers,step_ff));
    assert(reverse_ff.forward_mm[0]>reverse_no_ff.forward_mm[0]&&
           turn_ff.forward_mm[0]<turn_no_ff.forward_mm[0]);
    assert(close(step_ff.forward_mm[0],p.stand_forward_mm));
    ff_enabled.drive={1,0,false};ff_enabled.dynamic_body_x_mm=0.5f;
    V10MotionFrame ff_plus_dynamic{};
    assert(gait.frameAdvanced(MotionMode::Trot,0.10f,p,ff_enabled,centers,ff_plus_dynamic));
    for(int leg=0;leg<4;++leg)
        assert(close(ff_plus_dynamic.forward_mm[leg],forward_ff.forward_mm[leg]-0.5f,0.03f));

    // Vertical swing begins/ends with zero velocity.  Finite differences at
    // both boundaries must collapse quadratically rather than linearly.
    V10MotionFrame v0{},v1{},v2{};
    assert(gait.frame(MotionMode::Trot,0.0f,p,centers,v0));
    assert(gait.frame(MotionMode::Trot,0.001f,p,centers,v1));
    assert(gait.frame(MotionMode::Trot,0.002f,p,centers,v2));
    assert(std::fabs(v1.down_mm[0]-v0.down_mm[0]) <
           0.6f*std::fabs(v2.down_mm[0]-v1.down_mm[0]));

    V10MotionFrame stand{}, plus{}, minus{};
    assert(gait.frame(MotionMode::Stand,0,p,centers,stand));
    p.stand_forward_mm=5; assert(gait.frame(MotionMode::Stand,0,p,centers,plus));
    p.stand_forward_mm=-5; assert(gait.frame(MotionMode::Stand,0,p,centers,minus));
    // forward is adapted once to local -X. Left/right hip directions mirror.
    assert(plus.servo_deg[0] < minus.servo_deg[0]);
    assert(plus.servo_deg[2] > minus.servo_deg[2]);
    assert(plus.servo_deg[4] > minus.servo_deg[4]);
    assert(plus.servo_deg[6] < minus.servo_deg[6]);
    assert(plus.servo_deg[1] < centers[1] && plus.servo_deg[3] > centers[3]);
    assert(plus.servo_deg[5] > centers[5] && plus.servo_deg[7] < centers[7]);
    p.stand_forward_mm=0;

    // Trot diagonals (1,3) and (2,4) alternate by half a cycle.
    V10MotionFrame trot{};
    assert(gait.frame(MotionMode::Trot,0.25f,p,centers,trot));
    assert(trot.swing[0] && !trot.swing[1] && trot.swing[2] && !trot.swing[3]);
    assert(gait.frame(MotionMode::Trot,0.75f,p,centers,trot));
    assert(!trot.swing[0] && trot.swing[1] && !trot.swing[2] && trot.swing[3]);

    // Walk has at most one swing leg, ordered 1->2->3->4, with support gaps.
    const float swing_phases[4]={0.05f,0.30f,0.55f,0.80f};
    for (int expected=0;expected<4;++expected) {
        V10MotionFrame f{}; assert(gait.frame(MotionMode::Walk,swing_phases[expected],p,centers,f));
        int count=0,index=-1; for (int leg=0;leg<4;++leg) if (f.swing[leg]) {++count;index=leg;}
        assert(count==1 && index==expected);
    }
    for (float phase : {0.225f,0.475f,0.725f,0.975f}) {
        V10MotionFrame f{}; assert(gait.frame(MotionMode::Walk,phase,p,centers,f));
        for (bool swing : f.swing) assert(!swing);
    }
    // Across the complete cycle, every foot that remains in support moves
    // rearward (or holds within numerical tolerance) as phase advances.
    // This guards the trajectory bias as well as the nominal step term.
    constexpr int derivative_samples=20000;
    V10MotionFrame derivative_a{},derivative_b{};
    for (int i=0;i<derivative_samples;++i) {
        const float a=static_cast<float>(i)/derivative_samples;
        const float b=static_cast<float>(i+1)/derivative_samples;
        assert(gait.frame(MotionMode::Walk,a,p,centers,derivative_a));
        assert(gait.frame(MotionMode::Walk,b,p,centers,derivative_b));
        for (int leg=0;leg<4;++leg)
            if (!derivative_a.swing[leg] && !derivative_b.swing[leg])
                assert(derivative_b.forward_mm[leg] <=
                       derivative_a.forward_mm[leg]+0.0001f);
    }
    V10MotionFrame end{}, start{};
    assert(gait.frame(MotionMode::Walk,0.99999f,p,centers,end));
    assert(gait.frame(MotionMode::Walk,0.0f,p,centers,start));
    for (int leg=0;leg<4;++leg) {
        assert(close(end.forward_mm[leg],start.forward_mm[leg]));
        assert(close(end.down_mm[leg],start.down_mm[leg]));
        assert(close(end.servo_deg[2*leg],start.servo_deg[2*leg]));
        assert(close(end.servo_deg[2*leg+1],start.servo_deg[2*leg+1]));
    }

    // Every single-leg test keeps the other three at the exact stand command.
    for (int selected=0; selected<4; ++selected) {
        V10MotionFrame leg{};
        assert(gait.frame(static_cast<MotionMode>(static_cast<int>(MotionMode::Leg1)+selected),
                          0.25f,p,centers,leg));
        for (int other=0; other<4; ++other) {
            if (other == selected) continue;
            assert(close(leg.servo_deg[2*other],stand.servo_deg[2*other]));
            assert(close(leg.servo_deg[2*other+1],stand.servo_deg[2*other+1]));
        }
        assert(!close(leg.servo_deg[2*selected],stand.servo_deg[2*selected]) ||
               !close(leg.servo_deg[2*selected+1],stand.servo_deg[2*selected+1]));
    }

    // Invalid and unreachable requests fail without hardware access.
    V10MotionParameters bad=p; bad.stand_down_mm=140;
    assert(!gait.preflight(MotionMode::Walk,bad,centers));
    bad=p;bad.step_length_mm=30.1f;assert(!gait.parametersValid(bad));
    bad=p;bad.lift_mm=30.1f;assert(!gait.parametersValid(bad));
    bad=p; bad.period_s=0.2f; assert(!gait.preflight(MotionMode::Walk,bad,centers));
    bad=p; bad.step_length_mm=std::numeric_limits<float>::quiet_NaN();
    assert(!gait.preflight(MotionMode::Walk,bad,centers));
    std::uint8_t edge[8]={0,0,180,180,180,180,0,0};
    assert(!gait.preflight(MotionMode::Stand,p,edge));
    V10MotionConfig invalid=board::kV10MotionConfig;
    invalid.geometry.upper_mm=0;
    assert(!V10Gait(invalid).preflight(MotionMode::Stand,p,centers));
    invalid=board::kV10MotionConfig; invalid.channels[7]=invalid.channels[0];
    assert(!V10Gait(invalid).preflight(MotionMode::Stand,p,centers));
    invalid=board::kV10MotionConfig; invalid.hip_direction[0]=0;
    assert(!V10Gait(invalid).preflight(MotionMode::Stand,p,centers));
    invalid=board::kV10MotionConfig; invalid.hip_window_deg=0.1f;
    assert(!V10Gait(invalid).preflight(MotionMode::Stand,p,centers));
    const V10ShankMapPoint duplicate_q[3]={{0,0},{30,9},{50,9}};
    invalid=board::kV10MotionConfig; invalid.shank_points=duplicate_q;
    assert(!V10Gait(invalid).preflight(MotionMode::Stand,p,centers));
    const V10ShankMapPoint insufficient_q[2]={{0,0},{1,1}};
    invalid=board::kV10MotionConfig; invalid.shank_points=insufficient_q;
    invalid.shank_point_count=2;
    assert(!V10Gait(invalid).preflight(MotionMode::Stand,p,centers));

    // Coordinated hardware mode performs complete validation before writing,
    // never allOff between good frames, and shuts down on the Nth failure.
    Clock clock; Pwm pwm;
    ServoController servos(pwm,clock,500,2500,0,board::kSelectedServoChannels,8,500,2500);
    assert(servos.begin() && servos.arm());
    std::uint16_t pulses[8]={1500,1500,1500,1500,1500,1500,1500,1500};
    const int baseline_off=pwm.off_count;
    std::uint8_t duplicate_channels[8]={0,1,2,3,12,9,10,10};
    std::uint8_t bad_channels[8]={0,1,2,3,12,9,10,15};
    assert(servos.coordinatedFrame(duplicate_channels,pulses,8)==ServoResult::OutOfRange);
    assert(servos.coordinatedFrame(board::kSelectedServoChannels,pulses,7)==ServoResult::OutOfRange);
    assert(servos.coordinatedFrame(bad_channels,pulses,8)==ServoResult::OutOfRange);
    assert(pwm.calls==0 && pwm.off_count==baseline_off);
    assert(servos.coordinatedFrame(board::kSelectedServoChannels,pulses,8)==ServoResult::Ok);
    assert(servos.coordinatedFrame(board::kSelectedServoChannels,pulses,8)==ServoResult::Ok);
    assert(pwm.calls==16 && pwm.off_count==baseline_off && servos.coordinated());
    assert(servos.pulse(0,1500)==ServoResult::ModeConflict);
    assert(servos.offChannel(0)==ServoResult::ModeConflict);
    assert(pwm.calls==16 && pwm.off_count==baseline_off);
    servos.off(); assert(servos.arm());
    pwm.fail_on=pwm.calls+5;
    assert(servos.coordinatedFrame(board::kSelectedServoChannels,pulses,8)==ServoResult::HardwareFault);
    assert(servos.faulted() && !servos.armed() && pwm.off_count==baseline_off+2);
    const int failed_calls=pwm.calls;
    assert(servos.coordinatedFrame(board::kSelectedServoChannels,pulses,8)==ServoResult::HardwareFault);
    assert(pwm.calls==failed_calls);

    // A pre-existing single output and calibration session both exclude a
    // coordinated frame without writing any part of it.
    Clock c2; Pwm p2;
    ServoController single(p2,c2,500,2500,0,board::kSelectedServoChannels,8,500,2500);
    assert(single.begin() && single.arm());
    assert(single.pulse(0,1500)==ServoResult::Ok);
    const int after_single=p2.calls;
    assert(single.coordinatedFrame(board::kSelectedServoChannels,pulses,8)==ServoResult::ModeConflict);
    assert(p2.calls==after_single);
    single.off();
    assert(single.startCalibration(board::kSelectedServoChannels,pulses,8)==ServoResult::Ok);
    const int after_calibration=p2.calls;
    assert(single.coordinatedFrame(board::kSelectedServoChannels,pulses,8)==ServoResult::ModeConflict);
    assert(p2.calls==after_calibration);
}
