#include "Ctrl/V10DriveControl.h"

#include <cmath>

namespace dog {
namespace {
float clamp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
bool finite(float v){return std::isfinite(v);}
}

bool driveScales(const V10DriveIntent& intent,float& left,float& right){
    if(!finite(intent.forward)||!finite(intent.turn)||std::fabs(intent.forward)>1.0f||
       std::fabs(intent.turn)>1.0f)return false;
    if(intent.stepping){left=right=0;return true;}
    left=clamp(intent.forward-intent.turn,-1.0f,1.0f);
    right=clamp(intent.forward+intent.turn,-1.0f,1.0f);
    return true;
}
bool applyStraightTrim(const V10DriveIntent& intent,float trim,
                       V10DriveIntent& effective){
    float left=0,right=0;
    if(!driveScales(intent,left,right)||!finite(trim)||std::fabs(trim)>0.30f)return false;
    effective=intent;
    if(intent.stepping||std::fabs(intent.forward)<=0.0001f||
       std::fabs(intent.turn)>0.0001f)return true;
    effective.turn=intent.forward>0.0f?-trim:trim;
    return driveScales(effective,left,right);
}
bool trotFeedforward(float step,const V10DriveIntent& intent,
                     const V10TrotFeedforwardConfig& config,float& offset){
    float left=0,right=0;
    if(!finite(step)||step<0||!driveScales(intent,left,right)||
       !finite(config.forward_gain)||config.forward_gain<0||config.forward_gain>5||
       !finite(config.backward_gain)||config.backward_gain<0||config.backward_gain>5||
       !finite(config.turn_gain)||config.turn_gain<0||config.turn_gain>5||
       !finite(config.max_abs_mm)||config.max_abs_mm<0||config.max_abs_mm>3)return false;
    offset=0;if(!config.enabled||intent.stepping)return true;
    const float s_left=step*left,s_right=step*right;
    const float translation=(s_left+s_right)*0.05f; // average stride / 10
    const float turn=(s_right-s_left)*0.05f;
    if(translation>0.0001f)offset=translation*config.forward_gain;
    else if(translation<-0.0001f)offset=translation*config.backward_gain;
    else if(std::fabs(turn)>0.0001f)offset=std::fabs(turn)*config.turn_gain;
    offset=clamp(offset,-config.max_abs_mm,config.max_abs_mm);return true;
}

bool V10DriveSession::canBegin(std::uint32_t expected,std::uint32_t seq)const{
    return !active_&&!expired_latched_&&lease_ms_!=0&&expected==epoch_&&seq!=0;
}
bool V10DriveSession::begin(std::uint32_t expected,std::uint32_t seq,std::uint32_t now){
    if(!canBegin(expected,seq))return false;
    ++epoch_;if(epoch_==0)++epoch_;
    active_=true;last_seq_=seq;expires_ms_=now+lease_ms_;return true;
}
bool V10DriveSession::renew(std::uint32_t epoch,std::uint32_t seq,std::uint32_t now){
    if(active_&&static_cast<std::int32_t>(now-expires_ms_)>=0){
        active_=false;last_seq_=0;expires_ms_=0;++epoch_;if(epoch_==0)++epoch_;
        expired_latched_=true;return false;}
    if(!active_||epoch!=epoch_||seq<=last_seq_)return false;
    last_seq_=seq;expires_ms_=now+lease_ms_;return true;
}
bool V10DriveSession::end(std::uint32_t epoch,std::uint32_t seq){
    (void)seq;
    if(!active_||epoch!=epoch_)return false;
    invalidate();return true;
}
bool V10DriveSession::expire(std::uint32_t now){
    if(!active_||static_cast<std::int32_t>(now-expires_ms_)<0)return false;
    active_=false;last_seq_=0;expires_ms_=0;++epoch_;if(epoch_==0)++epoch_;
    expired_latched_=true;return true;
}
bool V10DriveSession::consumeExpired(){const bool result=expired_latched_;expired_latched_=false;return result;}
void V10DriveSession::invalidate(){active_=false;last_seq_=0;expires_ms_=0;expired_latched_=false;
    ++epoch_;if(epoch_==0)++epoch_;}
std::uint32_t V10DriveSession::remainingMs(std::uint32_t now)const{
    if(!active_||static_cast<std::int32_t>(now-expires_ms_)>=0)return 0;
    return expires_ms_-now;
}

bool V10MovingCg::configValid()const{
    return finite(config_.height_mm)&&config_.height_mm>=50&&config_.height_mm<=150&&
      finite(config_.angle_gain)&&config_.angle_gain>=0&&config_.angle_gain<=2&&
      finite(config_.rate_gain_mm_per_rad_s)&&config_.rate_gain_mm_per_rad_s>=0&&config_.rate_gain_mm_per_rad_s<=10&&
      finite(config_.filter_tau_s)&&config_.filter_tau_s>=0&&config_.filter_tau_s<=1&&
      finite(config_.max_abs_shift_mm)&&config_.max_abs_shift_mm>0&&config_.max_abs_shift_mm<=2&&
      finite(config_.max_slew_mm_s)&&config_.max_slew_mm_s>0&&config_.max_slew_mm_s<=10&&
      finite(config_.min_dt_s)&&finite(config_.max_dt_s)&&config_.min_dt_s>0&&
      config_.max_dt_s>=config_.min_dt_s&&config_.max_dt_s<=0.1f;
}
void V10MovingCg::clear(){reference_deg_=previous_deg_=filtered_target_mm_=shift_mm_=last_rate_deg_s_=0;
    previous_timestamp_ms_=attitude_reference_ms_=0;reference_valid_=false;}
void V10MovingCg::start(float reference,float measured,std::uint32_t timestamp,std::uint32_t attitude_ref){
    clear();if(!configValid()||!finite(reference)||!finite(measured)||timestamp==0||attitude_ref==0)return;
    reference_deg_=reference;previous_deg_=measured;previous_timestamp_ms_=timestamp;
    attitude_reference_ms_=attitude_ref;reference_valid_=true;
}
bool V10MovingCg::rebase(float measured,std::uint32_t timestamp,std::uint32_t attitude_ref){
    if(!reference_valid_||!finite(measured)||timestamp==0||attitude_ref!=attitude_reference_ms_)return false;
    previous_deg_=measured;previous_timestamp_ms_=timestamp;last_rate_deg_s_=0;return true;
}
bool V10MovingCg::update(float measured,std::uint32_t timestamp,std::uint32_t attitude_ref,float& shift){
    if(!reference_valid_||!finite(measured))return false;
    const std::uint32_t elapsed=timestamp-previous_timestamp_ms_;
    if(elapsed==0)return false;
    const float dt=elapsed*0.001f;
    if(!finite(dt)||dt<config_.min_dt_s||dt>config_.max_dt_s||attitude_ref!=attitude_reference_ms_){
        clear();shift=0;return false;}
    const float rate_deg_s=(measured-previous_deg_)/dt;
    constexpr float rad=0.01745329251994329577f;
    const float error_rad=(measured-reference_deg_)*rad;
    const float raw=config_.height_mm*std::tan(error_rad*config_.angle_gain)+
        rate_deg_s*rad*config_.rate_gain_mm_per_rad_s;
    if(!finite(raw)){clear();shift=0;return false;}
    const float alpha=config_.filter_tau_s<=0?1.0f:dt/(config_.filter_tau_s+dt);
    filtered_target_mm_+=alpha*(clamp(raw,-config_.max_abs_shift_mm,config_.max_abs_shift_mm)-filtered_target_mm_);
    const float max_change=config_.max_slew_mm_s*dt;
    shift_mm_+=clamp(filtered_target_mm_-shift_mm_,-max_change,max_change);
    shift_mm_=clamp(shift_mm_,-config_.max_abs_shift_mm,config_.max_abs_shift_mm);
    previous_deg_=measured;previous_timestamp_ms_=timestamp;last_rate_deg_s_=rate_deg_s;
    shift=shift_mm_;return true;
}

bool V10CgPlanner::configValid()const{return finite(config_.body_slew_mm_s)&&config_.body_slew_mm_s>0&&
    config_.body_slew_mm_s<=30&&finite(config_.target_tolerance_mm)&&config_.target_tolerance_mm>=0&&
    config_.target_tolerance_mm<=1&&config_.wait_timeout_ms>0;}
void V10CgPlanner::begin(float base,float front,float rear){clear();
    if(!configValid()||!finite(base)||!finite(front)||!finite(rear))return;
    body_x_mm_=base;active_=true;}
void V10CgPlanner::clear(){body_x_mm_=stage_time_s_=actual_elapsed_s_=phase_=0;wait_elapsed_ms_=0;
    stage_=0;cycle_count_=0;active_=false;}
bool V10CgPlanner::transfer(float target,float dt){
    const float delta=target-body_x_mm_,step=config_.body_slew_mm_s*dt;
    if(std::fabs(delta)<=config_.target_tolerance_mm){body_x_mm_=target;wait_elapsed_ms_=0;return true;}
    body_x_mm_+=clamp(delta,-step,step);wait_elapsed_ms_+=static_cast<std::uint32_t>(dt*1000.0f+0.5f);return false;
}
void V10CgPlanner::output(float period,float target,V10CgPlanOutput& out)const{
    out.phase=phase_;out.body_x_mm=body_x_mm_;out.target_body_x_mm=target;
    out.nominal_elapsed_s=phase_*period;out.actual_elapsed_s=actual_elapsed_s_;
    out.waiting=stage_==0||stage_==4||stage_==8;out.stage=stage_;
    out.cycle_count=cycle_count_;
}
bool V10CgPlanner::tick(float dt,float period,float base,float front,float rear,V10CgPlanOutput& out){
    if(!active_||!configValid()||!finite(dt)||!finite(period)||dt<=0||dt>0.1f||period<=0||
       !finite(base)||!finite(front)||!finite(rear))return false;
    actual_elapsed_s_+=dt;
    const float starts[10]={0.0f,0.10f,0.25f,0.275f,0.425f,0.525f,0.675f,0.70f,0.85f,0.85f};
    const float ends[10]  ={0.0f,0.25f,0.275f,0.425f,0.425f,0.675f,0.70f,0.85f,0.85f,1.0f};
    float target=stage_<4?front:(stage_<8?rear:base);
    if(stage_==0||stage_==4||stage_==8){
        if(transfer(target,dt)){++stage_;stage_time_s_=0;phase_=starts[stage_];}
        else if(wait_elapsed_ms_>config_.wait_timeout_ms){output(period,target,out);return false;}
    }else{
        const float duration=(ends[stage_]-starts[stage_])*period;
        stage_time_s_+=dt;
        const float t=duration>0?clamp(stage_time_s_/duration,0.0f,1.0f):1.0f;
        phase_=starts[stage_]+(ends[stage_]-starts[stage_])*t;
        if(t>=1.0f){++stage_;stage_time_s_=0;if(stage_>=10){stage_=0;phase_=0.0f;++cycle_count_;} }
    }
    output(period,target,out);return true;
}

} // namespace dog
