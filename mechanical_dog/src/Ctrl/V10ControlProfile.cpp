#include "Ctrl/V10ControlProfile.h"

#include <cmath>

namespace dog {
namespace {bool finite(float v){return std::isfinite(v);}}

bool validControlProfile(const V10ControlProfile& p){
    return finite(p.body.pitch_deg)&&p.body.pitch_deg>=kBodyPitchMinDeg&&
      p.body.pitch_deg<=kBodyPitchMaxDeg&&
      finite(p.body.roll_deg)&&p.body.roll_deg>=kBodyRollMinDeg&&
      p.body.roll_deg<=kBodyRollMaxDeg&&
      finite(p.body.body_x_mm)&&std::fabs(p.body.body_x_mm)<=8&&
      finite(p.cg_shift_mm)&&p.cg_shift_mm>=0&&p.cg_shift_mm<=3&&
      finite(p.planner.body_slew_mm_s)&&p.planner.body_slew_mm_s>=5&&p.planner.body_slew_mm_s<=30&&
      finite(p.planner.target_tolerance_mm)&&p.planner.target_tolerance_mm>=0.01f&&p.planner.target_tolerance_mm<=0.5f&&
      p.planner.wait_timeout_ms>=1000&&p.planner.wait_timeout_ms<=30000&&
      finite(p.moving.height_mm)&&p.moving.height_mm>=100&&p.moving.height_mm<=115&&
      finite(p.moving.angle_gain)&&p.moving.angle_gain>=0&&p.moving.angle_gain<=2&&
      finite(p.moving.rate_gain_mm_per_rad_s)&&p.moving.rate_gain_mm_per_rad_s>=0&&p.moving.rate_gain_mm_per_rad_s<=10&&
      finite(p.moving.filter_tau_s)&&p.moving.filter_tau_s>=0&&p.moving.filter_tau_s<=1&&
      finite(p.moving.max_abs_shift_mm)&&p.moving.max_abs_shift_mm>0&&p.moving.max_abs_shift_mm<=0.5f&&
      finite(p.moving.max_slew_mm_s)&&p.moving.max_slew_mm_s>0&&p.moving.max_slew_mm_s<=2&&
      finite(p.moving.min_dt_s)&&finite(p.moving.max_dt_s)&&p.moving.min_dt_s==0.005f&&p.moving.max_dt_s==0.05f&&
      finite(p.pitch_kp)&&p.pitch_kp>=0&&p.pitch_kp<=2&&
      finite(p.pitch_kd_s)&&p.pitch_kd_s>=0&&p.pitch_kd_s<=1&&
      finite(p.roll_kp)&&p.roll_kp>=0&&p.roll_kp<=2&&
      finite(p.roll_kd_s)&&p.roll_kd_s>=0&&p.roll_kd_s<=1&&
      finite(p.trot_feedforward.forward_gain)&&p.trot_feedforward.forward_gain>=0&&p.trot_feedforward.forward_gain<=5&&
      finite(p.trot_feedforward.backward_gain)&&p.trot_feedforward.backward_gain>=0&&p.trot_feedforward.backward_gain<=5&&
      finite(p.trot_feedforward.turn_gain)&&p.trot_feedforward.turn_gain>=0&&p.trot_feedforward.turn_gain<=5&&
      finite(p.trot_feedforward.max_abs_mm)&&p.trot_feedforward.max_abs_mm>=0&&p.trot_feedforward.max_abs_mm<=3;
}

bool sameControlProfile(const V10ControlProfile& a,const V10ControlProfile& b,float e){
    const float av[]={a.body.pitch_deg,a.body.roll_deg,a.body.body_x_mm,a.cg_shift_mm,
      a.planner.body_slew_mm_s,a.planner.target_tolerance_mm,static_cast<float>(a.planner.wait_timeout_ms),
      a.moving.height_mm,a.moving.angle_gain,a.moving.rate_gain_mm_per_rad_s,a.moving.filter_tau_s,
      a.moving.max_abs_shift_mm,a.moving.max_slew_mm_s,a.moving.min_dt_s,a.moving.max_dt_s,
      a.pitch_kp,a.pitch_kd_s,a.roll_kp,a.roll_kd_s,a.trot_feedforward.forward_gain,
      a.trot_feedforward.backward_gain,a.trot_feedforward.turn_gain,a.trot_feedforward.max_abs_mm};
    const float bv[]={b.body.pitch_deg,b.body.roll_deg,b.body.body_x_mm,b.cg_shift_mm,
      b.planner.body_slew_mm_s,b.planner.target_tolerance_mm,static_cast<float>(b.planner.wait_timeout_ms),
      b.moving.height_mm,b.moving.angle_gain,b.moving.rate_gain_mm_per_rad_s,b.moving.filter_tau_s,
      b.moving.max_abs_shift_mm,b.moving.max_slew_mm_s,b.moving.min_dt_s,b.moving.max_dt_s,
      b.pitch_kp,b.pitch_kd_s,b.roll_kp,b.roll_kd_s,b.trot_feedforward.forward_gain,
      b.trot_feedforward.backward_gain,b.trot_feedforward.turn_gain,b.trot_feedforward.max_abs_mm};
    if(a.cg_walk_enabled!=b.cg_walk_enabled||a.trot_feedforward.enabled!=b.trot_feedforward.enabled)return false;
    for(unsigned i=0;i<sizeof(av)/sizeof(av[0]);++i)if(std::fabs(av[i]-bv[i])>e)return false;
    return true;
}
} // namespace dog
