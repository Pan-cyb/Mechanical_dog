#include "Ctrl/V10StaticBalance.h"

#include <cmath>

namespace dog {
namespace {
float clamp(float value,float low,float high){return value<low?low:(value>high?high:value);}
}

bool V10StaticBalance::configValid() const {
    return std::isfinite(config_.kp) && config_.kp >= 0.0f &&
        std::isfinite(config_.kd_s) && config_.kd_s >= 0.0f &&
        std::isfinite(config_.max_correction_deg) && config_.max_correction_deg > 0.0f &&
        std::isfinite(config_.max_slew_deg_s) && config_.max_slew_deg_s > 0.0f &&
        std::isfinite(config_.min_dt_s) && std::isfinite(config_.max_dt_s) &&
        config_.min_dt_s > 0.0f && config_.max_dt_s >= config_.min_dt_s;
}

void V10StaticBalance::clear() {
    reference_deg_=previous_measurement_deg_=correction_deg_=last_rate_deg_s_=0.0f;
    previous_timestamp_ms_=attitude_reference_ms_=0;
    reference_valid_=false;
}

void V10StaticBalance::setReference(float body_pitch_deg,std::uint32_t timestamp_ms,
                                    std::uint32_t attitude_reference_ms) {
    start(body_pitch_deg,body_pitch_deg,timestamp_ms,
          attitude_reference_ms==0 ? timestamp_ms : attitude_reference_ms);
}

void V10StaticBalance::start(float reference,float measured,std::uint32_t timestamp_ms,
                             std::uint32_t attitude_reference_ms) {
    clear();
    if(!configValid() || !std::isfinite(reference) || !std::isfinite(measured) ||
       timestamp_ms==0 || attitude_reference_ms==0) return;
    reference_deg_=reference;previous_measurement_deg_=measured;
    previous_timestamp_ms_=timestamp_ms;
    attitude_reference_ms_=attitude_reference_ms;
    reference_valid_=true;
}

bool V10StaticBalance::update(float measured,std::uint32_t timestamp_ms,
                              std::uint32_t attitude_reference_ms,float& correction) {
    if(!reference_valid_ || !std::isfinite(measured)) return false;
    const std::uint32_t elapsed_ms=timestamp_ms-previous_timestamp_ms_;
    if(elapsed_ms==0) return false;
    const float dt=elapsed_ms*0.001f;
    if(!std::isfinite(dt) || dt<config_.min_dt_s || dt>config_.max_dt_s ||
       attitude_reference_ms!=attitude_reference_ms_) {
        clear(); correction=0.0f; return false;
    }
    const float rate=(measured-previous_measurement_deg_)/dt;
    const float error=reference_deg_-measured;
    const float target=clamp(config_.kp*error-config_.kd_s*rate,
                             -config_.max_correction_deg,config_.max_correction_deg);
    const float max_change=config_.max_slew_deg_s*dt;
    correction_deg_ += clamp(target-correction_deg_,-max_change,max_change);
    correction_deg_=clamp(correction_deg_,-config_.max_correction_deg,
                          config_.max_correction_deg);
    previous_measurement_deg_=measured;
    previous_timestamp_ms_=timestamp_ms;
    last_rate_deg_s_=rate;
    correction=correction_deg_;
    return true;
}

} // namespace dog
