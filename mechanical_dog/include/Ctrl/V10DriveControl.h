#pragma once

#include <cstdint>

namespace dog {

inline constexpr float kDefaultStraightTrim = 0.25f;

struct V10DriveIntent {
    float forward = 1.0f; // -1 reverse .. +1 forward
    float turn = 0.0f;    // +1 left turn: left side slower/reverse
    bool stepping = false; // lift in place, zero fore/aft amplitude
};

bool driveScales(const V10DriveIntent& intent,float& left,float& right);
// RAM-only straight-line trim. Positive trim weakens the physical right side
// for both forward and reverse travel; negative trim weakens the left side.
// Explicit turns and stepping are returned unchanged.
bool applyStraightTrim(const V10DriveIntent& intent,float trim,
                       V10DriveIntent& effective);

struct V10TrotFeedforwardConfig {
    bool enabled=false;
    float forward_gain=4.0f;
    float backward_gain=4.0f;
    float turn_gain=2.0f;
    float max_abs_mm=2.0f;
};
bool trotFeedforward(float step_length_mm,const V10DriveIntent& intent,
                     const V10TrotFeedforwardConfig& config,float& offset_mm);

class V10DriveSession {
public:
    explicit V10DriveSession(std::uint32_t lease_ms) : lease_ms_(lease_ms) {}
    bool canBegin(std::uint32_t expected_epoch,std::uint32_t seq) const;
    bool begin(std::uint32_t expected_epoch,std::uint32_t seq,std::uint32_t now_ms);
    bool renew(std::uint32_t epoch,std::uint32_t seq,std::uint32_t now_ms);
    bool end(std::uint32_t epoch,std::uint32_t seq);
    bool expire(std::uint32_t now_ms);
    bool consumeExpired();
    void invalidate();
    bool active() const { return active_; }
    std::uint32_t epoch() const { return epoch_; }
    std::uint32_t lastSeq() const { return last_seq_; }
    std::uint32_t remainingMs(std::uint32_t now_ms) const;
private:
    std::uint32_t lease_ms_;
    std::uint32_t epoch_=1;
    std::uint32_t last_seq_=0;
    std::uint32_t expires_ms_=0;
    bool active_=false;
    bool expired_latched_=false;
};

struct V10MovingCgConfig {
    float height_mm;
    float angle_gain;
    float rate_gain_mm_per_rad_s;
    float filter_tau_s;
    float max_abs_shift_mm;
    float max_slew_mm_s;
    float min_dt_s;
    float max_dt_s;
};

class V10MovingCg {
public:
    explicit V10MovingCg(const V10MovingCgConfig& config):config_(config){}
    void clear();
    void start(float reference_body_pitch_deg,float measured_body_pitch_deg,
               std::uint32_t timestamp_ms,std::uint32_t attitude_reference_ms);
    bool rebase(float measured_body_pitch_deg,std::uint32_t timestamp_ms,
                std::uint32_t attitude_reference_ms);
    bool update(float measured_body_pitch_deg,std::uint32_t timestamp_ms,
                std::uint32_t attitude_reference_ms,float& shift_mm);
    bool referenceValid() const{return reference_valid_;}
    float shiftMm() const{return shift_mm_;}
    float rateDegS() const{return last_rate_deg_s_;}
    const V10MovingCgConfig& config() const{return config_;}
private:
    bool configValid() const;
    V10MovingCgConfig config_;
    float reference_deg_=0;
    float previous_deg_=0;
    float filtered_target_mm_=0;
    float shift_mm_=0;
    float last_rate_deg_s_=0;
    std::uint32_t previous_timestamp_ms_=0;
    std::uint32_t attitude_reference_ms_=0;
    bool reference_valid_=false;
};

struct V10CgPlannerConfig {
    float body_slew_mm_s;
    float target_tolerance_mm;
    std::uint32_t wait_timeout_ms;
};

struct V10CgPlanOutput {
    float phase=0;
    float body_x_mm=0;
    float target_body_x_mm=0;
    float nominal_elapsed_s=0;
    float actual_elapsed_s=0;
    bool waiting=true;
    std::uint8_t stage=0;
    std::uint32_t cycle_count=0;
};

class V10CgPlanner {
public:
    explicit V10CgPlanner(const V10CgPlannerConfig& config):config_(config){}
    void begin(float base_body_x_mm,float front_target_mm,float rear_target_mm);
    bool tick(float dt_s,float period_s,float base_body_x_mm,
              float front_target_mm,float rear_target_mm,V10CgPlanOutput& out);
    void clear();
    bool active() const{return active_;}
    const V10CgPlannerConfig& config() const{return config_;}
private:
    bool configValid() const;
    bool transfer(float target,float dt_s);
    void output(float period_s,float target,V10CgPlanOutput& out) const;
    V10CgPlannerConfig config_;
    float body_x_mm_=0;
    float stage_time_s_=0;
    float actual_elapsed_s_=0;
    float phase_=0;
    std::uint32_t wait_elapsed_ms_=0;
    std::uint8_t stage_=0;
    std::uint32_t cycle_count_=0;
    bool active_=false;
};

} // namespace dog
