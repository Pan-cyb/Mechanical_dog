#pragma once

#include <cstdint>

namespace dog {

// Confirmed installation mapping: when the chassis left side is lower and
// the right side is higher, sensor Roll is positive. Body Roll is defined
// positive for the left side higher, therefore body_roll = -sensor_roll.
constexpr std::int8_t kDefaultRollSensorSign = -1;

struct V10StaticBalanceConfig {
    float kp;
    float kd_s;
    float max_correction_deg;
    float max_slew_deg_s;
    float min_dt_s;
    float max_dt_s;
};

// Bounded PD on the bias-corrected attitude estimate.  It intentionally does
// not translate the original per-call accumulating gains: dt is explicit,
// correction and slew are bounded, and no integral state can wind up.
class V10StaticBalance {
public:
    explicit V10StaticBalance(const V10StaticBalanceConfig& config)
        : config_(config) {}
    void clear();
    void setReference(float body_pitch_deg, std::uint32_t timestamp_ms,
                      std::uint32_t attitude_reference_ms = 0);
    void start(float reference_body_pitch_deg,float measured_body_pitch_deg,
               std::uint32_t timestamp_ms,std::uint32_t attitude_reference_ms);
    bool update(float measured_body_pitch_deg, std::uint32_t timestamp_ms,
                std::uint32_t attitude_reference_ms, float& correction_deg);
    bool referenceValid() const { return reference_valid_; }
    float referenceDeg() const { return reference_deg_; }
    float correctionDeg() const { return correction_deg_; }
    float lastRateDegS() const { return last_rate_deg_s_; }

    // Confirmed installation mapping from the user's lift-nose observation.
    static float bodyPitchFromSensor(float sensor_pitch_deg) { return -sensor_pitch_deg; }
    static float bodyPitchRateFromSensor(float sensor_pitch_rate_deg_s) {
        return -sensor_pitch_rate_deg_s;
    }

private:
    bool configValid() const;
    V10StaticBalanceConfig config_;
    float reference_deg_ = 0.0f;
    float previous_measurement_deg_ = 0.0f;
    float correction_deg_ = 0.0f;
    float last_rate_deg_s_ = 0.0f;
    std::uint32_t previous_timestamp_ms_ = 0;
    std::uint32_t attitude_reference_ms_ = 0;
    bool reference_valid_ = false;
};

} // namespace dog
