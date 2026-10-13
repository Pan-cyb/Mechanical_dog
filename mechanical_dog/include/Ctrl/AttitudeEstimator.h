#pragma once

#include <cstdint>
#include "Interface/IImu.h"

namespace dog {

struct AttitudeSample {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    std::uint32_t timestamp_ms = 0;
    std::uint32_t yaw_reference_ms = 0; // First sample after calibration or a sampling gap.
};

// Six-axis attitude for display and the opt-in experimental static controller;
// it is not an absolute heading, joint feedback, or safety interlock.
// Angles are ZYX Euler angles in the IMU's native sensor frame, with no board
// mounting transform: +X roll, +Y pitch, +Z yaw by the right-hand rule. With
// the sensor level and +Z accelerometer specific force, roll/pitch are zero.
// Initial roll/pitch come from gravity; yaw starts at zero and drifts because
// this sensor supplies no magnetometer/absolute heading observation.
// Before the first output, keep the board still for 100 stationary samples
// spaced at least 5 ms apart (nominal 10 ms). Occasional slow HTTP requests
// do not erase already collected samples. The averaged static gyro is removed
// from later readings. This is nonblocking: update() returns false until ready.
class AttitudeEstimator {
public:
    enum : std::uint16_t { kCalibrationSampleCount = 100 };

    // Returns false without modifying out or the previous estimate for an
    // invalid/duplicate sample. During startup, invalid/moving samples reset
    // the calibration run. A gap >100 ms discards orientation history and
    // starts again from gravity (including a new zero-relative yaw), but keeps
    // the calibrated gyro bias. reset() clears the bias as well.
    bool update(const ImuSample& sample, AttitudeSample& out);
    void reset();
    bool valid() const { return initialized_; }
    bool calibrated() const { return calibrated_; }
    std::uint16_t calibrationSamples() const { return calibration_count_; }

private:
    bool initialize(const ImuSample& sample, AttitudeSample& out);
    void resetOrientation();
    void resetCalibrationRun();

    double qw_ = 1.0;
    double qx_ = 0.0;
    double qy_ = 0.0;
    double qz_ = 0.0;
    double gyro_bias_[3] = {};
    double gyro_sum_[3] = {};
    double accel_norm_sum_ = 0.0;
    double gravity_reference_norm_ = 0.0; // Measured at rest; raw scale may differ.
    std::uint32_t last_timestamp_ms_ = 0;
    std::uint32_t yaw_reference_ms_ = 0;
    std::uint32_t last_calibration_ms_ = 0;
    std::uint16_t calibration_count_ = 0;
    bool calibrated_ = false;
    bool initialized_ = false;
};

} // namespace dog
