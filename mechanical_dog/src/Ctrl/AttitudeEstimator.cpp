#include "Ctrl/AttitudeEstimator.h"

#include <cmath>

namespace dog {
namespace {
constexpr double kMinPlausibleAccelNorm = 2.5;
constexpr double kMaxPlausibleAccelNorm = 20.0;
constexpr double kRadiansToDegrees = 57.29577951308232;
constexpr double kProportionalGain = 2.0; // gravity correction, 1/s
constexpr double kMaxGyroRadps = 40.0; // above the configured +/-1000 dps range
constexpr double kStaticGyroRadps = 0.05; // about 2.86 dps; keep the board still
constexpr std::uint32_t kMaxGapMs = 100;
constexpr std::uint32_t kMinCalibrationSpacingMs = 5;

double clampUnit(double value) {
    return value < -1.0 ? -1.0 : (value > 1.0 ? 1.0 : value);
}

bool usableSample(const ImuSample& sample, double& gravity_norm) {
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(sample.accel_mps2[axis]) ||
            !std::isfinite(sample.gyro_radps[axis])) return false;
    }
    const double ax = sample.accel_mps2[0];
    const double ay = sample.accel_mps2[1];
    const double az = sample.accel_mps2[2];
    gravity_norm = std::hypot(std::hypot(ax, ay), az);
    const double gyro_norm = std::hypot(
        std::hypot(static_cast<double>(sample.gyro_radps[0]),
                   static_cast<double>(sample.gyro_radps[1])),
        static_cast<double>(sample.gyro_radps[2]));
    // Free fall, hard shocks and impossible angular readings cannot provide
    // a credible attitude. Intermediate linear acceleration is accepted but
    // not mistaken for gravity by the correction below.
    return gravity_norm >= kMinPlausibleAccelNorm &&
           gravity_norm <= kMaxPlausibleAccelNorm &&
           gyro_norm <= kMaxGyroRadps;
}

bool stationaryGravity(double norm, double reference) {
    return reference > 0.0 && norm >= 0.8 * reference &&
           norm <= 1.2 * reference;
}

bool calibrationGravity(double norm, double reference) {
    return reference <= 0.0 ||
           (norm >= 0.9 * reference && norm <= 1.1 * reference);
}

AttitudeSample euler(double w, double x, double y, double z,
                     std::uint32_t timestamp_ms,
                     std::uint32_t yaw_reference_ms) {
    AttitudeSample result{};
    result.roll_deg = static_cast<float>(
        std::atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y)) * kRadiansToDegrees);
    result.pitch_deg = static_cast<float>(
        std::asin(clampUnit(2.0 * (w*y - z*x))) * kRadiansToDegrees);
    result.yaw_deg = static_cast<float>(
        std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)) * kRadiansToDegrees);
    result.timestamp_ms = timestamp_ms;
    result.yaw_reference_ms = yaw_reference_ms;
    return result;
}

bool finiteAttitude(const AttitudeSample& sample) {
    return std::isfinite(sample.roll_deg) && std::isfinite(sample.pitch_deg) &&
           std::isfinite(sample.yaw_deg);
}
} // namespace

void AttitudeEstimator::resetOrientation() {
    qw_ = 1.0;
    qx_ = qy_ = qz_ = 0.0;
    last_timestamp_ms_ = 0;
    yaw_reference_ms_ = 0;
    initialized_ = false;
}

void AttitudeEstimator::resetCalibrationRun() {
    calibration_count_ = 0;
    last_calibration_ms_ = 0;
    accel_norm_sum_ = 0.0;
    for (int axis = 0; axis < 3; ++axis) gyro_sum_[axis] = 0.0;
}

void AttitudeEstimator::reset() {
    resetOrientation();
    resetCalibrationRun();
    for (int axis = 0; axis < 3; ++axis) gyro_bias_[axis] = 0.0;
    gravity_reference_norm_ = 0.0;
    calibrated_ = false;
}

bool AttitudeEstimator::initialize(const ImuSample& sample, AttitudeSample& out) {
    double norm = 0.0;
    if (!usableSample(sample, norm) ||
        !stationaryGravity(norm, gravity_reference_norm_)) return false;
    const double ax = sample.accel_mps2[0];
    const double ay = sample.accel_mps2[1];
    const double az = sample.accel_mps2[2];
    const double roll = std::atan2(ay, az);
    const double pitch = std::atan2(-ax, std::hypot(ay, az));
    const double cr = std::cos(0.5 * roll);
    const double sr = std::sin(0.5 * roll);
    const double cp = std::cos(0.5 * pitch);
    const double sp = std::sin(0.5 * pitch);
    const double w = cr * cp;
    const double x = sr * cp;
    const double y = cr * sp;
    const double z = -sr * sp; // ZYX yaw = 0 at initialisation.
    AttitudeSample result = euler(w, x, y, z, sample.timestamp_ms,
                                  sample.timestamp_ms);
    result.yaw_deg = 0.0f; // At gimbal lock, yaw is mathematically ambiguous.
    if (!finiteAttitude(result)) return false;
    qw_ = w;
    qx_ = x;
    qy_ = y;
    qz_ = z;
    last_timestamp_ms_ = sample.timestamp_ms;
    yaw_reference_ms_ = sample.timestamp_ms;
    initialized_ = true;
    out = result;
    return true;
}

bool AttitudeEstimator::update(const ImuSample& sample, AttitudeSample& out) {
    double norm = 0.0;
    if (!usableSample(sample, norm)) {
        if (!calibrated_) resetCalibrationRun();
        return false;
    }
    if (!calibrated_) {
        const double gx = sample.gyro_radps[0];
        const double gy = sample.gyro_radps[1];
        const double gz = sample.gyro_radps[2];
        const double mean_norm = calibration_count_ == 0 ? 0.0 :
            accel_norm_sum_ / calibration_count_;
        if (!calibrationGravity(norm, mean_norm) ||
            std::hypot(std::hypot(gx, gy), gz) >= kStaticGyroRadps) {
            resetCalibrationRun();
            return false;
        }
        if (calibration_count_ != 0) {
            const std::uint32_t spacing = sample.timestamp_ms - last_calibration_ms_;
            // The HTTP server shares the main loop with IMU polling. A slow
            // request must not erase 99 valid stationary samples; use the
            // current sample only when enough time has passed.
            if (spacing < kMinCalibrationSpacingMs) return false;
        }
        for (int axis = 0; axis < 3; ++axis)
            gyro_sum_[axis] += sample.gyro_radps[axis];
        accel_norm_sum_ += norm;
        last_calibration_ms_ = sample.timestamp_ms;
        ++calibration_count_;
        if (calibration_count_ < kCalibrationSampleCount) return false;
        for (int axis = 0; axis < 3; ++axis)
            gyro_bias_[axis] = gyro_sum_[axis] / kCalibrationSampleCount;
        gravity_reference_norm_ = accel_norm_sum_ / kCalibrationSampleCount;
        calibrated_ = true;
        return initialize(sample, out);
    }
    if (!initialized_) return initialize(sample, out);

    const std::uint32_t elapsed_ms = sample.timestamp_ms - last_timestamp_ms_;
    if (elapsed_ms == 0) return false;
    if (elapsed_ms > kMaxGapMs) {
        // Also handles timestamps moving backwards (apart from true wrap).
        // Do not propagate the old gyro through an unknown interval.
        resetOrientation();
        return initialize(sample, out);
    }

    double gx = sample.gyro_radps[0] - gyro_bias_[0];
    double gy = sample.gyro_radps[1] - gyro_bias_[1];
    double gz = sample.gyro_radps[2] - gyro_bias_[2];
    if (stationaryGravity(norm, gravity_reference_norm_)) {
        const double ax = sample.accel_mps2[0] / norm;
        const double ay = sample.accel_mps2[1] / norm;
        const double az = sample.accel_mps2[2] / norm;
        // Predicted gravity/specific-force direction in sensor coordinates:
        // R(sensor -> world)^T * [0, 0, 1]. Accel x predicted is the
        // complementary correction in the same coordinates as gyro.
        const double vx = 2.0 * (qx_*qz_ - qw_*qy_);
        const double vy = 2.0 * (qy_*qz_ + qw_*qx_);
        const double vz = 1.0 - 2.0 * (qx_*qx_ + qy_*qy_);
        gx += kProportionalGain * (ay*vz - az*vy);
        gy += kProportionalGain * (az*vx - ax*vz);
        gz += kProportionalGain * (ax*vy - ay*vx);
    }

    const double dt = static_cast<double>(elapsed_ms) * 0.001;
    const double speed = std::hypot(std::hypot(gx, gy), gz);
    const double half_angle = 0.5 * speed * dt;
    const double sin_over_speed = speed > 1.0e-12
        ? std::sin(half_angle) / speed : 0.5 * dt;
    const double dw = std::cos(half_angle);
    const double dx = gx * sin_over_speed;
    const double dy = gy * sin_over_speed;
    const double dz = gz * sin_over_speed;
    double w = qw_*dw - qx_*dx - qy_*dy - qz_*dz;
    double x = qw_*dx + qx_*dw + qy_*dz - qz_*dy;
    double y = qw_*dy - qx_*dz + qy_*dw + qz_*dx;
    double z = qw_*dz + qx_*dy - qy_*dx + qz_*dw;
    const double q_norm = std::hypot(std::hypot(w, x), std::hypot(y, z));
    if (!(q_norm > 0.0) || !std::isfinite(q_norm)) return false;
    w /= q_norm;
    x /= q_norm;
    y /= q_norm;
    z /= q_norm;
    const AttitudeSample result = euler(w, x, y, z, sample.timestamp_ms,
                                         yaw_reference_ms_);
    if (!finiteAttitude(result)) return false;
    qw_ = w;
    qx_ = x;
    qy_ = y;
    qz_ = z;
    last_timestamp_ms_ = sample.timestamp_ms;
    out = result;
    return true;
}

} // namespace dog
