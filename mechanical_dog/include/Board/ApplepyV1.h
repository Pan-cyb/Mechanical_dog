#pragma once

#include <cstddef>
#include <cstdint>
#include "Ctrl/RelativeServoMap.h"
#include "Ctrl/V10ShankLinearMap.h"
#include "Ctrl/V10Gait.h"

namespace dog::board {

// SCH_Schematic1_2026-09-24.pdf, sheet 1. Both devices share this bus.
constexpr int kI2cSdaPin = 4;
constexpr int kI2cSclPin = 5;
constexpr std::uint32_t kI2cFrequencyHz = 100000;
constexpr std::uint8_t kPca9685Address = 0x40;
constexpr std::uint8_t kHxyImuAddress = 0x19; // AP_AD0 pulled up by R28.
constexpr std::uint16_t kServoFrequencyHz = 50;

// User-selected silk-screen outputs Servo1..4, Servo13, and Servo10..12,
// numbered one-based. The PCA9685 and raw `pwm` command use zero-based channels.
constexpr std::uint8_t kSelectedServoChannels[8] = {0, 1, 2, 3, 12, 9, 10, 11};
// Active V10 open-loop joint wiring used by the coordinated gait path. The
// legacy name remains source-compatible. Mapping and directions match the
// user's bench observations; the gait itself is not hardware-validated.
// Order: L1H,L1S,L2H,L2S,L3H,L3S,L4H,L4S.
// Silkscreen ServoN uses N = PCA zero-based channel + 1.
constexpr std::uint8_t kProposedJointChannels[8] = {0, 1, 2, 3, 12, 9, 10, 11};

// Read fallback for the user's L3H move from Servo9/CH8 to Servo13/CH12.
// A real c12 key always wins, even when its value is invalid.
// Explicit saves use c12 only; c8 is never erased or rewritten by the alias.
constexpr std::uint8_t kCalibrationFallbackChannel = 12;
constexpr std::uint8_t kCalibrationLegacyChannel = 8;

// User's new neutral convention and observed shank command directions.
// Legs 1/4 = left front/rear, legs 2/3 = right front/rear.
// Hip ranges below are electrical only; do not apply shank +/-50 to hips.
constexpr RelativeServoLimit kRelativeServoLimits[8] = {
    {0, "L1H", -180, 180, false}, {1, "L1S", -50, 0, true},
    {2, "L2H", -180, 180, false}, {3, "L2S", 0, 50, true},
    {12, "L3H", -180, 180, false}, {9, "L3S", 0, 50, true},
    {10, "L4H", -180, 180, false}, {11, "L4S", -50, 0, true},
};

// The curve was measured from leg 4 left-rear photos. The user confirmed all
// four legs share geometry, assembly and mechanical-neutral convention, so one
// candidate table is explicitly bound to each shank channel. Direction remains
// owned by kRelativeServoLimits; the node data itself is unsigned and shared.
constexpr V10ShankMapPoint kSharedShankPhotoMapPoints[3] = {
    {0.0f, 0.0f}, {30.0f, 9.0f}, {50.0f, 23.0f}
};
constexpr V10ShankMapConfig kShankMapConfigs[4] = {
    {1, kSharedShankPhotoMapPoints, 3},
    {3, kSharedShankPhotoMapPoints, 3},
    {9, kSharedShankPhotoMapPoints, 3},
    {11, kSharedShankPhotoMapPoints, 3}
};
constexpr std::size_t kShankMapConfigCount =
    sizeof(kShankMapConfigs) / sizeof(kShankMapConfigs[0]);

// Complete V10 open-loop mapping. Geometry neutral is deliberately distinct
// from each channel's persisted calibration centre.
constexpr V10MotionConfig kV10MotionConfig = {
    {80.0f, 61.357f}, 90.0f, 90.0f, 45.0f, 1.5f, 20, 200,
    {0,1,2,3,12,9,10,11},
    {-1,1,1,-1}, {-1,1,1,-1},
    kSharedShankPhotoMapPoints, 3,
    142.0f, 92.8f // Original nominal reference; this chassis is not measured yet.
};

// User-requested full calibration input: no hidden 45..135 deg restriction.
// Electrical envelope ONLY. All movement still requires explicit ARM/output.
constexpr std::uint16_t kBenchMinPulseUs = 500;
constexpr std::uint16_t kBenchMaxPulseUs = 2500;
// Legacy 0.5.1 range interface stays compatible; default is now identical.
// This is the driver's electrical envelope, NOT a certified MG90 travel range.
constexpr std::uint16_t kExtendedMinPulseUs = 500;
constexpr std::uint16_t kExtendedMaxPulseUs = 2500;
// User-requested manual bench hold: ARM and active PWM remain until OFF or a
// detected fault. No software timeout; physical servo-power cutoff is required.
constexpr std::uint32_t kCommandTimeoutMs = 0;

// Do NOT use GPIO6 as an IMU interrupt: the sheet labels it INT, but the
// IMU INT1 pin is not visibly connected. OE# is pulled low, not MCU-controlled.
} // namespace dog::board
