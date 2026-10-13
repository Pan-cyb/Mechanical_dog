#pragma once

#include <cstdint>
#include "Ctrl/V10Gait.h"

namespace dog {

enum class CommandType {
    Invalid, Help, Status, Imu, Attitude, Arm, CalStart, CalSaveAll, Off, OffChannel,
    KeepAlive, Pwm, Offset, Shank, Nudge, Save, Center, RangeNormal, RangeExtended,
    Stand, Walk, Trot, LegTest, MotionStop, MotionSet, MotionPreview,
    ArmStand, MotionSave, MotionLoad, BodySet, BalanceCapture, BalanceOn,
    BalanceOff, BalanceRollSign, DriveBegin, DriveRenew, DriveEnd,
    MovingCgOn, MovingCgOff, ControlSet, ProfileSave, ProfileLoad,
    TrotFeedforwardSet, StraightTrimSet, TrotPitchSet, PoseLiveSet
};

struct Command {
    CommandType type = CommandType::Invalid;
    std::uint8_t channel = 0;
    std::uint16_t pulse_us = 0;
    std::int8_t delta_deg = 0;
    float offset_deg = 0;
    float shank_q_deg = 0;
    std::uint8_t leg = 0;
    float motion_values[6] = {};
    float body_values[5] = {};
    std::int8_t axis_sign = 0;
    std::uint32_t epoch = 0;
    std::uint32_t seq = 0;
    V10DriveIntent drive_intent{};
    MotionMode drive_mode = MotionMode::Idle;
    float control_values[11] = {};
    float trot_ff_values[5] = {};
    float straight_trim = 0.0f;
    float trot_pitch_offset_deg = 0.0f;
    float pose_live_values[4] = {};
    MotionMode motion_mode = MotionMode::Idle;
};

// Strict, bounded command grammar. Never execute received text as code.
Command parseCommand(const char* line);
// Strict signed decimal (no NaN/Inf/exponents or trailing text), +/-180 max.
bool parseServoOffset(const char* text, float& value);
// Strict non-negative decimal q magnitude (no NaN/Inf/exponents/trailing text).
bool parseShankAngle(const char* text, float& value);

} // namespace dog
