#pragma once

#include <cstdint>
#include "Ctrl/V10BodyPose.h"
#include "Ctrl/V10DriveControl.h"

namespace dog {

struct V10ControlProfile {
    V10ControlProfile() { body.body_x_mm=8.0f; }
    V10BodyPoseCommand body{};
    bool cg_walk_enabled=false;
    float cg_shift_mm=2.5f;
    V10CgPlannerConfig planner{20.0f,0.05f,15000};
    V10MovingCgConfig moving{110.0f,1.1f,5.0f,0.10f,0.5f,2.0f,0.005f,0.05f};
    float pitch_kp=0.8f;
    float pitch_kd_s=0.08f;
    float roll_kp=0.8f;
    float roll_kd_s=0.08f;
    V10TrotFeedforwardConfig trot_feedforward{};
};

bool validControlProfile(const V10ControlProfile& profile);
bool sameControlProfile(const V10ControlProfile& a,const V10ControlProfile& b,
                        float epsilon=0.0001f);

} // namespace dog
