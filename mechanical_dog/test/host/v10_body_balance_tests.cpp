#include <cassert>
#include <cmath>
#include "Ctrl/V10BodyPose.h"
#include "Ctrl/V10StaticBalance.h"

namespace {
bool close(float a,float b,float e=0.001f){return std::fabs(a-b)<e;}
}

int main(){
    using namespace dog;
    const float forward[4]={1,2,3,4};
    const float down[4]={108,107,106,105};
    V10BodyPose pose(142.0f,92.8f);
    V10BodyPoseCommand command{};
    float f[4]={},d[4]={};
    assert(pose.apply(forward,down,command,f,d));
    for(int i=0;i<4;++i){assert(close(f[i],forward[i]));assert(close(d[i],down[i]));}

    command.body_x_mm=5;
    assert(pose.apply(forward,down,command,f,d));
    for(int i=0;i<4;++i) assert(close(f[i],forward[i]-5));

    command={}; command.body_x_mm=8;
    assert(pose.apply(forward,down,command,f,d));
    for(int i=0;i<4;++i) {
        assert(close(f[i],forward[i]-8));
        assert(close(d[i],down[i]));
    }

    command={}; command.pitch_deg=3;
    assert(pose.apply(forward,down,command,f,d));
    assert(d[0]>down[0] && d[1]>down[1]);
    assert(d[2]<down[2] && d[3]<down[3]);

    command={}; command.roll_deg=3;
    assert(pose.apply(forward,down,command,f,d));
    assert(d[0]>down[0] && d[3]>down[3]); // legs 1/4 are left
    assert(d[1]<down[1] && d[2]<down[2]);

    command={};command.pitch_deg=kBodyPitchMinDeg;assert(pose.apply(forward,down,command,f,d));
    command.pitch_deg=kBodyPitchMinDeg-0.1f;assert(!pose.apply(forward,down,command,f,d));
    command={};command.pitch_deg=kBodyPitchMaxDeg;assert(pose.apply(forward,down,command,f,d));
    command.pitch_deg=kBodyPitchMaxDeg+0.1f;assert(!pose.apply(forward,down,command,f,d));
    command={};command.roll_deg=kBodyRollMinDeg;assert(pose.apply(forward,down,command,f,d));
    command.roll_deg=kBodyRollMinDeg-0.1f;assert(!pose.apply(forward,down,command,f,d));
    command={};command.roll_deg=kBodyRollMaxDeg;assert(pose.apply(forward,down,command,f,d));
    command.roll_deg=kBodyRollMaxDeg+0.1f;assert(!pose.apply(forward,down,command,f,d));

    V10StaticBalance balance({0.8f,0.08f,1.5f,8.0f,0.005f,0.05f});
    balance.setReference(0.0f,1000);
    float correction=0;
    assert(!balance.update(0.0f,1000,1000,correction)); // duplicate/reference sample
    assert(balance.update(2.0f,1020,1000,correction));
    assert(correction<0); // nose-up measurement commands nose-down correction
    const float first=correction;
    assert(balance.update(2.0f,1040,1000,correction));
    assert(correction<=first && correction>=-1.5f);
    assert(!balance.update(2.0f,1200,1000,correction)); // excessive dt clears output
    assert(close(balance.correctionDeg(),0));

    // User-observed mounting: raising the nose makes sensor Pitch negative.
    assert(close(V10StaticBalance::bodyPitchFromSensor(-2.0f),2.0f));
    assert(close(V10StaticBalance::bodyPitchRateFromSensor(-0.5f),0.5f));

    // A captured reference can sit idle for seconds. Enabling rebases the
    // derivative timestamp/measurement without discarding that reference.
    balance.start(0.0f,0.0f,6000,1000);
    assert(balance.referenceValid());
    assert(balance.update(1.0f,6020,1000,correction));
    assert(correction<0.0f);
}
