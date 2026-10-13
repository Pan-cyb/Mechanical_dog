#include <cassert>
#include <cmath>
#include "Ctrl/V10DriveControl.h"

namespace { bool close(float a,float b,float e=0.001f){return std::fabs(a-b)<e;} }

int main(){
    using namespace dog;
    static_assert(kDefaultStraightTrim==0.25f);
    const V10DriveIntent raw_default{};
    assert(close(raw_default.forward,1)&&close(raw_default.turn,0)&&!raw_default.stepping);
    V10DriveIntent intent{1.0f,0.0f,false};
    float left=0,right=0;
    assert(driveScales(intent,left,right)&&close(left,1)&&close(right,1));
    V10DriveIntent effective{};
    assert(applyStraightTrim({1,0,false},0.0f,effective)&&
           effective.forward==1&&close(effective.turn,0));
    assert(applyStraightTrim({1,0,false},0.05f,effective)&&
           driveScales(effective,left,right)&&close(left,1)&&close(right,0.95f));
    assert(applyStraightTrim({-1,0,false},0.05f,effective)&&
           driveScales(effective,left,right)&&close(left,-1)&&close(right,-0.95f));
    assert(applyStraightTrim({1,0,false},kDefaultStraightTrim,effective)&&
           driveScales(effective,left,right)&&close(left,1)&&close(right,0.75f));
    assert(applyStraightTrim({-1,0,false},kDefaultStraightTrim,effective)&&
           driveScales(effective,left,right)&&close(left,-1)&&close(right,-0.75f));
    assert(applyStraightTrim({1,0,false},-0.05f,effective)&&
           driveScales(effective,left,right)&&close(left,0.95f)&&close(right,1));
    assert(applyStraightTrim({-1,0,false},-0.05f,effective)&&
           driveScales(effective,left,right)&&close(left,-0.95f)&&close(right,-1));
    assert(applyStraightTrim({0,1,false},0.20f,effective)&&close(effective.turn,1));
    assert(applyStraightTrim({0,-1,false},-0.20f,effective)&&close(effective.turn,-1));
    assert(applyStraightTrim({0,0,true},0.20f,effective)&&effective.stepping&&close(effective.turn,0));
    assert(!applyStraightTrim({1,0,false},0.301f,effective));
    assert(!applyStraightTrim({1,0,false},NAN,effective));
    intent={-1,0,false};assert(driveScales(intent,left,right)&&left<0&&right<0);
    intent={0,1,false};assert(driveScales(intent,left,right)&&left<0&&right>0);
    intent={0,0,true};assert(driveScales(intent,left,right)&&close(left,0)&&close(right,0));
    intent={2,0,false};assert(!driveScales(intent,left,right));
    V10TrotFeedforwardConfig ff{true,4,4,2,2};float offset=0;
    assert(trotFeedforward(6,{1,0,false},ff,offset)&&close(offset,2));
    assert(trotFeedforward(6,{-1,0,false},ff,offset)&&close(offset,-2));
    assert(trotFeedforward(6,{0,1,false},ff,offset)&&offset>0);
    assert(trotFeedforward(6,{0,-1,false},ff,offset)&&offset>0);
    assert(trotFeedforward(6,{0,0,true},ff,offset)&&close(offset,0));
    ff.enabled=false;assert(trotFeedforward(6,{1,0,false},ff,offset)&&close(offset,0));

    V10DriveSession session(500);
    assert(session.epoch()==1 && !session.active());
    assert(!session.begin(2,1,100));
    assert(session.begin(1,1,100));
    const std::uint32_t epoch=session.epoch();
    assert(session.active() && session.renew(epoch,2,400));
    assert(!session.renew(epoch,2,450)); // replay
    assert(!session.expire(899));
    assert(session.expire(900) && !session.active());
    assert(!session.renew(epoch,3,901)); // old epoch cannot revive
    assert(!session.canBegin(session.epoch(),10)&&session.consumeExpired());
    assert(session.begin(session.epoch(),10,1000));
    const auto second=session.epoch();
    assert(session.end(second,11) && !session.active());
    assert(session.begin(session.epoch(),20,2000));
    const auto third=session.epoch();
    assert(!session.renew(third,21,2500)); // exact expiry invalidates before renewal
    assert(!session.active()&&!session.canBegin(session.epoch(),22)&&
           session.consumeExpired()&&!session.consumeExpired()&&
           !session.renew(third,22,2501));
    assert(session.begin(session.epoch(),30,0xFFFFFF00u));
    const auto wrapped=session.epoch();
    assert(session.renew(wrapped,31,0x00000020u)); // rollover, still before deadline
    assert(session.end(wrapped,1)); // current-epoch stop is priority, seq cannot block it

    V10MovingCg controller({110.0f,1.1f,5.0f,0.10f,0.5f,2.0f,0.005f,0.05f});
    controller.start(0,0,1000,77);
    float shift=0;
    assert(controller.update(2.0f,1020,77,shift));
    assert(shift>0 && shift<=0.04f+0.001f); // +nose-up -> +body-x correction, slew limited
    for(int i=2;i<40;++i)assert(controller.update(2.0f,1000+i*20,77,shift));
    assert(shift<=0.5f && shift>0);
    assert(!controller.update(2.0f,1900,77,shift));
    assert(close(controller.shiftMm(),0));
    V10MovingCg invalid_gain({110,20,5,.1f,.5f,2,.005f,.05f});
    invalid_gain.start(0,0,1000,1);assert(!invalid_gain.referenceValid());

    V10CgPlanner planner({2.0f,0.05f,15000});
    planner.begin(0, -2.5f, 8.5f);
    V10CgPlanOutput out{};
    assert(planner.tick(0.02f,3.2f,0,-2.5f,8.5f,out));
    assert(out.waiting && close(out.phase,0));
    for(int i=0;i<40;++i)assert(planner.tick(0.02f,3.2f,0,-2.5f,8.5f,out));
    assert(out.body_x_mm<0 && out.phase==0); // slew has not reached target yet
    for(int i=0;i<30;++i)assert(planner.tick(0.02f,3.2f,0,-2.5f,8.5f,out));
    assert(!out.waiting && out.phase>=0.10f); // swing starts only after command arrives
    assert(out.actual_elapsed_s>out.nominal_elapsed_s);

    V10CgPlanner timeout({0.1f,0.01f,100});
    timeout.begin(0,-2.5f,8.5f);
    for(int i=0;i<5;++i) (void)timeout.tick(0.02f,3.2f,0,-2.5f,8.5f,out);
    assert(!timeout.tick(0.02f,3.2f,0,-2.5f,8.5f,out));
    V10CgPlanner invalid_planner({NAN,0.05f,100});
    invalid_planner.begin(0,-2.5f,8.5f);assert(!invalid_planner.active());
}
