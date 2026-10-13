#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include "Board/ApplepyV1.h"
#include "Ctrl/ChannelCalibration.h"
#include "Ctrl/Command.h"
#include "Ctrl/ServoController.h"
#include "Ctrl/V10Gait.h"
#include "Ctrl/V10ShankLinearMap.h"
#include "UserApp/RobotApp.h"

namespace {
struct Clock : dog::IClock {
    mutable std::uint32_t now=0;
    mutable std::uint32_t synthetic_work_remaining_ms=0;
    mutable std::uint32_t synthetic_cost_per_read_ms=1;
    std::uint32_t millis() const override{
        if(synthetic_work_remaining_ms){const std::uint32_t cost=
            synthetic_work_remaining_ms<synthetic_cost_per_read_ms?
            synthetic_work_remaining_ms:synthetic_cost_per_read_ms;
            now+=cost;synthetic_work_remaining_ms-=cost;}
        return now;
    }
    void delayMs(std::uint32_t ms) override{now+=ms;}
};
struct Write { std::uint8_t channel; std::uint16_t pulse; };
struct Pwm : dog::IServoPwm {
    int writes=0,offs=0,fail_on=-1;
    std::array<std::uint16_t,16> pulse{};
    std::vector<Write> history;
    bool begin() override{return true;}
    bool allOff() override{++offs;return true;}
    bool offChannel(std::uint8_t) override{return true;}
    bool setPulseUs(std::uint8_t ch,std::uint16_t us) override{
        ++writes;
        if(writes==fail_on)return false;
        pulse[ch]=us; history.push_back({ch,us}); return true;
    }
};
struct Store : dog::ICalibrationStore {
    std::array<std::uint8_t,16> center{}; int saves=0;
    Store(){center.fill(90);}
    bool load(std::uint8_t ch,std::uint8_t& v) override{v=center[ch];return true;}
    bool save(std::uint8_t,std::uint8_t) override{++saves;return true;}
};
struct MotionStore : dog::IMotionParameterStore {
    dog::MotionParameterRecord record{};
    dog::MotionStoreResult load_result=dog::MotionStoreResult::Missing;
    dog::MotionStoreResult save_result=dog::MotionStoreResult::Ok;
    int loads=0,saves=0;
    dog::MotionStoreResult load(dog::MotionParameterRecord& out) override {
        ++loads; if(load_result==dog::MotionStoreResult::Ok) out=record; return load_result;
    }
    dog::MotionStoreResult save(const dog::MotionParameterRecord& in) override {
        ++saves; if(save_result==dog::MotionStoreResult::Ok) {
            record=in; load_result=dog::MotionStoreResult::Ok;
        }
        return save_result;
    }
};
struct ProfileStore : dog::IControlProfileStore {
    dog::ControlProfileRecord record{};
    dog::ControlProfileStoreResult load_result=dog::ControlProfileStoreResult::Missing;
    dog::ControlProfileStoreResult save_result=dog::ControlProfileStoreResult::Ok;
    int loads=0,saves=0;
    dog::ControlProfileStoreResult load(dog::ControlProfileRecord& out)override{
        ++loads;if(load_result==dog::ControlProfileStoreResult::Ok)out=record;return load_result;}
    dog::ControlProfileStoreResult save(const dog::ControlProfileRecord& in)override{
        ++saves;if(save_result==dog::ControlProfileStoreResult::Ok){record=in;load_result=dog::ControlProfileStoreResult::Ok;}
        return save_result;}
};
std::uint32_t profileChecksum(const dog::ControlProfileRecord& record){
    std::uint32_t hash=2166136261U;
    const auto mix=[&hash](const void* data,std::size_t size){const auto* bytes=
        static_cast<const std::uint8_t*>(data);for(std::size_t i=0;i<size;++i){hash^=bytes[i];hash*=16777619U;}};
    mix(&record.magic,sizeof(record.magic));mix(&record.version,sizeof(record.version));
    mix(&record.length,sizeof(record.length));mix(&record.flags,sizeof(record.flags));
    mix(record.values,sizeof(record.values));return hash;
}
struct Imu : dog::IImu {
    Clock& c; bool fail=false,nonfinite=false; int reads=0; float sensor_pitch_deg=0,sensor_roll_deg=0;
    float sensor_pitch_rate_deg_s=0,sensor_roll_rate_deg_s=0;
    explicit Imu(Clock& x):c(x){}
    bool begin() override{return true;}
    bool read(dog::ImuSample& s) override{
        ++reads;
        if(fail)return false;
        if(nonfinite){s.timestamp_ms=c.now;s.accel_mps2[0]=NAN;return true;}
        constexpr float k=0.01745329251994329577f;
        const float pitch=sensor_pitch_deg*k,roll=sensor_roll_deg*k;
        s.timestamp_ms=c.now;
        s.accel_mps2[0]=-9.8f*std::sin(pitch);
        s.accel_mps2[1]=9.8f*std::sin(roll)*std::cos(pitch);
        s.accel_mps2[2]=9.8f*std::cos(roll)*std::cos(pitch);
        s.gyro_radps[0]=sensor_roll_rate_deg_s*k;
        s.gyro_radps[1]=sensor_pitch_rate_deg_s*k;
        return true;
    }
};
struct Console : dog::IConsole { std::string last; void line(const char* s) override{last=s;} };
struct Rig {
    Clock clock; Pwm pwm; Store store; MotionStore local_motion_store;ProfileStore local_profile_store;
    dog::IMotionParameterStore* motion_store;dog::IControlProfileStore* profile_store; Imu imu{clock}; Console console;
    dog::ServoController servo{pwm,clock,500,2500,0,dog::board::kSelectedServoChannels,8,500,2500};
    dog::ChannelCalibration cal{store,servo,dog::board::kSelectedServoChannels,8,500,2500};
    dog::RobotApp app;
    explicit Rig(const dog::V10MotionConfig* config=&dog::board::kV10MotionConfig,
                 dog::IMotionParameterStore* external_motion_store=nullptr,
                 dog::IControlProfileStore* external_profile_store=nullptr)
        : motion_store(external_motion_store ? external_motion_store : &local_motion_store),
          profile_store(external_profile_store ? external_profile_store : &local_profile_store),
          app(servo,cal,imu,clock,console,dog::board::kRelativeServoLimits,8,
              dog::board::kShankMapConfigs,dog::board::kShankMapConfigCount,
              config,motion_store,profile_store) {app.begin();}
    void advance(std::uint32_t ms){clock.now+=ms;app.tick();}
};

int finishPreflight(Rig& rig,int max_ticks=10000,std::uint32_t step_ms=1) {
    int ticks=0;
    dog::DebugSnapshot state=rig.app.snapshot();
    while(state.preview_pending&&ticks<max_ticks){
        const std::uint32_t before_frames=state.preview_frames_checked;
        rig.advance(step_ms);
        ++ticks;
        state=rig.app.snapshot();
        assert(state.preview_frames_checked-before_frames<=1U);
    }
    assert(!state.preview_pending);
    return ticks;
}

const char* commandFor(dog::MotionMode mode) {
    using dog::MotionMode;
    switch(mode) {
    case MotionMode::Stand:return "stand";
    case MotionMode::Walk:return "walk";
    case MotionMode::Trot:return "trot";
    case MotionMode::Leg1:return "leg_test 1";
    case MotionMode::Leg2:return "leg_test 2";
    case MotionMode::Leg3:return "leg_test 3";
    case MotionMode::Leg4:return "leg_test 4";
    default:return "";
    }
}

void assertLastFrame(const Rig& r, const dog::V10MotionFrame& expected) {
    assert(r.pwm.history.size()>=8);
    const std::size_t first=r.pwm.history.size()-8;
    for(int i=0;i<8;++i) {
        assert(r.pwm.history[first+i].channel==dog::board::kSelectedServoChannels[i]);
        assert(r.pwm.history[first+i].pulse==expected.pulse_us[i]);
    }
}

void startAndTransition(Rig& r, dog::MotionMode mode) {
    using namespace dog;
    if(mode==MotionMode::Walk)
        assert(r.app.execute(parseCommand("motion_set 6 2 2.5 108 0 2")).ok);
    assert(r.app.execute(parseCommand("arm")).ok);
    assert(r.app.execute(parseCommand(commandFor(mode))).ok);
    assert(r.pwm.writes==0 && r.app.snapshot().preview_pending &&
           !r.app.snapshot().motion_pending);
    assert(finishPreflight(r)>1);
    assert(r.pwm.writes==0 && r.app.snapshot().motion_pending);
    r.app.tick(); // Same sample/time is not a fresh-IMU start opportunity.
    assert(r.pwm.writes==0 && r.app.snapshot().motion_pending);
    r.advance(10); // Fresh IMU tick emits exactly the eight current centres.
    assert(r.pwm.writes==8 && r.app.snapshot().motion_transition);
    for(auto ch:board::kSelectedServoChannels) assert(r.pwm.pulse[ch]==1500);
    assert(r.pwm.pulse[12]==1500 && r.pwm.pulse[8]==0);
    for(int i=0;i<75;++i) r.advance(20);
    assert(!r.app.snapshot().motion_transition);
    assert(r.pwm.writes==8*76);
}

void prepareReference(Rig& r) {
    using namespace dog;
    startAndTransition(r,MotionMode::Stand);
    for(int i=0;i<30;++i)r.advance(10);
    assert(r.app.execute(parseCommand("balance_capture")).ok);
    for(int i=0;i<20;++i)r.advance(10);
    assert(r.app.snapshot().balance_reference_ready);
}

void fullCycleMatchesModel(dog::MotionMode mode) {
    using namespace dog;
    Rig r;
    const int initial_off=r.pwm.offs;
    startAndTransition(r,mode);
    V10Gait gait(board::kV10MotionConfig); V10MotionParameters p{};
    if(mode==MotionMode::Walk){p.step_length_mm=6;p.lift_mm=2;p.period_s=2.5f;
        p.stand_down_mm=108;p.stand_forward_mm=0;p.walk_body_shift_mm=2;}
    V10AdvancedMotionParameters advanced=r.app.snapshot().advanced_parameters;
    std::uint8_t centers[8]={90,90,90,90,90,90,90,90};
    V10MotionFrame expected{};
    assert(gait.frameAdvanced(mode,0,p,advanced,centers,expected));
    assertLastFrame(r,expected);
    const int after_transition=r.pwm.writes;
    const int cycle_steps=static_cast<int>(std::lround(p.period_s*1000.0f/20.0f));
    for(int i=1;i<=cycle_steps;++i) {
        r.advance(20);
        if(mode==MotionMode::Stand) {
            assert(r.pwm.writes==after_transition); // Stable stand holds final frame.
        } else {
            assert(r.pwm.writes==after_transition+8*i);
            assert(gait.frameAdvanced(mode,static_cast<float>(i)/cycle_steps,p,
                                      advanced,centers,expected));
            assertLastFrame(r,expected);
        }
        assert(r.pwm.offs==initial_off);
    }
    assert(r.store.saves==0);
}
}

int main(){
    using namespace dog;
    static_assert(kDefaultStraightTrim==0.25f);
    assert(parseCommand("stand").type==CommandType::Stand);
    assert(parseCommand("walk").type==CommandType::Walk);
    assert(parseCommand("trot").type==CommandType::Trot);
    assert(parseCommand("leg_test 4").type==CommandType::LegTest);
    assert(parseCommand("leg_test 0").type==CommandType::Invalid);
    assert(parseCommand("motion_set 6 2 4.0 108 0 2").type==CommandType::MotionSet);
    assert(parseCommand("motion_set 6 2 nan 108 0 2").type==CommandType::Invalid);
    assert(parseCommand("motion_set 6 2 4x 108 0 2").type==CommandType::Invalid);
    assert(parseCommand("motion_preview leg3").motion_mode==MotionMode::Leg3);
    assert(parseCommand("pose_live_set 108 -4 7 2").type==CommandType::PoseLiveSet);
    assert(std::fabs(parseCommand("pose_live_set 108 -4 7 2").pose_live_values[3]-2)<0.001f);
    assert(parseCommand("pose_live_set 108 -4 7").type==CommandType::Invalid);
    assert(parseCommand("pose_live_set 108 nan 0 2").type==CommandType::Invalid);
    assert(parseCommand("pose_live_set 108 0 0 2 extra").type==CommandType::Invalid);
    assert(parseCommand("arm_stand").type==CommandType::ArmStand);
    assert(parseCommand("motion_save").type==CommandType::MotionSave);
    assert(parseCommand("motion_load").type==CommandType::MotionLoad);
    assert(parseCommand("body_set 1 -1 3 1 2.5").type==CommandType::BodySet);
    assert(parseCommand("body_set 1 -1 nan 1 2.5").type==CommandType::Invalid);
    assert(parseCommand("body_set 1 -1 3 1 2.5x").type==CommandType::Invalid);
    assert(parseCommand("balance_capture").type==CommandType::BalanceCapture);
    assert(parseCommand("balance_on").type==CommandType::BalanceOn);
    assert(parseCommand("balance_off").type==CommandType::BalanceOff);
    assert(parseCommand("balance_roll_sign -1").axis_sign==-1);
    assert(parseCommand("balance_roll_sign +1").axis_sign==1);
    assert(parseCommand("balance_roll_sign 0").type==CommandType::BalanceRollSign);
    assert(parseCommand("balance_roll_sign 2").type==CommandType::Invalid);
    assert(parseCommand("drive_begin 1 2 trot -1 0 0").type==CommandType::DriveBegin);
    assert(parseCommand("drive_begin 1 2 walk 0 1 1").drive_intent.stepping);
    assert(parseCommand("drive_begin 1 2 walk nan 0 0").type==CommandType::Invalid);
    assert(parseCommand("drive_begin 1 2 walk 0 0 2").type==CommandType::Invalid);
    assert(parseCommand("drive_renew 2 3").type==CommandType::DriveRenew);
    assert(parseCommand("drive_end 2 4").type==CommandType::DriveEnd);
    assert(parseCommand("drive_renew 4294967295 4294967294").epoch==0xFFFFFFFFU);
    assert(parseCommand("drive_renew 4294967296 1").type==CommandType::Invalid);
    assert(parseCommand("moving_cg_on").type==CommandType::MovingCgOn);
    assert(parseCommand("moving_cg_off").type==CommandType::MovingCgOff);
    assert(parseCommand("control_set 20 15 1.1 5 0.1 0.5 2 0.8 0.08 0.9 0.07").type==CommandType::ControlSet);
    assert(parseCommand("control_set 20 15 nan 5 0.1 0.5 2 0.8 0.08 0.9 0.07").type==CommandType::Invalid);
    assert(parseCommand("profile_save").type==CommandType::ProfileSave);
    assert(parseCommand("profile_load").type==CommandType::ProfileLoad);
    assert(parseCommand("trot_ff_set 1 4 4 2 2").type==CommandType::TrotFeedforwardSet);
    assert(parseCommand("straight_trim_set 0.05").type==CommandType::StraightTrimSet);
    assert(parseCommand("straight_trim_set nan").type==CommandType::Invalid);
    assert(parseCommand("trot_pitch_set -0.5").type==CommandType::TrotPitchSet);
    assert(std::fabs(parseCommand("trot_pitch_set -0.5").trot_pitch_offset_deg+0.5f)<0.001f);
    assert(parseCommand("trot_pitch_set nan").type==CommandType::Invalid);
    assert(parseCommand("trot_pitch_set -0.5 extra").type==CommandType::Invalid);

    // Confirmed Roll installation sign is a pure boot default only. It never
    // restores enable/reference, arms outputs, moves PWM, or writes either NVS.
    Rig roll_boot;
    const DebugSnapshot roll_boot_state=roll_boot.app.snapshot();
    assert(std::fabs(roll_boot_state.motion_parameters.step_length_mm-17.9f)<0.001f&&
           std::fabs(roll_boot_state.motion_parameters.lift_mm-12.8f)<0.001f&&
           std::fabs(roll_boot_state.motion_parameters.period_s-0.5f)<0.001f&&
           std::fabs(roll_boot_state.motion_parameters.stand_down_mm-114.0f)<0.001f);
    assert(std::string(roll_boot_state.motion_parameter_state)=="nvs_missing_using_default");
    assert(roll_boot_state.balance_roll_sensor_sign==kDefaultRollSensorSign);
    assert(kDefaultRollSensorSign==-1 && !roll_boot_state.balance_enabled &&
           !roll_boot_state.balance_reference_ready && !roll_boot_state.armed);
    assert(!roll_boot_state.control_profile.trot_feedforward.enabled&&
           std::fabs(roll_boot_state.trot_feedforward_mm)<0.0001f&&
           std::fabs(roll_boot_state.control_profile.body.body_x_mm-8.0f)<0.001f);
    assert(std::fabs(roll_boot_state.straight_trim-0.25f)<0.001f&&
           std::fabs(roll_boot_state.effective_drive_turn)<0.001f);
    assert(std::fabs(roll_boot_state.trot_pitch_offset_deg)<0.001f&&
           std::fabs(roll_boot_state.trot_effective_pitch_deg-
                     roll_boot_state.advanced_parameters.body.pitch_deg)<0.001f);
    assert(roll_boot.pwm.writes==0 && roll_boot.store.saves==0 &&
           roll_boot.local_motion_store.saves==0);

    // The built-in straight trim applies to ordinary straight starts without
    // changing the requested V10DriveIntent default itself.
    Rig default_trim_start;
    assert(default_trim_start.app.execute(parseCommand("arm")).ok);
    assert(default_trim_start.app.execute(parseCommand("trot")).ok);
    assert(std::fabs(default_trim_start.app.snapshot().straight_trim-0.25f)<0.001f&&
           std::fabs(default_trim_start.app.snapshot().effective_drive_turn+0.25f)<0.001f);
    assert(finishPreflight(default_trim_start)>1&&
           default_trim_start.app.snapshot().motion_pending);
    assert(default_trim_start.app.execute(parseCommand("motion_stop")).ok);

    // Straight trim is an explicit RAM-only diagnostic. Applying it is a
    // zero-output operation; straight starts/previews receive an effective
    // turn, while explicit turn and stepping intents remain untouched.
    Rig trim;
    const int trim_writes=trim.pwm.writes;
    const DebugReply trim_applied=trim.app.execute(parseCommand("straight_trim_set 0.05"));
    assert(trim_applied.ok&&std::string(trim_applied.message).find("default 0.25")!=std::string::npos);
    assert(trim.pwm.writes==trim_writes&&std::fabs(trim.app.snapshot().straight_trim-0.05f)<0.001f);
    assert(!trim.app.execute(parseCommand("straight_trim_set 0.31")).ok);
    assert(std::fabs(trim.app.snapshot().straight_trim-0.05f)<0.001f);
    assert(trim.app.execute(parseCommand("motion_preview trot")).ok);
    assert(std::fabs(trim.app.snapshot().effective_drive_turn+0.05f)<0.001f&&trim.pwm.writes==trim_writes);
    finishPreflight(trim);
    assert(trim.pwm.writes==trim_writes);
    assert(trim.app.execute(parseCommand("off")).ok&&std::fabs(trim.app.snapshot().straight_trim-0.05f)<0.001f);
    assert(trim.app.execute(parseCommand("arm")).ok);
    assert(trim.app.execute(parseCommand("trot")).ok);
    assert(std::fabs(trim.app.snapshot().effective_drive_turn+0.05f)<0.001f);
    assert(finishPreflight(trim)>1&&trim.app.snapshot().motion_pending);
    V10MotionFrame trimmed_frame{};std::uint8_t trim_centers[8]={90,90,90,90,90,90,90,90};
    const DebugSnapshot trim_state=trim.app.snapshot();
    assert(V10Gait(board::kV10MotionConfig).frameAdvanced(MotionMode::Trot,0.125f,
        trim_state.motion_parameters,trim_state.advanced_parameters,trim_centers,trimmed_frame));
    assert(std::fabs(trimmed_frame.forward_mm[0]-trimmed_frame.forward_mm[2])>0.1f);
    assert(!trim.app.execute(parseCommand("straight_trim_set 0.04")).ok);
    assert(trim.app.execute(parseCommand("motion_stop")).ok&&std::fabs(trim.app.snapshot().straight_trim-0.05f)<0.001f);
    Rig trim_reboot;
    assert(std::fabs(trim_reboot.app.snapshot().straight_trim-0.25f)<0.001f);

    // Trot Pitch compensation is an independently revisioned RAM-only value.
    // Applying it preflights the current default-forward Trot with current
    // centres/trim, sends no PWM, and survives Stop/Off but not reboot.
    Rig trot_pitch;
    const int pitch_writes=trot_pitch.pwm.writes;
    const std::uint32_t pitch_revision=trot_pitch.app.snapshot().trot_pitch_revision;
    assert(trot_pitch.app.execute(parseCommand("trot_pitch_set -0.5")).ok);
    DebugSnapshot pitch_state=trot_pitch.app.snapshot();
    assert(trot_pitch.pwm.writes==pitch_writes&&
           std::fabs(pitch_state.trot_pitch_offset_deg+0.5f)<0.001f&&
           std::fabs(pitch_state.trot_effective_pitch_deg+0.5f)<0.001f&&
           pitch_state.trot_pitch_revision==pitch_revision+1);
    assert(!trot_pitch.app.execute(parseCommand("trot_pitch_set -0.6")).ok);
    assert(!trot_pitch.app.execute(parseCommand("trot_pitch_set 0.1")).ok);
    assert(!trot_pitch.app.execute(parseCommand("trot_pitch_set -0.25")).ok);
    assert(std::fabs(trot_pitch.app.snapshot().trot_pitch_offset_deg+0.5f)<0.001f&&
           trot_pitch.pwm.writes==pitch_writes);
    assert(trot_pitch.app.execute(parseCommand("motion_preview trot")).ok);
    assert(std::fabs(trot_pitch.app.snapshot().effective_drive_turn+0.25f)<0.001f);
    finishPreflight(trot_pitch);
    assert(std::string(trot_pitch.app.snapshot().preview_result)=="passed"&&
           trot_pitch.pwm.writes==pitch_writes);
    assert(trot_pitch.app.execute(parseCommand("off")).ok&&
           std::fabs(trot_pitch.app.snapshot().trot_pitch_offset_deg+0.5f)<0.001f);
    assert(trot_pitch.app.execute(parseCommand("arm")).ok);
    assert(trot_pitch.app.execute(parseCommand("trot")).ok);
    assert(!trot_pitch.app.execute(parseCommand("trot_pitch_set 0")).ok);
    finishPreflight(trot_pitch);
    assert(trot_pitch.app.execute(parseCommand("motion_stop")).ok&&
           std::fabs(trot_pitch.app.snapshot().trot_pitch_offset_deg+0.5f)<0.001f);
    assert(trot_pitch.app.execute(parseCommand("off")).ok&&
           std::fabs(trot_pitch.app.snapshot().trot_pitch_offset_deg+0.5f)<0.001f);
    Rig trot_pitch_reboot;
    assert(std::fabs(trot_pitch_reboot.app.snapshot().trot_pitch_offset_deg)<0.001f&&
           trot_pitch_reboot.app.snapshot().trot_pitch_revision==0);
    Rig trot_pitch_direction;
    assert(trot_pitch_direction.app.execute(parseCommand("trot_pitch_set -0.5")).ok);
    assert(trot_pitch_direction.app.execute(parseCommand("arm")).ok);
    char pitch_drive[96];std::snprintf(pitch_drive,sizeof(pitch_drive),
        "drive_begin %lu 1 trot 1 0 0",
        static_cast<unsigned long>(trot_pitch_direction.app.snapshot().drive_epoch));
    assert(trot_pitch_direction.app.execute(parseCommand(pitch_drive)).ok);
    assert(trot_pitch_direction.pwm.writes==0&&finishPreflight(trot_pitch_direction)>1&&
           trot_pitch_direction.app.snapshot().motion_pending);
    assert(trot_pitch_direction.app.execute(parseCommand("motion_stop")).ok);

    // A geometrically invalid candidate is rejected transactionally. The
    // current 12.9 mm lift is valid for Stand but not for -0.5 degree Trot.
    Rig pitch_reject;
    assert(pitch_reject.app.execute(parseCommand("motion_set 17.9 12.9 0.5 114 0 2")).ok);
    assert(!pitch_reject.app.execute(parseCommand("trot_pitch_set -0.5")).ok);
    assert(std::fabs(pitch_reject.app.snapshot().trot_pitch_offset_deg)<0.001f&&
           pitch_reject.pwm.writes==0);

    // Profile persistence is deliberately independent: save/load never
    // changes the volatile offset, while a reboot always returns it to zero.
    ProfileStore pitch_profile_store;
    {
        Rig pitch_profile(&board::kV10MotionConfig,nullptr,&pitch_profile_store);
        assert(pitch_profile.app.execute(parseCommand("trot_pitch_set -0.5")).ok);
        assert(pitch_profile.app.execute(parseCommand("profile_save")).ok);
        assert(pitch_profile.app.execute(parseCommand("trot_pitch_set -0.2")).ok);
        assert(pitch_profile.app.execute(parseCommand("profile_load")).ok);
        assert(std::fabs(pitch_profile.app.snapshot().trot_pitch_offset_deg+0.2f)<0.001f);
    }
    Rig pitch_profile_reboot(&board::kV10MotionConfig,nullptr,&pitch_profile_store);
    assert(std::fabs(pitch_profile_reboot.app.snapshot().trot_pitch_offset_deg)<0.001f);

    const struct { const char* raw; float expected_turn; } trim_intents[]={
        {"1 0 0",-0.25f},{"-1 0 0",0.25f},{"0 1 0",1.0f},
        {"0 -1 0",-1.0f},{"0 0 1",0.0f}};
    for(bool apply_explicit_default:{false,true})for(const auto& sample:trim_intents){
        Rig direction_trim;
        if(apply_explicit_default)
            assert(direction_trim.app.execute(parseCommand("straight_trim_set 0.25")).ok);
        assert(direction_trim.app.execute(parseCommand("arm")).ok);
        const std::uint32_t epoch=direction_trim.app.snapshot().drive_epoch;
        char command[96];std::snprintf(command,sizeof(command),"drive_begin %lu 1 trot %s",
            static_cast<unsigned long>(epoch),sample.raw);
        assert(direction_trim.app.execute(parseCommand(command)).ok);
        assert(std::fabs(direction_trim.app.snapshot().effective_drive_turn-sample.expected_turn)<0.001f);
        assert(direction_trim.pwm.writes==0);
        assert(direction_trim.app.execute(parseCommand("motion_stop")).ok);
    }

    // Straight trim is excluded from both persistent records. Save/load leaves
    // the current RAM value alone, while a new RobotApp boot restores 0.25.
    MotionStore trim_motion_store;
    {
        Rig trim_motion(&board::kV10MotionConfig,&trim_motion_store);
        assert(trim_motion.app.execute(parseCommand("straight_trim_set 0.08")).ok);
        assert(trim_motion.app.execute(parseCommand("motion_save")).ok);
        assert(trim_motion.app.execute(parseCommand("straight_trim_set 0.09")).ok);
        assert(trim_motion.app.execute(parseCommand("motion_load")).ok);
        assert(std::fabs(trim_motion.app.snapshot().straight_trim-0.09f)<0.001f);
    }
    Rig trim_motion_reboot(&board::kV10MotionConfig,&trim_motion_store);
    assert(std::fabs(trim_motion_reboot.app.snapshot().straight_trim-0.25f)<0.001f);
    ProfileStore trim_profile_store;
    {
        Rig trim_profile(&board::kV10MotionConfig,nullptr,&trim_profile_store);
        assert(trim_profile.app.execute(parseCommand("straight_trim_set 0.08")).ok);
        assert(trim_profile.app.execute(parseCommand("profile_save")).ok);
        assert(trim_profile.app.execute(parseCommand("straight_trim_set 0.09")).ok);
        assert(trim_profile.app.execute(parseCommand("profile_load")).ok);
        assert(std::fabs(trim_profile.app.snapshot().straight_trim-0.09f)<0.001f);
    }
    Rig trim_profile_reboot(&board::kV10MotionConfig,nullptr,&trim_profile_store);
    assert(std::fabs(trim_profile_reboot.app.snapshot().straight_trim-0.25f)<0.001f);

    // Real 0.8.3 regression: a valid 2.5 s Walk preview took 476 ms on the
    // ESP32-C3. The old synchronous execute() path let the 100 ms IMU sample
    // expire and rejected the otherwise-passing report. Acceptance must now
    // only queue bounded work and emit no target PWM in the request handler.
    Rig slow_preflight;
    const std::uint8_t user_centers[8]={90,100,90,90,90,97,90,102};
    for(int i=0;i<8;++i)
        slow_preflight.store.center[board::kSelectedServoChannels[i]]=user_centers[i];
    slow_preflight.app.begin();
    assert(slow_preflight.app.execute(parseCommand("motion_set 6 2 2.5 108 0 2")).ok);
    assert(slow_preflight.app.execute(parseCommand("arm_stand")).ok);
    slow_preflight.advance(10);
    for(int i=0;i<80;++i)slow_preflight.advance(20);
    assert(slow_preflight.app.execute(parseCommand("motion_stop")).ok);
    const int before_slow_preflight=slow_preflight.pwm.writes;
    const DebugReply queued_walk=slow_preflight.app.execute(parseCommand("walk"));
    assert(queued_walk.ok);
    slow_preflight.clock.synthetic_work_remaining_ms=476;
    assert(slow_preflight.app.snapshot().preview_pending&&
           !slow_preflight.app.snapshot().motion_pending&&
           slow_preflight.pwm.writes==before_slow_preflight);
    const int reads_before_preflight=slow_preflight.imu.reads;
    const int preflight_ticks=finishPreflight(slow_preflight,2000,1);
    const DebugSnapshot completed_preflight=slow_preflight.app.snapshot();
    assert(preflight_ticks>1&&slow_preflight.imu.reads-reads_before_preflight>1);
    assert(std::string(completed_preflight.preview_result)=="passed"&&
           completed_preflight.preview_frames_checked>=500&&
           completed_preflight.preview_max_slice_ms>=1&&
           completed_preflight.preflight_duration_ms>=476&&
           completed_preflight.motion_pending&&completed_preflight.imu_ok&&
           slow_preflight.pwm.writes==before_slow_preflight);
    slow_preflight.app.tick();
    assert(slow_preflight.pwm.writes==before_slow_preflight);
    slow_preflight.advance(10);
    assert(slow_preflight.pwm.writes==before_slow_preflight&&
           slow_preflight.app.snapshot().motion_transition);
    slow_preflight.advance(20);
    assert(slow_preflight.pwm.writes==before_slow_preflight+8);

    Rig fresh_100;
    assert(fresh_100.app.execute(parseCommand("arm")).ok);
    fresh_100.clock.now+=100;
    assert(fresh_100.app.execute(parseCommand("trot")).ok&&
           fresh_100.app.snapshot().preview_pending);
    assert(fresh_100.app.execute(parseCommand("motion_stop")).ok);
    Rig stale_101;
    assert(stale_101.app.execute(parseCommand("arm")).ok);
    stale_101.clock.now+=101;
    assert(!stale_101.app.execute(parseCommand("trot")).ok&&
           !stale_101.app.snapshot().preview_pending);

    Rig cancelled_check;
    assert(cancelled_check.app.execute(parseCommand("motion_set 6 2 2.5 108 0 2")).ok);
    assert(cancelled_check.app.execute(parseCommand("arm")).ok);
    assert(cancelled_check.app.execute(parseCommand("walk")).ok);
    for(int i=0;i<25;++i)cancelled_check.advance(1);
    const int cancelled_writes=cancelled_check.pwm.writes;
    assert(cancelled_check.app.execute(parseCommand("motion_stop")).ok);
    assert(!cancelled_check.app.snapshot().preview_pending&&
           std::string(cancelled_check.app.snapshot().preview_result)=="cancelled_motion_stop");
    for(int i=0;i<600;++i)cancelled_check.advance(1);
    assert(cancelled_check.pwm.writes==cancelled_writes&&
           !cancelled_check.app.snapshot().motion_pending);
    assert(cancelled_check.app.execute(parseCommand("off")).ok);
    assert(cancelled_check.app.execute(parseCommand("arm_stand")).ok);
    assert(std::string(cancelled_check.app.snapshot().preview_result)=="passed"&&
           std::string(cancelled_check.app.snapshot().preview_error)=="none");

    Rig imu_failed_check;
    assert(imu_failed_check.app.execute(parseCommand("arm")).ok);
    assert(imu_failed_check.app.execute(parseCommand("trot")).ok);
    imu_failed_check.imu.fail=true;imu_failed_check.advance(10);
    assert(!imu_failed_check.app.snapshot().preview_pending&&
           std::string(imu_failed_check.app.snapshot().preview_result)=="cancelled_imu_failure"&&
           !imu_failed_check.app.snapshot().armed);

    Rig attitude_reset_check;
    for(int i=0;i<120;++i)attitude_reset_check.advance(10);
    assert(attitude_reset_check.app.snapshot().attitude_ok);
    assert(attitude_reset_check.app.execute(parseCommand("motion_set 6 2 2.5 108 0 2")).ok);
    assert(attitude_reset_check.app.execute(parseCommand("arm")).ok);
    assert(attitude_reset_check.app.execute(parseCommand("walk")).ok);
    attitude_reset_check.advance(120);
    assert(!attitude_reset_check.app.snapshot().preview_pending&&
           std::string(attitude_reset_check.app.snapshot().preview_result)=="cancelled_attitude_reset"&&
           attitude_reset_check.pwm.writes==0);

    Rig preview;
    const int initial_off=preview.pwm.offs;
    assert(preview.app.execute(parseCommand("motion_preview walk")).ok);
    assert(finishPreflight(preview)>1);
    assert(std::string(preview.app.snapshot().preview_result)=="passed");
    assert(preview.app.execute(parseCommand("motion_preview trot")).ok);
    assert(finishPreflight(preview)>1);
    assert(std::string(preview.app.snapshot().preview_result)=="passed");
    assert(preview.app.execute(parseCommand("motion_set 6 2 1.2 108 0 2")).ok);
    assert(preview.app.execute(parseCommand("motion_preview walk")).ok);
    finishPreflight(preview);
    assert(std::string(preview.app.snapshot().preview_result)=="passed");
    assert(preview.app.execute(parseCommand("motion_set 6 2 4 108 0 2")).ok);
    assert(preview.pwm.writes==0 && preview.pwm.offs==initial_off && preview.store.saves==0);
    assert(!preview.app.execute(parseCommand("walk")).ok); // never implicit ARM

    // Global enable is an atomic validate+arm+Stand request.  It writes no
    // PWM in the command handler; the existing fresh-IMU gate owns first IO.
    Rig arm_stand;
    assert(arm_stand.app.execute(parseCommand("arm_stand")).ok);
    assert(arm_stand.app.snapshot().armed && arm_stand.app.snapshot().motion_pending);
    assert(arm_stand.pwm.writes==0);
    assert(!arm_stand.app.execute(parseCommand("arm_stand")).ok);
    assert(arm_stand.pwm.writes==0);
    arm_stand.app.tick();
    assert(arm_stand.pwm.writes==0 && arm_stand.app.snapshot().motion_pending);
    arm_stand.advance(10);
    assert(arm_stand.pwm.writes==8 && arm_stand.app.snapshot().motion_transition);
    assert(!arm_stand.app.execute(parseCommand("arm_stand")).ok);

    Rig arm_stand_off;
    assert(arm_stand_off.app.execute(parseCommand("arm_stand")).ok);
    assert(arm_stand_off.app.execute(parseCommand("off")).ok);
    arm_stand_off.advance(20);
    assert(arm_stand_off.pwm.writes==0 && !arm_stand_off.app.snapshot().armed &&
           !arm_stand_off.app.snapshot().motion_pending);

    // A first start request only plans and produces no PWM itself.
    Rig first_start;
    assert(first_start.app.execute(parseCommand("arm")).ok);
    const int before_first_start=first_start.pwm.writes;
    assert(first_start.app.execute(parseCommand("stand")).ok);
    assert(first_start.pwm.writes==before_first_start &&
           first_start.app.snapshot().preview_pending&&
           !first_start.app.snapshot().motion_pending);
    finishPreflight(first_start);
    assert(first_start.app.snapshot().motion_pending);

    // Only one bounded preflight job exists; a repeated start is rejected
    // rather than building an implicit queue.
    Rig replace_pending;
    assert(replace_pending.app.execute(parseCommand("motion_set 6 2 2.5 108 0 2")).ok);
    assert(replace_pending.app.execute(parseCommand("arm")).ok);
    assert(replace_pending.app.execute(parseCommand("walk")).ok);
    const int before_pending_replace=replace_pending.pwm.writes;
    assert(!replace_pending.app.execute(parseCommand("trot")).ok);
    assert(replace_pending.pwm.writes==before_pending_replace &&
           replace_pending.app.snapshot().preview_pending);
    finishPreflight(replace_pending);
    replace_pending.advance(10);
    assert(replace_pending.pwm.writes==8 &&
           std::string(replace_pending.app.snapshot().motion_mode)=="walk");

    // A planned start is already an exclusive motion session. Conflicting
    // calls are rejected before execute() can run the fresh-IMU motion tick.
    Rig pending_conflicts;
    assert(pending_conflicts.app.execute(parseCommand("motion_set 6 2 2.5 108 0 2")).ok);
    assert(pending_conflicts.app.execute(parseCommand("arm")).ok);
    assert(pending_conflicts.app.execute(parseCommand("walk")).ok);
    pending_conflicts.clock.now+=10; // fresh sample is available to the next tick
    const int before_pending_conflict=pending_conflicts.pwm.writes;
    assert(!pending_conflicts.app.execute(parseCommand("pwm 0 1500")).ok);
    assert(!pending_conflicts.app.execute(parseCommand("motion_preview trot")).ok);
    assert(!pending_conflicts.app.execute(parseCommand("motion_set 5 1 4 108 0 1")).ok);
    assert(!pending_conflicts.app.execute(parseCommand("save 0")).ok);
    assert(!pending_conflicts.app.execute(parseCommand("motion_save")).ok);
    assert(!pending_conflicts.app.execute(parseCommand("motion_load")).ok);
    assert(!pending_conflicts.app.execute(parseCommand("profile_save")).ok);
    assert(!pending_conflicts.app.execute(parseCommand("profile_load")).ok);
    assert(pending_conflicts.pwm.writes==before_pending_conflict &&
           pending_conflicts.app.snapshot().preview_pending&&
           !pending_conflicts.app.snapshot().motion_pending);

    for(MotionMode mode : {MotionMode::Stand,MotionMode::Walk,MotionMode::Trot,
                           MotionMode::Leg1,MotionMode::Leg2,MotionMode::Leg3,
                           MotionMode::Leg4}) fullCycleMatchesModel(mode);

    // A mode switch accepted while a periodic frame is due must not advance
    // that old frame inside execute(); it only arms a pending transition.
    Rig due_switch;
    startAndTransition(due_switch,MotionMode::Walk);
    due_switch.advance(20);
    due_switch.clock.now+=20;
    const int before_due_switch=due_switch.pwm.writes;
    assert(due_switch.app.execute(parseCommand("trot")).ok);
    const DebugSnapshot due_snapshot=due_switch.app.snapshot();
    assert(due_switch.pwm.writes==before_due_switch && due_snapshot.preview_pending &&
           !due_snapshot.motion_pending&&
           std::string(due_snapshot.motion_mode)=="walk");
    assert(finishPreflight(due_switch)>1);
    assert(due_switch.pwm.writes>before_due_switch&&
           std::string(due_switch.app.snapshot().motion_mode)=="walk"&&
           due_switch.app.snapshot().motion_pending);

    // Coordinated mode rejects conflicting bench/configuration actions before
    // execute() can advance a periodic frame.
    Rig conflicts;
    startAndTransition(conflicts,MotionMode::Walk);
    conflicts.advance(20);
    const int conflict_writes=conflicts.pwm.writes;
    assert(!conflicts.app.execute(parseCommand("pwm 0 1500")).ok);
    assert(!conflicts.app.execute(parseCommand("off 0")).ok);
    assert(!conflicts.app.execute(parseCommand("nudge 0 +1")).ok);
    assert(!conflicts.app.execute(parseCommand("cal_start")).ok);
    assert(!conflicts.app.execute(parseCommand("motion_preview trot")).ok);
    assert(!conflicts.app.execute(parseCommand("motion_set 5 1 4 108 0 1")).ok);
    assert(conflicts.pwm.writes==conflict_writes && conflicts.store.saves==0);

    // Motion stop during transition and run holds the last complete frame.
    Rig stop_transition;
    assert(stop_transition.app.execute(parseCommand("arm")).ok);
    assert(stop_transition.app.execute(parseCommand("trot")).ok);
    finishPreflight(stop_transition);
    stop_transition.advance(10); stop_transition.advance(20);
    const int transition_writes=stop_transition.pwm.writes;
    assert(stop_transition.app.execute(parseCommand("motion_stop")).ok);
    for(int i=0;i<220;++i) stop_transition.advance(20);
    assert(stop_transition.pwm.writes==transition_writes);
    assert(std::string(stop_transition.app.snapshot().motion_mode)=="hold");
    assert(stop_transition.app.snapshot().motion_phase==0);

    Rig stop_run;
    startAndTransition(stop_run,MotionMode::Walk); stop_run.advance(20);
    const int run_writes=stop_run.pwm.writes;
    assert(stop_run.app.execute(parseCommand("motion_stop")).ok);
    for(int i=0;i<220;++i) stop_run.advance(20);
    assert(stop_run.pwm.writes==run_writes);

    // OFF cancels pending/transition state and no later tick resurrects it.
    Rig pending;
    assert(pending.app.execute(parseCommand("arm")).ok);
    assert(pending.app.execute(parseCommand("trot")).ok);
    assert(pending.app.execute(parseCommand("off")).ok);
    for(int i=0;i<220;++i) pending.advance(20);
    assert(pending.pwm.writes==0 && !pending.app.snapshot().motion_pending &&
           !pending.app.snapshot().preview_pending&&!pending.app.snapshot().armed);
    Rig off_transition;
    assert(off_transition.app.execute(parseCommand("arm")).ok);
    assert(off_transition.app.execute(parseCommand("leg_test 2")).ok);
    finishPreflight(off_transition);
    off_transition.advance(10); off_transition.advance(20);
    const int before_off=off_transition.pwm.writes;
    assert(off_transition.app.execute(parseCommand("off")).ok);
    for(int i=0;i<220;++i) off_transition.advance(20);
    assert(off_transition.pwm.writes==before_off && !off_transition.app.snapshot().armed);

    // An IMU read failure globally shuts down; no later periodic frame writes.
    Rig imu_failure;
    startAndTransition(imu_failure,MotionMode::Trot);
    imu_failure.imu.fail=true;
    const int before_imu_failure=imu_failure.pwm.writes;
    const int before_imu_off=imu_failure.pwm.offs;
    imu_failure.advance(20);
    assert(imu_failure.pwm.writes==before_imu_failure);
    assert(imu_failure.pwm.offs==before_imu_off+1 && !imu_failure.servo.armed());
    assert(std::string(imu_failure.app.snapshot().motion_error)=="imu_read_failed");

    // Runtime model failure holds the last frame without a partial write.
    V10MotionConfig runtime_config=board::kV10MotionConfig;
    Rig runtime(&runtime_config);
    startAndTransition(runtime,MotionMode::Walk); runtime.advance(20);
    const int before_invalid=runtime.pwm.writes;
    runtime_config.geometry.upper_mm=0;
    runtime.advance(20);
    assert(runtime.pwm.writes==before_invalid);
    assert(std::string(runtime.app.snapshot().motion_mode)=="hold_error");
    assert(std::string(runtime.app.snapshot().motion_error)=="runtime_frame_invalid");

    Rig gap;
    startAndTransition(gap,MotionMode::Leg1); gap.advance(20);
    const int before_gap=gap.pwm.writes;
    gap.advance(201);
    assert(gap.pwm.writes==before_gap &&
           std::string(gap.app.snapshot().motion_mode)=="hold_error");
    assert(std::string(gap.app.snapshot().motion_error)=="scheduler_gap_gt_200ms");

    // Unsigned clock rollover remains a short, valid transition interval.
    Rig rollover;
    rollover.clock.now=0xFFFFFFF0u;
    rollover.app.tick(); // obtains a fresh sample at the near-wrap time
    assert(rollover.app.execute(parseCommand("arm")).ok);
    assert(rollover.app.execute(parseCommand("stand")).ok);
    finishPreflight(rollover);
    rollover.app.tick(); assert(rollover.pwm.writes==0);
    rollover.advance(20);
    assert(rollover.pwm.writes==8 && rollover.app.snapshot().motion_transition);
    rollover.advance(20);
    assert(rollover.pwm.writes==16 &&
           std::string(rollover.app.snapshot().motion_mode)!="hold_error");

    // A mid-frame bus failure attempts all-off, faults, and cannot restart.
    Rig failure;
    assert(failure.app.execute(parseCommand("motion_set 6 2 2.5 108 0 2")).ok);
    assert(failure.app.execute(parseCommand("arm")).ok);
    assert(failure.app.execute(parseCommand("walk")).ok);
    finishPreflight(failure);
    failure.pwm.fail_on=5;
    failure.advance(10);
    assert(failure.servo.faulted() && !failure.servo.armed() && failure.pwm.offs>=2);
    const int fault_writes=failure.pwm.writes;
    assert(!failure.app.execute(parseCommand("walk")).ok);
    assert(failure.pwm.writes==fault_writes);

    // Redefine Stand as the electrical centres after holding a running Walk.
    // This large but otherwise valid transition was rejected in 0.8.5 only
    // by the transition-speed guard; it now starts the normal 1.5 s blend.
    V10MotionConfig speed_config=board::kV10MotionConfig;
    Rig speed(&speed_config);
    startAndTransition(speed,MotionMode::Walk); speed.advance(20);
    assert(speed.app.execute(parseCommand("motion_stop")).ok);
    V10Gait old_gait(speed_config); V10MotionParameters defaults{};
    defaults.step_length_mm=6;defaults.lift_mm=2;defaults.period_s=2.5f;
    defaults.stand_down_mm=108;defaults.stand_forward_mm=0;defaults.walk_body_shift_mm=2;
    std::uint8_t c[8]={90,90,90,90,90,90,90,90}; V10MotionFrame old_stand{};
    assert(old_gait.frame(MotionMode::Stand,0,defaults,c,old_stand));
    speed_config.hip_geometry_neutral_deg = 90.0f +
        (old_stand.servo_deg[0]-90.0f)/speed_config.hip_direction[0];
    float stand_u=std::fabs(old_stand.servo_deg[1]-90.0f), stand_q=0;
    assert(V10ShankLinearMap(speed_config.shank_points,speed_config.shank_point_count)
               .qFromU(stand_u,stand_q));
    speed_config.knee_fold_neutral_deg-=stand_q;
    V10MotionFrame new_stand{};
    assert(V10Gait(speed_config).frame(MotionMode::Stand,0,defaults,c,new_stand));
    const DebugSnapshot current=speed.app.snapshot();
    bool target_differs=false;
    for(int i=0;i<8;++i)target_differs=target_differs||
        std::fabs(new_stand.servo_deg[i]-current.command_deg[i])>0.1f;
    assert(target_differs);
    assert(V10Gait(speed_config).preflight(MotionMode::Stand,defaults,c));
    speed.clock.now+=20; // Old Walk frame is due; execute itself must not emit it.
    const int before_switch=speed.pwm.writes;
    assert(speed.app.execute(parseCommand("stand")).ok);
    assert(speed.pwm.writes==before_switch);
    const int switch_preflight_ticks=finishPreflight(speed);
    assert(switch_preflight_ticks>0&&speed.pwm.writes==before_switch);
    assert(speed.app.snapshot().motion_pending&&
           std::string(speed.app.snapshot().preview_result)=="passed");
    bool entered_transition=false,transition_frame_written=false;
    for(int i=0;i<10&&!(entered_transition&&transition_frame_written);++i){
        speed.advance(10);
        const DebugSnapshot state=speed.app.snapshot();
        entered_transition=entered_transition||state.motion_transition;
        transition_frame_written=transition_frame_written||speed.pwm.writes>=before_switch+8;
    }
    const DebugSnapshot switching=speed.app.snapshot();
    assert(entered_transition&&transition_frame_written&&switching.motion_transition);
    assert(std::string(switching.motion_mode)=="stand"&&!switching.motion_pending&&
           std::string(switching.motion_error)=="none"&&
           std::string(switching.preview_error)=="none"&&
           std::string(switching.preview_result)=="passed");
    assert(speed.store.saves==0);

    // Stop/Hold and stable Stand are safe tuning seams: applying and previewing
    // perform no PWM writes and do not disturb the held frame.
    Rig hold_tune;
    startAndTransition(hold_tune,MotionMode::Trot); hold_tune.advance(20);
    assert(hold_tune.app.execute(parseCommand("motion_stop")).ok);
    const int hold_writes=hold_tune.pwm.writes;
    const auto hold_pulses=hold_tune.pwm.pulse;
    assert(hold_tune.app.execute(parseCommand("motion_set 4 1 1.2 108 0 1")).ok);
    assert(hold_tune.app.execute(parseCommand("motion_preview trot")).ok);
    finishPreflight(hold_tune);
    assert(hold_tune.pwm.writes==hold_writes && hold_tune.pwm.pulse==hold_pulses);
    assert(!hold_tune.app.snapshot().motion_parameters_saved);
    // The next explicit start uses the new RAM values and continues from Hold,
    // without writing an intermediate all-centres frame.
    assert(hold_tune.app.execute(parseCommand("trot")).ok);
    finishPreflight(hold_tune);
    hold_tune.advance(10);
    assert(hold_tune.pwm.writes==hold_writes && hold_tune.app.snapshot().motion_transition);
    hold_tune.advance(20);
    assert(hold_tune.pwm.writes==hold_writes+8);

    Rig stand_tune;
    startAndTransition(stand_tune,MotionMode::Stand);
    const int stand_writes=stand_tune.pwm.writes;
    assert(stand_tune.app.execute(parseCommand("motion_set 3 1 2 108 0 1")).ok);
    assert(stand_tune.pwm.writes==stand_writes);

    // Running/transition tuning remains rejected without opportunistic output.
    Rig running_tune;
    startAndTransition(running_tune,MotionMode::Trot); running_tune.advance(20);
    const int running_writes=running_tune.pwm.writes;
    assert(!running_tune.app.execute(parseCommand("motion_set 3 1 2 108 0 1")).ok);
    assert(running_tune.pwm.writes==running_writes);

    // A valid record saved with older parameter values remains authoritative;
    // A firmware upgrade must not replace it with newer built-in defaults. Saving needs
    // global OFF; reboot loading never arms or writes PWM and leaves centre NVS.
    MotionStore persisted;
    {
        Rig save_rig(&board::kV10MotionConfig,&persisted);
        assert(save_rig.app.execute(parseCommand("motion_set 4 1 1.5 107 1 1")).ok);
        assert(save_rig.app.execute(parseCommand("motion_save")).ok);
        assert(persisted.saves==1 && save_rig.store.saves==0);
        assert(save_rig.app.snapshot().motion_parameters_saved);
    }
    {
        Rig rebooted(&board::kV10MotionConfig,&persisted);
        const DebugSnapshot s=rebooted.app.snapshot();
        assert(std::fabs(s.motion_parameters.step_length_mm-4.0f)<0.001f&&
               std::fabs(s.motion_parameters.lift_mm-1.0f)<0.001f&&
               std::fabs(s.motion_parameters.period_s-1.5f)<0.001f&&
               std::fabs(s.motion_parameters.stand_down_mm-107.0f)<0.001f&&
               std::fabs(s.motion_parameters.stand_forward_mm-1.0f)<0.001f&&
               std::fabs(s.motion_parameters.walk_body_shift_mm-1.0f)<0.001f);
        assert(std::string(s.motion_parameter_state)=="loaded_nvs");
        assert(s.motion_parameters_saved && !s.armed && rebooted.pwm.writes==0);
    }
    MotionStore corrupt=persisted; corrupt.record.checksum^=1U;
    Rig corrupted(&board::kV10MotionConfig,&corrupt);
    assert(std::fabs(corrupted.app.snapshot().motion_parameters.lift_mm-12.8f)<0.001f&&
           std::fabs(corrupted.app.snapshot().motion_parameters.period_s-0.5f)<0.001f);
    assert(std::string(corrupted.app.snapshot().motion_parameter_state)==
           "nvs_invalid_using_default");
    MotionStore failed; failed.load_result=MotionStoreResult::IoError;
    Rig load_failed(&board::kV10MotionConfig,&failed);
    assert(std::string(load_failed.app.snapshot().motion_parameter_state)==
           "nvs_load_failed_using_default");
    failed.save_result=MotionStoreResult::IoError;
    assert(!load_failed.app.execute(parseCommand("motion_save")).ok);
    assert(std::string(load_failed.app.snapshot().motion_parameter_state)=="nvs_save_failed");

    // New profile is a third independent complete record. It persists body/CG
    // planning and moving-feedback tuning, never enable/reference/drive state,
    // and never writes either legacy store.
    ProfileStore profile_persisted;
    {
        Rig save_profile(&board::kV10MotionConfig,nullptr,&profile_persisted);
        assert(save_profile.app.execute(parseCommand("body_set 0 0 8 1 2.5")).ok);
        assert(save_profile.app.execute(parseCommand("pose_live_set 108 1 -1 2")).ok);
        assert(save_profile.app.execute(parseCommand("control_set 18 12 1.2 4 0.2 0.4 1.5 0.7 0.06 0.9 0.04")).ok);
        assert(save_profile.app.execute(parseCommand("trot_ff_set 1 4 3 2 1.5")).ok);
        assert(save_profile.app.execute(parseCommand("profile_save")).ok);
        const DebugSnapshot s=save_profile.app.snapshot();
        assert(profile_persisted.saves==1&&save_profile.store.saves==0&&
               save_profile.local_motion_store.saves==0&&s.control_profile_saved);
        assert(s.control_profile.trot_feedforward.enabled&&
               std::fabs(s.trot_feedforward_mm-1.5f)<0.001f);
    }
    {
        Rig profile_reboot(&board::kV10MotionConfig,nullptr,&profile_persisted);
        const DebugSnapshot s=profile_reboot.app.snapshot();
        assert(std::string(s.control_profile_state)=="loaded_nvs"&&s.control_profile_saved);
        assert(std::fabs(s.control_profile.body.pitch_deg-1)<0.001f&&
               std::fabs(s.control_profile.body.body_x_mm-2)<0.001f&&
               std::fabs(s.control_profile.planner.body_slew_mm_s-18)<0.001f&&
               std::fabs(s.control_profile.moving.max_abs_shift_mm-0.4f)<0.001f&&
               std::fabs(s.control_profile.pitch_kp-0.7f)<0.001f&&
               std::fabs(s.control_profile.roll_kp-0.9f)<0.001f&&
               std::fabs(s.control_profile.pitch_kd_s-0.06f)<0.001f&&
               std::fabs(s.control_profile.roll_kd_s-0.04f)<0.001f&&
               s.control_profile.trot_feedforward.enabled&&
               std::fabs(s.control_profile.trot_feedforward.backward_gain-3)<0.001f&&
               std::fabs(s.trot_feedforward_mm-1.5f)<0.001f);
        assert(!s.armed&&!s.balance_enabled&&!s.balance_reference_ready&&
               !s.moving_cg_enabled&&!s.drive_session_active&&profile_reboot.pwm.writes==0);
    }
    ProfileStore corrupt_profile=profile_persisted;corrupt_profile.record.values[6]=NAN;
    corrupt_profile.record.checksum=profileChecksum(corrupt_profile.record);
    Rig profile_corrupt(&board::kV10MotionConfig,nullptr,&corrupt_profile);
    assert(std::string(profile_corrupt.app.snapshot().control_profile_state)==
           "nvs_invalid_using_default"&&
           std::fabs(profile_corrupt.app.snapshot().control_profile.body.body_x_mm-8.0f)<0.001f);
    ProfileStore profile_failed;profile_failed.load_result=ControlProfileStoreResult::IoError;
    Rig profile_load_failed(&board::kV10MotionConfig,nullptr,&profile_failed);
    assert(std::string(profile_load_failed.app.snapshot().control_profile_state)==
           "nvs_load_failed_using_default"&&
           std::fabs(profile_load_failed.app.snapshot().control_profile.body.body_x_mm-8.0f)<0.001f);

    ProfileStore old_zero_profile=profile_persisted;
    old_zero_profile.record.values[2]=0.0f;
    old_zero_profile.record.checksum=profileChecksum(old_zero_profile.record);
    Rig old_zero_reboot(&board::kV10MotionConfig,nullptr,&old_zero_profile);
    assert(std::string(old_zero_reboot.app.snapshot().control_profile_state)=="loaded_nvs"&&
           old_zero_reboot.app.snapshot().control_profile_saved&&
           std::fabs(old_zero_reboot.app.snapshot().control_profile.body.body_x_mm)<0.001f);
    profile_failed.save_result=ControlProfileStoreResult::IoError;
    assert(!profile_load_failed.app.execute(parseCommand("profile_save")).ok);
    assert(std::string(profile_load_failed.app.snapshot().control_profile_state)=="nvs_save_failed");
    assert(!profile_load_failed.app.execute(parseCommand("control_set 40 12 1.2 4 0.2 0.4 1.5 0.8 0.08 0.8 0.08")).ok);

    // arm_stand validates before ARM.  Invalid geometry cannot leave an
    // accidental armed state; legacy bench ARM still never moves a channel.
    V10MotionConfig invalid_arm=board::kV10MotionConfig;
    invalid_arm.geometry.upper_mm=0;
    Rig denied_arm(&invalid_arm);
    assert(!denied_arm.app.execute(parseCommand("arm_stand")).ok);
    assert(!denied_arm.app.snapshot().armed && denied_arm.pwm.writes==0);
    Rig bench_arm;
    assert(bench_arm.app.execute(parseCommand("arm")).ok);
    assert(bench_arm.pwm.writes==0 && !bench_arm.app.snapshot().motion_pending);

    // Atomic live pose updates never arm implicitly. OFF accepts a valid tuple
    // into RAM with zero writes; invalid/hard-bound tuples change neither gait
    // nor profile state.
    Rig live_off;
    const int live_off_writes=live_off.pwm.writes;
    assert(live_off.app.execute(parseCommand("pose_live_set 108 -4 0 4")).ok);
    assert(!live_off.app.snapshot().armed&&live_off.pwm.writes==live_off_writes);
    assert(std::fabs(live_off.app.snapshot().motion_parameters.stand_down_mm-108)<0.001f);
    assert(std::fabs(live_off.app.snapshot().advanced_parameters.body.pitch_deg+4)<0.001f);
    assert(std::fabs(live_off.app.snapshot().advanced_parameters.body.body_x_mm-4)<0.001f);
    assert(live_off.app.execute(parseCommand("pose_live_set 108 0 0 -8")).ok);
    assert(std::fabs(live_off.app.snapshot().advanced_parameters.body.body_x_mm+8)<0.001f);
    assert(live_off.app.execute(parseCommand("pose_live_set 108 0 0 8")).ok);
    assert(std::fabs(live_off.app.snapshot().advanced_parameters.body.body_x_mm-8)<0.001f);
    assert(live_off.app.execute(parseCommand("pose_live_set 108 0 7 4")).ok);
    const DebugSnapshot live_before_reject=live_off.app.snapshot();
    for(const char* invalid : {"pose_live_set 114 -7 10 8","pose_live_set 108 -7.1 0 8",
                               "pose_live_set 108 5.1 0 8","pose_live_set 108 0 10.1 8",
                               "pose_live_set 108 0 0 -8.1","pose_live_set 108 0 0 8.1"})
        assert(!live_off.app.execute(parseCommand(invalid)).ok);
    const DebugSnapshot live_after_reject=live_off.app.snapshot();
    assert(std::fabs(live_after_reject.motion_parameters.stand_down_mm-
                     live_before_reject.motion_parameters.stand_down_mm)<0.001f);
    assert(std::fabs(live_after_reject.advanced_parameters.body.pitch_deg-
                     live_before_reject.advanced_parameters.body.pitch_deg)<0.001f);
    assert(std::fabs(live_after_reject.advanced_parameters.body.roll_deg-
                     live_before_reject.advanced_parameters.body.roll_deg)<0.001f);
    assert(std::fabs(live_after_reject.advanced_parameters.body.body_x_mm-
                     live_before_reject.advanced_parameters.body.body_x_mm)<0.001f);
    assert(live_after_reject.motion_parameter_revision==live_before_reject.motion_parameter_revision);
    assert(live_after_reject.advanced_parameter_revision==live_before_reject.advanced_parameter_revision);

    // Stable Stand/Hold live updates atomically commit then retarget Stand via
    // the existing smooth transition, gated on a fresh IMU sample.
    Rig live_stand;
    startAndTransition(live_stand,MotionMode::Stand);
    const int live_stand_writes=live_stand.pwm.writes;
    assert(live_stand.app.execute(parseCommand("pose_live_set 113 1 1 6")).ok);
    assert(live_stand.app.snapshot().motion_pending&&!live_stand.app.snapshot().motion_transition);
    assert(live_stand.pwm.writes==live_stand_writes);
    live_stand.advance(10);
    assert(live_stand.app.snapshot().motion_transition);
    for(int i=0;i<75;++i)live_stand.advance(20);
    assert(!live_stand.app.snapshot().motion_transition);
    assert(std::fabs(live_stand.app.snapshot().motion_parameters.stand_down_mm-113)<0.001f);
    assert(std::fabs(live_stand.app.snapshot().advanced_parameters.body.body_x_mm-6)<0.001f);
    assert(live_stand.app.execute(parseCommand("motion_stop")).ok);
    assert(live_stand.app.execute(parseCommand("pose_live_set 112.5 -1 1 5")).ok);
    assert(live_stand.app.snapshot().motion_pending);

    // Diagnostics count successful coordinated frames and scheduler gaps.
    Rig diagnostics;
    startAndTransition(diagnostics,MotionMode::Trot);
    const std::uint32_t frames=diagnostics.app.snapshot().motion_frame_count;
    diagnostics.advance(60);
    const DebugSnapshot ds=diagnostics.app.snapshot();
    assert(ds.motion_frame_count==frames+1);
    assert(ds.motion_last_update_gap_ms==60 && ds.motion_max_update_gap_ms>=60);
    assert(ds.motion_missed_deadlines>=2);

    // Body pose/CG parameters are separate RAM-only settings. Applying them in
    // Hold is zero-write; CG Walk executes the opt-in multi-stage model for a
    // full 3.2 s cycle under the shared coordinated-speed guard.
    Rig body_hold;
    startAndTransition(body_hold,MotionMode::Stand);
    assert(body_hold.app.execute(parseCommand("motion_stop")).ok);
    const int body_hold_writes=body_hold.pwm.writes;
    assert(body_hold.app.execute(parseCommand("body_set 1 0 2 0 2.5")).ok);
    assert(body_hold.pwm.writes==body_hold_writes);
    assert(std::fabs(body_hold.app.snapshot().advanced_parameters.body.pitch_deg-1)<0.001f);
    assert(!body_hold.app.execute(parseCommand("body_set 4 0 0 0 2.5")).ok);
    assert(body_hold.pwm.writes==body_hold_writes);

    Rig cg_runtime;
    assert(cg_runtime.app.execute(parseCommand("motion_set 6 2 3.2 108 0 2")).ok);
    assert(cg_runtime.app.execute(parseCommand("body_set 0 0 0 1 2.5")).ok);
    assert(cg_runtime.app.execute(parseCommand("arm")).ok);
    assert(cg_runtime.app.execute(parseCommand("walk")).ok);
    assert(cg_runtime.pwm.writes==0);
    assert(finishPreflight(cg_runtime)>1);
    cg_runtime.advance(10);
    for(int i=0;i<75;++i)cg_runtime.advance(20);
    const int cg_before=cg_runtime.pwm.writes;
    for(int i=0;i<160;++i)cg_runtime.advance(20);
    assert(cg_runtime.pwm.writes==cg_before+160*8);
    assert(std::string(cg_runtime.app.snapshot().motion_mode)=="walk");
    assert(cg_runtime.app.snapshot().advanced_parameters.cg_walk_enabled);

    // Static balance requires an explicit 20-sample reference and enable.
    // Waiting five seconds after capture is safe because balance_on rebases
    // the derivative time while preserving the captured angle reference.
    Rig balance;
    assert(balance.app.execute(parseCommand("balance_roll_sign 0")).ok);
    startAndTransition(balance,MotionMode::Stand);
    for(int i=0;i<30;++i)balance.advance(10);
    assert(balance.app.snapshot().attitude_ok);
    const int before_capture=balance.pwm.writes;
    assert(balance.app.execute(parseCommand("balance_capture")).ok);
    assert(balance.pwm.writes==before_capture);
    for(int i=0;i<20;++i)balance.advance(10);
    assert(balance.app.snapshot().balance_reference_ready);
    for(int i=0;i<500;++i)balance.advance(10);
    const int before_enable=balance.pwm.writes;
    assert(balance.app.execute(parseCommand("balance_on")).ok);
    assert(balance.pwm.writes==before_enable);
    balance.advance(10);
    assert(balance.app.snapshot().balance_enabled && balance.pwm.writes==before_enable);
    balance.imu.sensor_pitch_deg=-2.0f; // user's lift-nose sensor sign
    for(int i=0;i<40;++i)balance.advance(10);
    DebugSnapshot balanced=balance.app.snapshot();
    assert(balanced.balance_enabled && balanced.balance_pitch_correction_deg<0);
    assert(balanced.balance_roll_sensor_sign==0 &&
           std::fabs(balanced.balance_roll_correction_deg)<0.0001f);
    assert(balance.pwm.writes>before_enable); // truly enabled, non-zero eight-frame response
    const int before_balance_stop=balance.pwm.writes;
    assert(balance.app.execute(parseCommand("motion_stop")).ok);
    assert(balance.pwm.writes==before_balance_stop && !balance.app.snapshot().balance_enabled);
    for(int i=0;i<20;++i)balance.advance(20);
    assert(balance.pwm.writes==before_balance_stop);

    // Default -1 path needs no sign command. Sensor Roll positive means body
    // left side is low (negative body Roll), so correction must be positive.
    Rig default_roll;
    startAndTransition(default_roll,MotionMode::Stand);
    for(int i=0;i<30;++i)default_roll.advance(10);
    assert(default_roll.app.execute(parseCommand("balance_capture")).ok);
    for(int i=0;i<20;++i)default_roll.advance(10);
    assert(default_roll.app.execute(parseCommand("balance_on")).ok);
    default_roll.advance(10);
    default_roll.imu.sensor_pitch_deg=-1.5f;
    default_roll.imu.sensor_roll_deg=1.5f;
    for(int i=0;i<50;++i)default_roll.advance(20);
    DebugSnapshot default_active=default_roll.app.snapshot();
    assert(default_active.balance_enabled && default_active.balance_roll_sensor_sign==-1);
    assert(default_active.balance_pitch_correction_deg<0 &&
           default_active.balance_roll_correction_deg>0);
    default_roll.imu.sensor_pitch_deg=0;
    default_roll.imu.sensor_roll_deg=-1.5f;
    for(int i=0;i<80;++i)default_roll.advance(20);
    assert(default_roll.app.snapshot().balance_roll_correction_deg<0);
    assert(default_roll.app.execute(parseCommand("balance_off")).ok);
    assert(default_roll.app.snapshot().balance_reference_ready);
    assert(default_roll.app.execute(parseCommand("balance_roll_sign 0")).ok);
    assert(!default_roll.app.snapshot().balance_reference_ready &&
           default_roll.app.snapshot().balance_roll_sensor_sign==0);
    assert(default_roll.app.execute(parseCommand("off")).ok);
    assert(!default_roll.app.snapshot().armed && !default_roll.app.snapshot().balance_enabled);
    Rig default_roll_reboot;
    assert(default_roll_reboot.app.snapshot().balance_roll_sensor_sign==-1 &&
           !default_roll_reboot.app.snapshot().balance_enabled &&
           !default_roll_reboot.app.snapshot().balance_reference_ready &&
           default_roll_reboot.pwm.writes==0);

    // Both confirmed Roll sign choices run the second axis. Positive body
    // roll (left side high) receives a negative correction in either mapping.
    for(int sign : {1,-1}) {
        Rig roll;
        const char* sign_command=sign>0?"balance_roll_sign +1":"balance_roll_sign -1";
        assert(roll.app.execute(parseCommand(sign_command)).ok);
        startAndTransition(roll,MotionMode::Stand);
        for(int i=0;i<30;++i)roll.advance(10);
        assert(roll.app.execute(parseCommand("balance_capture")).ok);
        for(int i=0;i<20;++i)roll.advance(10);
        assert(roll.app.snapshot().balance_reference_ready);
        assert(roll.app.execute(parseCommand("balance_on")).ok);
        roll.advance(10);
        roll.imu.sensor_pitch_deg=-1.5f;
        roll.imu.sensor_roll_deg=sign>0?1.5f:-1.5f;
        for(int i=0;i<50;++i)roll.advance(20);
        const DebugSnapshot rs=roll.app.snapshot();
        assert(rs.balance_enabled && rs.balance_pitch_correction_deg<0 &&
               rs.balance_roll_correction_deg<0);
    }

    // A long attitude sampling gap changes the estimator reference and forces
    // a deterministic global-off state rather than using stale correction.
    Rig reset_guard;
    startAndTransition(reset_guard,MotionMode::Stand);
    for(int i=0;i<30;++i)reset_guard.advance(10);
    assert(reset_guard.app.execute(parseCommand("balance_capture")).ok);
    for(int i=0;i<20;++i)reset_guard.advance(10);
    assert(reset_guard.app.execute(parseCommand("balance_on")).ok);
    reset_guard.advance(10);
    const int off_before_reset=reset_guard.pwm.offs;
    reset_guard.advance(120);
    assert(!reset_guard.app.snapshot().armed &&
           !reset_guard.app.snapshot().balance_reference_ready);
    assert(reset_guard.pwm.offs==off_before_reset+1);

    // balance_off is a priority freeze even if another motion request is
    // pending or transitioning; no later tick may resurrect that plan.
    Rig balance_off_pending;
    assert(balance_off_pending.app.execute(parseCommand("arm_stand")).ok);
    assert(balance_off_pending.app.execute(parseCommand("balance_off")).ok);
    for(int i=0;i<20;++i)balance_off_pending.advance(20);
    assert(balance_off_pending.pwm.writes==0 &&
           !balance_off_pending.app.snapshot().motion_pending);
    Rig balance_off_transition;
    assert(balance_off_transition.app.execute(parseCommand("arm_stand")).ok);
    balance_off_transition.advance(10);
    const int transition_before_off=balance_off_transition.pwm.writes;
    assert(balance_off_transition.app.snapshot().motion_transition);
    assert(balance_off_transition.app.execute(parseCommand("balance_off")).ok);
    for(int i=0;i<20;++i)balance_off_transition.advance(20);
    assert(balance_off_transition.pwm.writes==transition_before_off);
    Rig balance_off_switch;
    startAndTransition(balance_off_switch,MotionMode::Stand);
    assert(balance_off_switch.app.execute(parseCommand("trot")).ok);
    const int switch_before_off=balance_off_switch.pwm.writes;
    assert(balance_off_switch.app.execute(parseCommand("balance_off")).ok);
    for(int i=0;i<20;++i)balance_off_switch.advance(20);
    assert(balance_off_switch.pwm.writes==switch_before_off);

    // Reference capture rejects continuous rotation and estimator reference
    // resets; configuration changes invalidate an old reference.
    Rig moving_capture;
    startAndTransition(moving_capture,MotionMode::Stand);
    for(int i=0;i<30;++i)moving_capture.advance(10);
    assert(moving_capture.app.execute(parseCommand("balance_capture")).ok);
    moving_capture.imu.sensor_pitch_rate_deg_s=-20;
    for(int i=0;i<25;++i) {
        moving_capture.imu.sensor_pitch_deg-=0.2f;
        moving_capture.advance(10);
    }
    assert(!moving_capture.app.snapshot().balance_reference_ready);
    moving_capture.imu.sensor_pitch_rate_deg_s=0;
    assert(moving_capture.app.execute(parseCommand("balance_off")).ok);
    assert(moving_capture.app.execute(parseCommand("stand")).ok);
    moving_capture.advance(10);for(int i=0;i<75;++i)moving_capture.advance(20);
    for(int i=0;i<30;++i)moving_capture.advance(10);
    assert(moving_capture.app.execute(parseCommand("balance_capture")).ok);
    moving_capture.advance(10);
    moving_capture.advance(120);
    assert(!moving_capture.app.snapshot().balance_capture_pending &&
           !moving_capture.app.snapshot().balance_reference_ready);

    Rig stale_parameters;
    startAndTransition(stale_parameters,MotionMode::Stand);
    for(int i=0;i<30;++i)stale_parameters.advance(10);
    assert(stale_parameters.app.execute(parseCommand("balance_capture")).ok);
    for(int i=0;i<20;++i)stale_parameters.advance(10);
    assert(stale_parameters.app.snapshot().balance_reference_ready);
    assert(stale_parameters.app.execute(parseCommand("body_set 1 0 0 0 2.5")).ok);
    assert(!stale_parameters.app.snapshot().balance_reference_ready);
    assert(!stale_parameters.app.execute(parseCommand("balance_capture")).ok);
    assert(!stale_parameters.app.execute(parseCommand("balance_on")).ok);

    // A provisional drive lease is visible immediately so the browser can
    // renew it, but release/expiry during checking cancels the job forever.
    Rig provisional_release;startAndTransition(provisional_release,MotionMode::Stand);
    std::uint32_t provisional_expected=provisional_release.app.snapshot().drive_epoch;
    char provisional_cmd[96];std::snprintf(provisional_cmd,sizeof(provisional_cmd),
        "drive_begin %lu 1 trot 1 0 0",static_cast<unsigned long>(provisional_expected));
    const int provisional_writes=provisional_release.pwm.writes;
    assert(provisional_release.app.execute(parseCommand(provisional_cmd)).ok);
    const std::uint32_t provisional_epoch=provisional_release.app.snapshot().drive_epoch;
    assert(provisional_release.app.snapshot().preview_pending&&
           provisional_release.app.snapshot().drive_session_active);
    std::snprintf(provisional_cmd,sizeof(provisional_cmd),"drive_end %lu 2",
        static_cast<unsigned long>(provisional_epoch));
    assert(provisional_release.app.execute(parseCommand(provisional_cmd)).ok);
    for(int i=0;i<400;++i)provisional_release.advance(1);
    assert(!provisional_release.app.snapshot().preview_pending&&
           !provisional_release.app.snapshot().drive_session_active&&
           provisional_release.pwm.writes==provisional_writes);

    Rig provisional_expiry;startAndTransition(provisional_expiry,MotionMode::Stand);
    provisional_expected=provisional_expiry.app.snapshot().drive_epoch;
    std::snprintf(provisional_cmd,sizeof(provisional_cmd),"drive_begin %lu 1 trot 1 0 0",
        static_cast<unsigned long>(provisional_expected));
    const int provisional_expiry_writes=provisional_expiry.pwm.writes;
    assert(provisional_expiry.app.execute(parseCommand(provisional_cmd)).ok);
    provisional_expiry.clock.synthetic_work_remaining_ms=1000;
    provisional_expiry.clock.synthetic_cost_per_read_ms=3;
    for(int i=0;i<400&&provisional_expiry.app.snapshot().preview_pending;++i)
        provisional_expiry.advance(0);
    const DebugSnapshot expired_check=provisional_expiry.app.snapshot();
    assert(!expired_check.preview_pending&&!expired_check.drive_session_active&&
           std::string(expired_check.preview_result)=="cancelled_lease_expired"&&
           provisional_expiry.pwm.writes==provisional_expiry_writes);

    Rig provisional_speed_allowed;startAndTransition(provisional_speed_allowed,MotionMode::Stand);
    provisional_expected=provisional_speed_allowed.app.snapshot().drive_epoch;
    std::snprintf(provisional_cmd,sizeof(provisional_cmd),"drive_begin %lu 1 walk 1 0 0",
        static_cast<unsigned long>(provisional_expected));
    const int provisional_speed_writes=provisional_speed_allowed.pwm.writes;
    assert(provisional_speed_allowed.app.execute(parseCommand(provisional_cmd)).ok);
    assert(finishPreflight(provisional_speed_allowed)<500);
    const DebugSnapshot allowed_drive=provisional_speed_allowed.app.snapshot();
    assert(allowed_drive.drive_session_active&&
           std::string(allowed_drive.drive_session_reason)=="active"&&
           std::string(allowed_drive.preview_result)=="passed"&&
           allowed_drive.motion_pending&&
           std::string(allowed_drive.motion_mode)=="stand"&&
           provisional_speed_allowed.pwm.writes==provisional_speed_writes);

    // Remote drive intent reaches the real RobotApp/fake-PWM path for forward,
    // reverse, differential left/right and stepping. Renew only extends the
    // lease: it never emits PWM, resets phase, or restarts transition.
    const char* intents[5]={"1 0 0","-1 0 0","0 1 0","0 -1 0","0 0 1"};
    for(int intent_index=0;intent_index<5;++intent_index){
        const char* intent=intents[intent_index];
        Rig remote;startAndTransition(remote,MotionMode::Stand);
        std::uint32_t epoch=remote.app.snapshot().drive_epoch,seq=1;
        char command[96];std::snprintf(command,sizeof(command),"drive_begin %lu %lu trot %s",
            static_cast<unsigned long>(epoch),static_cast<unsigned long>(seq),intent);
        const int begin_writes=remote.pwm.writes;
        assert(remote.app.execute(parseCommand(command)).ok);
        DebugSnapshot begun=remote.app.snapshot();epoch=begun.drive_epoch;
        assert(begun.drive_session_active&&begun.preview_pending&&
               remote.pwm.writes==begin_writes);
        int checking_ticks=0;
        while(remote.app.snapshot().preview_pending&&checking_ticks<1000){
            if(checking_ticks&&checking_ticks%100==0){
                std::snprintf(command,sizeof(command),"drive_renew %lu %lu",
                    static_cast<unsigned long>(epoch),static_cast<unsigned long>(++seq));
                assert(remote.app.execute(parseCommand(command)).ok);
            }
            remote.advance(1);++checking_ticks;
        }
        assert(checking_ticks>1&&!remote.app.snapshot().preview_pending&&
               std::string(remote.app.snapshot().preview_result)=="passed"&&
               remote.pwm.writes==begin_writes);
        remote.advance(10);
        for(int i=0;i<75;++i){
            if(i%10==0){std::snprintf(command,sizeof(command),"drive_renew %lu %lu",
                static_cast<unsigned long>(epoch),static_cast<unsigned long>(++seq));
                const int before=remote.pwm.writes;assert(remote.app.execute(parseCommand(command)).ok);
                assert(remote.pwm.writes==before);}
            remote.advance(20);
        }
        float min_forward=1.0e9f,max_forward=-1.0e9f,min_down=1.0e9f,max_down=-1.0e9f;
        for(int i=0;i<60;++i){
            if(i%10==0){std::snprintf(command,sizeof(command),"drive_renew %lu %lu",
                static_cast<unsigned long>(epoch),static_cast<unsigned long>(++seq));
                const float phase_before=remote.app.snapshot().motion_phase;
                const int before=remote.pwm.writes;assert(remote.app.execute(parseCommand(command)).ok);
                assert(remote.pwm.writes==before&&
                       std::fabs(remote.app.snapshot().motion_phase-phase_before)<0.001f);}
            remote.advance(20);const DebugSnapshot s=remote.app.snapshot();
            min_forward=std::fmin(min_forward,s.target_forward_mm[0]);
            max_forward=std::fmax(max_forward,s.target_forward_mm[0]);
            min_down=std::fmin(min_down,s.target_down_mm[0]);
            max_down=std::fmax(max_down,s.target_down_mm[0]);
        }
        assert(remote.app.snapshot().drive_session_active);
        if(intent_index==4)assert(max_forward-min_forward<0.05f&&max_down-min_down>0.1f);
        else assert(max_forward-min_forward>0.1f);
        std::snprintf(command,sizeof(command),"drive_end %lu %lu",
            static_cast<unsigned long>(epoch),static_cast<unsigned long>(++seq));
        const int before_end=remote.pwm.writes;assert(remote.app.execute(parseCommand(command)).ok);
        assert(remote.pwm.writes==before_end&&!remote.app.snapshot().drive_session_active);
        for(int i=0;i<10;++i)remote.advance(20);
        assert(remote.pwm.writes==before_end);
    }

    // A renew received at/after expiry latches the expiry event. The next app
    // tick freezes the last complete frame; the stale packet cannot revive it.
    Rig lease;startAndTransition(lease,MotionMode::Stand);
    std::uint32_t lease_expected=lease.app.snapshot().drive_epoch;
    char lease_cmd[96];std::snprintf(lease_cmd,sizeof(lease_cmd),"drive_begin %lu 1 trot 1 0 0",
        static_cast<unsigned long>(lease_expected));
    assert(lease.app.execute(parseCommand(lease_cmd)).ok);
    const std::uint32_t lease_epoch=lease.app.snapshot().drive_epoch;
    assert(finishPreflight(lease)<500);
    lease.advance(10);std::uint32_t lease_seq=1;
    for(int i=0;i<75;++i){if(i%10==0){std::snprintf(lease_cmd,sizeof(lease_cmd),"drive_renew %lu %lu",
        static_cast<unsigned long>(lease_epoch),static_cast<unsigned long>(++lease_seq));
        assert(lease.app.execute(parseCommand(lease_cmd)).ok);}lease.advance(20);}
    const int lease_writes=lease.pwm.writes;lease.clock.now+=500;
    std::snprintf(lease_cmd,sizeof(lease_cmd),"drive_renew %lu %lu",
        static_cast<unsigned long>(lease_epoch),static_cast<unsigned long>(++lease_seq));
    assert(!lease.app.execute(parseCommand(lease_cmd)).ok&&lease.pwm.writes==lease_writes);
    const std::uint32_t latched_epoch=lease.app.snapshot().drive_epoch;
    std::snprintf(lease_cmd,sizeof(lease_cmd),"drive_begin %lu 99 trot 1 0 0",
        static_cast<unsigned long>(latched_epoch));
    assert(!lease.app.execute(parseCommand(lease_cmd)).ok&&lease.pwm.writes==lease_writes);
    lease.app.tick();assert(lease.pwm.writes==lease_writes&&!lease.app.snapshot().drive_session_active&&
        std::string(lease.app.snapshot().motion_mode)=="hold");
    for(int i=0;i<10;++i)lease.advance(20);
    assert(lease.pwm.writes==lease_writes);

    // Manual switching to the formerly speed-limited built-in Walk now passes;
    // the explicit manual command ends the remote session and queues Walk.
    Rig preserve;startAndTransition(preserve,MotionMode::Stand);
    std::uint32_t preserve_expected=preserve.app.snapshot().drive_epoch;
    char preserve_cmd[96];std::snprintf(preserve_cmd,sizeof(preserve_cmd),
        "drive_begin %lu 1 trot -1 0 0",static_cast<unsigned long>(preserve_expected));
    assert(preserve.app.execute(parseCommand(preserve_cmd)).ok);
    const std::uint32_t preserve_epoch=preserve.app.snapshot().drive_epoch;
    assert(finishPreflight(preserve)<500);
    std::snprintf(preserve_cmd,sizeof(preserve_cmd),"drive_renew %lu 2",
        static_cast<unsigned long>(preserve_epoch));
    assert(preserve.app.execute(parseCommand(preserve_cmd)).ok);
    const int preserve_writes=preserve.pwm.writes;
    assert(preserve.app.execute(parseCommand("walk")).ok);
    assert(finishPreflight(preserve)<500);
    const DebugSnapshot preserved=preserve.app.snapshot();
    assert(!preserved.drive_session_active);
    assert(preserved.drive_intent.forward>0);
    assert(preserved.motion_pending||preserved.motion_transition||
           std::string(preserved.motion_mode)=="walk");
    assert(std::string(preserved.preview_result)=="passed");
    assert(std::string(preserved.drive_session_reason)=="manual_walk");
    assert(preserve.pwm.writes>=preserve_writes);

    // Stale begin is rejected before it can cancel static feedback or mutate
    // the existing mode/intent.
    Rig stale_begin;prepareReference(stale_begin);
    assert(stale_begin.app.execute(parseCommand("balance_on")).ok);stale_begin.advance(10);
    const DebugSnapshot static_before=stale_begin.app.snapshot();
    char stale_cmd[96];std::snprintf(stale_cmd,sizeof(stale_cmd),
        "drive_begin %lu 1 trot 1 0 0",static_cast<unsigned long>(static_before.drive_epoch+1));
    const int stale_writes=stale_begin.pwm.writes;
    assert(!stale_begin.app.execute(parseCommand(stale_cmd)).ok);
    assert(stale_begin.app.snapshot().balance_enabled&&stale_begin.pwm.writes==stale_writes);

    // Moving CG feedback: enable in Stand is zero-write, waiting five seconds
    // does not poison the derivative because transition completion rebases it.
    Rig moving;assert(moving.app.execute(parseCommand("motion_set 6 2 3.2 108 0 2")).ok);
    assert(moving.app.execute(parseCommand("body_set 0 0 0 1 2.5")).ok);
    prepareReference(moving);
    const int moving_on_writes=moving.pwm.writes;
    assert(moving.app.execute(parseCommand("moving_cg_on")).ok);
    assert(moving.pwm.writes==moving_on_writes);moving.advance(10);
    assert(moving.app.snapshot().moving_cg_enabled);
    for(int i=0;i<500;++i)moving.advance(10);
    std::uint32_t moving_expected=moving.app.snapshot().drive_epoch;
    char moving_cmd[96];std::snprintf(moving_cmd,sizeof(moving_cmd),
        "drive_begin %lu 1 walk 1 0 0",static_cast<unsigned long>(moving_expected));
    assert(moving.app.execute(parseCommand(moving_cmd)).ok);
    const std::uint32_t moving_epoch=moving.app.snapshot().drive_epoch;std::uint32_t moving_seq=1;
    int moving_check_ticks=0;
    while(moving.app.snapshot().preview_pending&&moving_check_ticks<6000){
        if(moving_check_ticks&&moving_check_ticks%100==0){
            std::snprintf(moving_cmd,sizeof(moving_cmd),"drive_renew %lu %lu",
                static_cast<unsigned long>(moving_epoch),static_cast<unsigned long>(++moving_seq));
            assert(moving.app.execute(parseCommand(moving_cmd)).ok);
        }
        moving.advance(1);++moving_check_ticks;
    }
    assert(moving_check_ticks>1&&!moving.app.snapshot().preview_pending&&
           std::string(moving.app.snapshot().preview_result)=="passed");
    moving.advance(10);
    for(int i=0;i<75;++i){if(i%10==0){std::snprintf(moving_cmd,sizeof(moving_cmd),
        "drive_renew %lu %lu",static_cast<unsigned long>(moving_epoch),
        static_cast<unsigned long>(++moving_seq));assert(moving.app.execute(parseCommand(moving_cmd)).ok);}
        moving.advance(20);}
    moving.imu.sensor_pitch_deg=-1.5f;bool saw_wait=false;
    for(int i=0;i<260;++i){if(i%10==0){std::snprintf(moving_cmd,sizeof(moving_cmd),
        "drive_renew %lu %lu",static_cast<unsigned long>(moving_epoch),
        static_cast<unsigned long>(++moving_seq));assert(moving.app.execute(parseCommand(moving_cmd)).ok);}
        moving.advance(20);saw_wait=saw_wait||moving.app.snapshot().cg_phase_waiting;}
    DebugSnapshot moving_positive=moving.app.snapshot();
    assert(moving_positive.moving_cg_enabled&&moving_positive.moving_cg_shift_mm>0&&saw_wait&&
           moving_positive.motion_actual_elapsed_s>moving_positive.motion_nominal_elapsed_s&&
           moving_positive.preflight_actual_cycle_s>3.2f);
    moving.imu.sensor_pitch_deg=1.5f;
    for(int i=0;i<160;++i){if(i%10==0){std::snprintf(moving_cmd,sizeof(moving_cmd),
        "drive_renew %lu %lu",static_cast<unsigned long>(moving_epoch),
        static_cast<unsigned long>(++moving_seq));assert(moving.app.execute(parseCommand(moving_cmd)).ok);}
        moving.advance(20);}
    assert(moving.app.snapshot().moving_cg_shift_mm<0);
    const int moving_off_writes=moving.pwm.writes;
    assert(moving.app.execute(parseCommand("moving_cg_off")).ok);
    assert(moving.pwm.writes==moving_off_writes&&!moving.app.snapshot().moving_cg_enabled&&
           std::string(moving.app.snapshot().motion_mode)=="hold");

    Rig invalid_before_drive;prepareReference(invalid_before_drive);
    assert(invalid_before_drive.app.execute(parseCommand("moving_cg_on")).ok);
    invalid_before_drive.advance(10);
    invalid_before_drive.imu.nonfinite=true;invalid_before_drive.advance(10);
    const int invalid_before_writes=invalid_before_drive.pwm.writes;
    std::uint32_t invalid_epoch=invalid_before_drive.app.snapshot().drive_epoch;
    char invalid_cmd[96];std::snprintf(invalid_cmd,sizeof(invalid_cmd),
        "drive_begin %lu 1 trot 1 0 0",static_cast<unsigned long>(invalid_epoch));
    assert(!invalid_before_drive.app.execute(parseCommand(invalid_cmd)).ok);
    assert(invalid_before_drive.pwm.writes==invalid_before_writes&&
           !invalid_before_drive.app.snapshot().motion_pending&&
           !invalid_before_drive.app.snapshot().moving_cg_enabled);

    Rig invalid_pending;prepareReference(invalid_pending);
    assert(invalid_pending.app.execute(parseCommand("moving_cg_on")).ok);invalid_pending.advance(10);
    std::uint32_t pending_epoch=invalid_pending.app.snapshot().drive_epoch;
    std::snprintf(invalid_cmd,sizeof(invalid_cmd),"drive_begin %lu 1 trot 1 0 0",
        static_cast<unsigned long>(pending_epoch));
    assert(invalid_pending.app.execute(parseCommand(invalid_cmd)).ok);
    const int invalid_pending_writes=invalid_pending.pwm.writes;
    invalid_pending.imu.nonfinite=true;invalid_pending.advance(10);
    const DebugSnapshot invalid_pending_state=invalid_pending.app.snapshot();
    assert(invalid_pending.pwm.writes==invalid_pending_writes);
    assert(!invalid_pending_state.motion_pending);
    assert(!invalid_pending_state.moving_cg_enabled);
    assert(!invalid_pending_state.preview_pending&&
           std::string(invalid_pending_state.preview_result)=="cancelled_attitude_reset"&&
           std::string(invalid_pending_state.preview_error)=="reference_invalid");

    Rig invalid_running;prepareReference(invalid_running);
    assert(invalid_running.app.execute(parseCommand("moving_cg_on")).ok);invalid_running.advance(10);
    std::uint32_t running_epoch=invalid_running.app.snapshot().drive_epoch;
    std::snprintf(invalid_cmd,sizeof(invalid_cmd),"drive_begin %lu 1 trot 1 0 0",
        static_cast<unsigned long>(running_epoch));
    assert(invalid_running.app.execute(parseCommand(invalid_cmd)).ok);
    running_epoch=invalid_running.app.snapshot().drive_epoch;std::uint32_t running_seq=1;
    int invalid_check_ticks=0;
    while(invalid_running.app.snapshot().preview_pending&&invalid_check_ticks<3000){
        if(invalid_check_ticks&&invalid_check_ticks%100==0){
            std::snprintf(invalid_cmd,sizeof(invalid_cmd),"drive_renew %lu %lu",
                static_cast<unsigned long>(running_epoch),static_cast<unsigned long>(++running_seq));
            assert(invalid_running.app.execute(parseCommand(invalid_cmd)).ok);
        }
        invalid_running.advance(1);++invalid_check_ticks;
    }
    assert(!invalid_running.app.snapshot().preview_pending&&
           std::string(invalid_running.app.snapshot().preview_result)=="passed");
    invalid_running.advance(10);
    for(int i=0;i<75;++i){if(i%10==0){std::snprintf(invalid_cmd,sizeof(invalid_cmd),
        "drive_renew %lu %lu",static_cast<unsigned long>(running_epoch),
        static_cast<unsigned long>(++running_seq));assert(invalid_running.app.execute(parseCommand(invalid_cmd)).ok);}
        invalid_running.advance(20);}
    invalid_running.imu.sensor_pitch_deg=-1.5f;
    for(int i=0;i<40;++i){if(i%10==0){std::snprintf(invalid_cmd,sizeof(invalid_cmd),
        "drive_renew %lu %lu",static_cast<unsigned long>(running_epoch),
        static_cast<unsigned long>(++running_seq));assert(invalid_running.app.execute(parseCommand(invalid_cmd)).ok);}
        invalid_running.advance(20);}
    assert(invalid_running.app.snapshot().moving_cg_shift_mm>0);
    invalid_running.imu.nonfinite=true;invalid_running.advance(10);
    const int invalid_running_writes=invalid_running.pwm.writes;
    assert(!invalid_running.app.execute(parseCommand("trot")).ok);
    assert(invalid_running.pwm.writes==invalid_running_writes&&
           std::string(invalid_running.app.snapshot().motion_mode)=="hold_error"&&
           !invalid_running.app.snapshot().drive_session_active);
    invalid_running.advance(10);assert(invalid_running.pwm.writes==invalid_running_writes);

    // Every "stop and hold" alias stops both feedback writers. In particular,
    // moving_cg_off while static balance is active must not let balanceTick
    // observe Hold and turn servos globally off on the next tick.
    Rig stop_alias;prepareReference(stop_alias);
    assert(stop_alias.app.execute(parseCommand("balance_on")).ok);stop_alias.advance(10);
    const int alias_writes=stop_alias.pwm.writes,alias_offs=stop_alias.pwm.offs;
    assert(stop_alias.app.execute(parseCommand("moving_cg_off")).ok);
    assert(stop_alias.pwm.writes==alias_writes&&stop_alias.pwm.offs==alias_offs&&
           stop_alias.app.snapshot().armed&&!stop_alias.app.snapshot().balance_enabled&&
           std::string(stop_alias.app.snapshot().motion_mode)=="hold");
    for(int i=0;i<20;++i)stop_alias.advance(20);
    assert(stop_alias.pwm.writes==alias_writes&&stop_alias.pwm.offs==alias_offs);

    Rig shared_reference;prepareReference(shared_reference);
    assert(shared_reference.app.execute(parseCommand("moving_cg_on")).ok);shared_reference.advance(10);
    const int shared_writes=shared_reference.pwm.writes;
    assert(!shared_reference.app.execute(parseCommand("balance_capture")).ok);
    assert(shared_reference.pwm.writes==shared_writes&&
           shared_reference.app.snapshot().moving_cg_enabled&&
           shared_reference.app.snapshot().balance_reference_ready);
    assert(shared_reference.app.execute(parseCommand("moving_cg_off")).ok);

    Rig moving_pending_stop;prepareReference(moving_pending_stop);
    const int pending_stop_writes=moving_pending_stop.pwm.writes;
    assert(moving_pending_stop.app.execute(parseCommand("moving_cg_on")).ok);
    assert(moving_pending_stop.app.execute(parseCommand("moving_cg_off")).ok);
    for(int i=0;i<20;++i)moving_pending_stop.advance(20);
    assert(moving_pending_stop.pwm.writes==pending_stop_writes&&
           !moving_pending_stop.app.snapshot().moving_cg_enabled&&
           !moving_pending_stop.app.snapshot().moving_cg_enable_pending&&
           moving_pending_stop.app.snapshot().armed);
}
