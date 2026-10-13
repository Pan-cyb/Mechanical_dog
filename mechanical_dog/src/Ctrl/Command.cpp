#include "Ctrl/Command.h"

#include <cctype>
#include <cstring>
#include <cmath>
#include <cstdlib>

namespace dog {
namespace {
const char* skipSpace(const char* text) {
    while (*text == ' ' || *text == '\t') ++text;
    return text;
}

bool endOfLine(const char* text) {
    text = skipSpace(text);
    return *text == '\0' || *text == '\r' || *text == '\n';
}

bool singleWord(const char* text, const char* word) {
    const std::size_t len = std::strlen(word);
    return std::strncmp(text, word, len) == 0 && endOfLine(text + len);
}

bool number(const char*& text, unsigned& value) {
    text = skipSpace(text);
    if (!std::isdigit(static_cast<unsigned char>(*text))) return false;
    value = 0;
    while (std::isdigit(static_cast<unsigned char>(*text))) {
        value = value * 10 + static_cast<unsigned>(*text - '0');
        if (value > 65535) return false;
        ++text;
    }
    return *text == ' ' || *text == '\t' || endOfLine(text);
}
bool uint32Number(const char*& text,std::uint32_t& value){
    text=skipSpace(text);if(!std::isdigit(static_cast<unsigned char>(*text)))return false;
    std::uint64_t parsed=0;
    while(std::isdigit(static_cast<unsigned char>(*text))){
        parsed=parsed*10U+static_cast<unsigned>(*text-'0');
        if(parsed>0xFFFFFFFFULL)return false;
        ++text;
    }
    value=static_cast<std::uint32_t>(parsed);
    return *text==' '||*text=='\t'||endOfLine(text);
}

bool decimal(const char*& text, float& value) {
    text = skipSpace(text);
    char token[17] = {};
    std::size_t used = 0;
    while (*text && *text != ' ' && *text != '\t' && *text != '\r' &&
           *text != '\n' && used < sizeof(token)-1) token[used++] = *text++;
    if (used == 0 || used >= sizeof(token)-1) return false;
    const char* p = token;
    if (*p=='+' || *p=='-') ++p;
    bool digit = false;
    while (std::isdigit(static_cast<unsigned char>(*p))) { digit=true; ++p; }
    if (*p=='.') { ++p; while (std::isdigit(static_cast<unsigned char>(*p))) { digit=true; ++p; } }
    if (!digit || *p!='\0') return false;
    value = std::strtof(token, nullptr);
    return std::isfinite(value) && (*text==' ' || *text=='\t' || endOfLine(text));
}

bool motionModeWord(const char* text, MotionMode& mode) {
    if (singleWord(text,"stand")) mode=MotionMode::Stand;
    else if (singleWord(text,"walk")) mode=MotionMode::Walk;
    else if (singleWord(text,"trot")) mode=MotionMode::Trot;
    else if (singleWord(text,"leg1")) mode=MotionMode::Leg1;
    else if (singleWord(text,"leg2")) mode=MotionMode::Leg2;
    else if (singleWord(text,"leg3")) mode=MotionMode::Leg3;
    else if (singleWord(text,"leg4")) mode=MotionMode::Leg4;
    else return false;
    return true;
}

bool signedUnit(const char* text,std::int8_t& value) {
    text=skipSpace(text);
    if((text[0]=='-' && text[1]=='1' && endOfLine(text+2)) ||
       (text[0]=='+' && text[1]=='1' && endOfLine(text+2)) ||
       (text[0]=='1' && endOfLine(text+1))) { value=text[0]=='-'?-1:1; return true; }
    if(text[0]=='0' && endOfLine(text+1)) {value=0;return true;}
    return false;
}
} // namespace

bool parseServoOffset(const char* text, float& value) {
    if (!text || std::strlen(text) == 0 || std::strlen(text) > 8) return false;
    const char* cursor = text;
    if (*cursor == '-' || *cursor == '+') ++cursor;
    if (*cursor < '0' || *cursor > '9') return false;
    while (*cursor >= '0' && *cursor <= '9') ++cursor;
    if (*cursor == '.') {
        ++cursor;
        if (*cursor < '0' || *cursor > '9') return false;
        while (*cursor >= '0' && *cursor <= '9') ++cursor;
    }
    if (*cursor != '\0') return false;
    const float candidate = std::strtof(text, nullptr);
    if (!std::isfinite(candidate) || std::fabs(candidate) > 180) return false;
    value = candidate;
    return true;
}

bool parseShankAngle(const char* text, float& value) {
    float candidate = 0;
    if (!parseServoOffset(text, candidate) || candidate < 0.0f) return false;
    value = candidate;
    return true;
}

Command parseCommand(const char* input) {
    Command cmd{};
    if (input == nullptr) return cmd;
    const char* text = skipSpace(input);
    if (singleWord(text, "help")) cmd.type = CommandType::Help;
    else if (singleWord(text, "status")) cmd.type = CommandType::Status;
    else if (singleWord(text, "imu")) cmd.type = CommandType::Imu;
    else if (singleWord(text, "attitude") || singleWord(text, "rpy"))
        cmd.type = CommandType::Attitude;
    else if (singleWord(text, "arm")) cmd.type = CommandType::Arm;
    else if (singleWord(text, "arm_stand")) cmd.type = CommandType::ArmStand;
    else if (std::strncmp(text, "range", 5) == 0 &&
             (text[5] == ' ' || text[5] == '\t')) {
        text = skipSpace(text + 5);
        if (singleWord(text, "normal")) cmd.type = CommandType::RangeNormal;
        else if (singleWord(text, "extended")) cmd.type = CommandType::RangeExtended;
    }
    else if (singleWord(text, "cal_start")) cmd.type = CommandType::CalStart;
    else if (singleWord(text, "cal_save_all")) cmd.type = CommandType::CalSaveAll;
    else if (singleWord(text, "off")) cmd.type = CommandType::Off;
    else if (std::strncmp(text, "off", 3) == 0 &&
             (text[3] == ' ' || text[3] == '\t')) {
        text += 3;
        unsigned channel = 0;
        if (number(text, channel) && channel < 16 && endOfLine(text)) {
            cmd.type = CommandType::OffChannel;
            cmd.channel = static_cast<std::uint8_t>(channel);
        }
    }
    else if (singleWord(text, "keepalive")) cmd.type = CommandType::KeepAlive;
    else if (singleWord(text, "stand")) cmd.type = CommandType::Stand;
    else if (singleWord(text, "walk")) cmd.type = CommandType::Walk;
    else if (singleWord(text, "trot")) cmd.type = CommandType::Trot;
    else if (singleWord(text, "motion_stop")) cmd.type = CommandType::MotionStop;
    else if (singleWord(text, "motion_save")) cmd.type = CommandType::MotionSave;
    else if (singleWord(text, "motion_load")) cmd.type = CommandType::MotionLoad;
    else if (singleWord(text, "balance_capture")) cmd.type=CommandType::BalanceCapture;
    else if (singleWord(text, "balance_on")) cmd.type=CommandType::BalanceOn;
    else if (singleWord(text, "balance_off")) cmd.type=CommandType::BalanceOff;
    else if (singleWord(text, "moving_cg_on")) cmd.type=CommandType::MovingCgOn;
    else if (singleWord(text, "moving_cg_off")) cmd.type=CommandType::MovingCgOff;
    else if (singleWord(text, "profile_save")) cmd.type=CommandType::ProfileSave;
    else if (singleWord(text, "profile_load")) cmd.type=CommandType::ProfileLoad;
    else if (std::strncmp(text,"balance_roll_sign",17)==0 &&
             (text[17]==' ' || text[17]=='\t')) {
        text=skipSpace(text+17);
        if(signedUnit(text,cmd.axis_sign)) cmd.type=CommandType::BalanceRollSign;
    }
    else if (std::strncmp(text,"leg_test",8)==0 && (text[8]==' ' || text[8]=='\t')) {
        text += 8; unsigned leg=0;
        if (number(text,leg) && leg>=1 && leg<=4 && endOfLine(text)) {
            cmd.type=CommandType::LegTest; cmd.leg=static_cast<std::uint8_t>(leg);
        }
    }
    else if (std::strncmp(text,"motion_preview",14)==0 &&
             (text[14]==' ' || text[14]=='\t')) {
        text=skipSpace(text+14);
        if (motionModeWord(text,cmd.motion_mode)) cmd.type=CommandType::MotionPreview;
    }
    else if (std::strncmp(text,"motion_set",10)==0 &&
             (text[10]==' ' || text[10]=='\t')) {
        text += 10; bool ok=true;
        for (int i=0;i<6;++i) ok = ok && decimal(text,cmd.motion_values[i]);
        if (ok && endOfLine(text)) cmd.type=CommandType::MotionSet;
    }
    else if (std::strncmp(text,"body_set",8)==0 &&
             (text[8]==' ' || text[8]=='\t')) {
        text+=8; bool ok=true;
        for(int i=0;i<5;++i) ok=ok&&decimal(text,cmd.body_values[i]);
        if(ok&&endOfLine(text))cmd.type=CommandType::BodySet;
    }
    else if (std::strncmp(text,"control_set",11)==0&&
             (text[11]==' '||text[11]=='\t')){
        text+=11;bool ok=true;
        for(int i=0;i<11;++i)ok=ok&&decimal(text,cmd.control_values[i]);
        if(ok&&endOfLine(text))cmd.type=CommandType::ControlSet;
    }
    else if (std::strncmp(text,"trot_ff_set",11)==0&&
             (text[11]==' '||text[11]=='\t')){
        text+=11;bool ok=true;
        for(int i=0;i<5;++i)ok=ok&&decimal(text,cmd.trot_ff_values[i]);
        if(ok&&endOfLine(text))cmd.type=CommandType::TrotFeedforwardSet;
    }
    else if (std::strncmp(text,"straight_trim_set",17)==0&&
             (text[17]==' '||text[17]=='\t')){
        text+=17;
        if(decimal(text,cmd.straight_trim)&&endOfLine(text))
            cmd.type=CommandType::StraightTrimSet;
    }
    else if (std::strncmp(text,"trot_pitch_set",14)==0&&
             (text[14]==' '||text[14]=='\t')){
        text+=14;
        if(decimal(text,cmd.trot_pitch_offset_deg)&&endOfLine(text))
            cmd.type=CommandType::TrotPitchSet;
    }
    else if (std::strncmp(text,"pose_live_set",13)==0&&
             (text[13]==' '||text[13]=='\t')){
        text+=13;bool ok=true;
        for(int i=0;i<4;++i)ok=ok&&decimal(text,cmd.pose_live_values[i]);
        if(ok&&endOfLine(text))cmd.type=CommandType::PoseLiveSet;
    }
    else if (std::strncmp(text,"drive_begin",11)==0 &&
             (text[11]==' '||text[11]=='\t')) {
        text+=11;std::uint32_t epoch=0,seq=0;unsigned step=0;
        bool ok=uint32Number(text,epoch)&&uint32Number(text,seq);
        text=skipSpace(text);
        const char* mode_start=text;
        while(*text&&*text!=' '&&*text!='\t')++text;
        const std::size_t mode_len=static_cast<std::size_t>(text-mode_start);
        if(mode_len==4&&std::strncmp(mode_start,"walk",4)==0)cmd.drive_mode=MotionMode::Walk;
        else if(mode_len==4&&std::strncmp(mode_start,"trot",4)==0)cmd.drive_mode=MotionMode::Trot;
        else ok=false;
        ok=ok&&decimal(text,cmd.drive_intent.forward)&&decimal(text,cmd.drive_intent.turn)&&
           number(text,step)&&step<=1&&endOfLine(text);
        if(ok){cmd.type=CommandType::DriveBegin;cmd.epoch=epoch;cmd.seq=seq;
            cmd.drive_intent.stepping=step==1;}
    }
    else if ((std::strncmp(text,"drive_renew",11)==0||
              std::strncmp(text,"drive_end",9)==0)) {
        const bool renew=std::strncmp(text,"drive_renew",11)==0;
        const std::size_t len=renew?11:9;
        if(text[len]==' '||text[len]=='\t') {
            text+=len;std::uint32_t epoch=0,seq=0;
            if(uint32Number(text,epoch)&&uint32Number(text,seq)&&endOfLine(text)){
                cmd.type=renew?CommandType::DriveRenew:CommandType::DriveEnd;
                cmd.epoch=epoch;cmd.seq=seq;
            }
        }
    }
    else if (std::strncmp(text, "pwm", 3) == 0 &&
             (text[3] == ' ' || text[3] == '\t')) {
        text += 3;
        unsigned channel = 0;
        unsigned pulse = 0;
        if (number(text, channel) && number(text, pulse) && endOfLine(text) &&
            channel < 16 && pulse <= 65535) {
            cmd.type = CommandType::Pwm;
            cmd.channel = static_cast<std::uint8_t>(channel);
            cmd.pulse_us = static_cast<std::uint16_t>(pulse);
        }
    } else if (std::strncmp(text, "offset", 6) == 0 &&
               (text[6] == ' ' || text[6] == '\t')) {
        text += 6;
        unsigned channel = 0;
        if (number(text, channel) && channel < 16) {
            text = skipSpace(text);
            char token[9] = {};
            std::size_t used = 0;
            while (*text && *text != ' ' && *text != '\t' && *text != '\r' && *text != '\n' && used < 8)
                token[used++] = *text++;
            float offset = 0;
            if (endOfLine(text) && parseServoOffset(token, offset)) {
                cmd.type = CommandType::Offset;
                cmd.channel = static_cast<std::uint8_t>(channel);
                cmd.offset_deg = offset;
            }
        }
    } else if (std::strncmp(text, "shank", 5) == 0 &&
               (text[5] == ' ' || text[5] == '\t')) {
        text += 5;
        unsigned channel = 0;
        if (number(text, channel) && channel < 16) {
            text = skipSpace(text);
            char token[9] = {};
            std::size_t used = 0;
            while (*text && *text != ' ' && *text != '\t' && *text != '\r' && *text != '\n' && used < 8)
                token[used++] = *text++;
            float q = 0;
            if (endOfLine(text) && parseShankAngle(token, q)) {
                cmd.type = CommandType::Shank;
                cmd.channel = static_cast<std::uint8_t>(channel);
                cmd.shank_q_deg = q;
            }
        }
    } else if (std::strncmp(text, "nudge", 5) == 0 &&
               (text[5] == ' ' || text[5] == '\t')) {
        text += 5;
        unsigned channel = 0;
        if (number(text, channel) && channel < 16) {
            text = skipSpace(text);
            if ((*text == '+' || *text == '-') && text[1] == '1' &&
                endOfLine(text + 2)) {
                cmd.type = CommandType::Nudge;
                cmd.channel = static_cast<std::uint8_t>(channel);
                cmd.delta_deg = *text == '+' ? 1 : -1;
            }
        }
    } else if ((std::strncmp(text, "save", 4) == 0 &&
                (text[4] == ' ' || text[4] == '\t')) ||
               (std::strncmp(text, "center", 6) == 0 &&
                (text[6] == ' ' || text[6] == '\t'))) {
        const bool save = text[0] == 's';
        text += save ? 4 : 6;
        unsigned channel = 0;
        if (number(text, channel) && channel < 16 && endOfLine(text)) {
            cmd.type = save ? CommandType::Save : CommandType::Center;
            cmd.channel = static_cast<std::uint8_t>(channel);
        }
    }
    return cmd;
}

} // namespace dog
