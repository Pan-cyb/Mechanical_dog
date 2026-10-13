#include "Port/DebugWebServer.h"

#include <WiFi.h>
#include <esp_system.h>
#include <cmath>
#include <cstdio>
#include "Board/DebugWifiConfig.h"
#include "Port/FlashWebPageResponse.h"
#include "Port/DebugWebPage.h"

namespace dog {
namespace {

class WebServerPageTransport final : public FlashWebPageTransport {
public:
    explicit WebServerPageTransport(WebServer& server) : server_(server) {}

    void sendHeader(const char* name, const char* value) override {
        server_.sendHeader(name, value);
    }

    bool beginResponse(int status, const char* content_type,
                       std::size_t content_length) override {
        server_.setContentLength(content_length);
        server_.send(status, content_type, String());
        // WebServer's void send API only confirms that the response was handed
        // to its client layer; it does not expose network short-write status.
        return true;
    }

    bool writeFlash(const char* data, std::size_t length) override {
        server_.sendContent_P(data, length);
        // sendContent_P is also void, so delivery cannot be confirmed here.
        return true;
    }

    bool writeRam(const char* data, std::size_t length) override {
        server_.sendContent(data, length);
        // sendContent has the same void/no-delivery-status limitation.
        return true;
    }

private:
    WebServer& server_;
};

}  // namespace

DebugWebServer::DebugWebServer(RobotApp& app) : app_(app) {}

bool DebugWebServer::begin() {
    if (started_) return true;
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP); // No connection to a router or the Internet.
    if (!WiFi.softAP(board::kDebugApSsid, board::kDebugApPassword, 6, false, 1))
        return false;
    std::snprintf(csrf_token_, sizeof(csrf_token_), "%08lX%08lX",
                  static_cast<unsigned long>(esp_random()),
                  static_cast<unsigned long>(esp_random()));
    const char* headers[] = {"X-Applepy-Token"};
    server_.collectHeaders(headers, 1);
    server_.on("/", HTTP_GET, [this] { page(); });
    server_.on("/api/state", HTTP_GET, [this] { state(); });
    server_.on("/api/action", HTTP_POST, [this] { action(); });
    server_.onNotFound([this] {
        server_.sendHeader("Cache-Control", "no-store");
        server_.send(404, "text/plain; charset=utf-8", "Not found");
    });
    server_.begin();
    started_ = true;
    return true;
}

void DebugWebServer::handleClient() {
    if (started_) server_.handleClient();
}

void DebugWebServer::page() {
    WebServerPageTransport transport(server_);
    const FlashWebPageResult result = sendFlashWebPage(
        transport, kDebugWebPage, sizeof(kDebugWebPage) - 1U, "__TOKEN__",
        sizeof("__TOKEN__") - 1U, csrf_token_, sizeof(csrf_token_) - 1U);
    if (result == FlashWebPageResult::InvalidArgument ||
        result == FlashWebPageResult::PlaceholderMissing ||
        result == FlashWebPageResult::PlaceholderRepeated) {
        server_.sendHeader("Cache-Control", "no-store");
        server_.send(500, "text/plain; charset=utf-8",
                     "Invalid embedded debug page template");
    } else if (result == FlashWebPageResult::TransportError) {
        // A 200 header or partial body may already be on the wire. Never append
        // a second HTTP response with the previous fixed Content-Length.
        server_.client().stop();
    }
}

void DebugWebServer::state() {
    const DebugSnapshot s = app_.snapshot();
    String json;
    json.reserve(2200);
    json += "{\"firmware_version\":\""; json += s.firmware_version;
    json += "\",\"pca\":"; json += s.pca ? "true" : "false";
    json += ",\"armed\":"; json += s.armed ? "true" : "false";
    json += ",\"calibrating\":"; json += s.calibrating ? "true" : "false";
    json += ",\"calibration_pending\":"; json += s.calibration_pending ? "true" : "false";
    json += ",\"calibration_mask\":"; json += String(s.calibration_mask);
    json += ",\"fault\":"; json += s.fault ? "true" : "false";
    json += ",\"imu_ok\":"; json += s.imu_ok ? "true" : "false";
    json += ",\"calibration_samples\":";
    json += String(s.attitude_calibration_samples);
    json += ",\"gyro_calibrated\":";
    json += s.gyro_calibrated ? "true" : "false";
    json += ",\"calibration_required\":";
    json += String(s.attitude_calibration_required);
    json += ",\"accel_norm_mps2\":";
    json += s.imu_ok ? String(s.accel_norm_mps2, 3) : "null";
    json += ",\"active_channel\":"; json += String(s.active_channel);
    json += ",\"active_pulse_us\":"; json += String(s.active_pulse_us);
    json += ",\"remaining_ms\":"; json += String(s.remaining_ms);
    json += ",\"manual_hold\":"; json += s.manual_hold ? "true" : "false";
    json += ",\"extended_range\":"; json += s.extended_range ? "true" : "false";
    json += ",\"pulse_min_us\":"; json += String(s.pulse_min_us);
    json += ",\"pulse_max_us\":"; json += String(s.pulse_max_us);
    json += ",\"motion\":{\"mode\":\""; json += s.motion_mode;
    json += "\",\"error\":\""; json += s.motion_error;
    json += "\",\"pending\":"; json += s.motion_pending ? "true":"false";
    json += ",\"transition\":"; json += s.motion_transition ? "true":"false";
    json += ",\"phase\":"; json += String(s.motion_phase,3);
    json += ",\"parameter_state\":\""; json += s.motion_parameter_state; json += '"';
    json += ",\"parameters_saved\":"; json += s.motion_parameters_saved ? "true":"false";
    json += ",\"parameter_revision\":"; json += String(s.motion_parameter_revision);
    json += ",\"diagnostics\":{\"frame_count\":"; json += String(s.motion_frame_count);
    json += ",\"last_gap_ms\":"; json += String(s.motion_last_update_gap_ms);
    json += ",\"max_gap_ms\":"; json += String(s.motion_max_update_gap_ms);
    json += ",\"missed_deadlines\":"; json += String(s.motion_missed_deadlines); json += '}';
    json += ",\"parameters\":[";
    json += String(s.motion_parameters.step_length_mm,2); json += ',';
    json += String(s.motion_parameters.lift_mm,2); json += ',';
    json += String(s.motion_parameters.period_s,2); json += ',';
    json += String(s.motion_parameters.stand_down_mm,2); json += ',';
    json += String(s.motion_parameters.stand_forward_mm,2); json += ',';
    json += String(s.motion_parameters.walk_body_shift_mm,2); json += ']';
    json += ",\"feet\":[";
    for (int i=0;i<4;++i) { if(i)json+=','; json += '['; json += String(s.target_forward_mm[i],2); json += ','; json += String(s.target_down_mm[i],2); json += ']'; }
    json += "],\"command_deg\":[";
    for (int i=0;i<8;++i) { if(i)json+=','; json += String(s.command_deg[i],2); }
    json += "],\"body\":{\"pitch_deg\":";json+=String(s.advanced_parameters.body.pitch_deg,2);
    json += ",\"roll_deg\":";json+=String(s.advanced_parameters.body.roll_deg,2);
    json += ",\"x_mm\":";json+=String(s.advanced_parameters.body.body_x_mm,2);
    json += ",\"cg_enabled\":";json+=s.advanced_parameters.cg_walk_enabled?"true":"false";
    json += ",\"cg_shift_mm\":";json+=String(s.advanced_parameters.cg_shift_mm,2);
    json += ",\"revision\":";json+=String(s.advanced_parameter_revision);
    json += ",\"length_mm\":";json+=String(s.body_length_mm,1);
    json += ",\"width_mm\":";json+=String(s.body_width_mm,1);
    json += ",\"dimensions_measured\":";json+=s.body_dimensions_measured?"true":"false";
    json += ",\"cg_actual_x_min_mm\":";json+=String(s.cg_actual_body_x_min_mm,2);
    json += ",\"cg_actual_x_max_mm\":";json+=String(s.cg_actual_body_x_max_mm,2);json+='}';
    json += ",\"balance\":{\"capture_pending\":";json+=s.balance_capture_pending?"true":"false";
    json += ",\"capture_samples\":";json+=String(s.balance_capture_samples);
    json += ",\"capture_required\":";json+=String(s.balance_capture_required);
    json += ",\"reference_ready\":";json+=s.balance_reference_ready?"true":"false";
    json += ",\"enabled\":";json+=s.balance_enabled?"true":"false";
    json += ",\"enable_pending\":";json+=s.balance_enable_pending?"true":"false";
    json += ",\"reference_pitch_deg\":";json+=String(s.balance_reference_pitch_deg,2);
    json += ",\"reference_roll_deg\":";json+=String(s.balance_reference_roll_deg,2);
    json += ",\"pitch_correction_deg\":";json+=String(s.balance_pitch_correction_deg,2);
    json += ",\"roll_correction_deg\":";json+=String(s.balance_roll_correction_deg,2);
    json += ",\"roll_sensor_sign\":";json+=String(static_cast<int>(s.balance_roll_sensor_sign));
    json += ",\"error\":\"";json+=s.balance_error;json+="\"}";
    json += ",\"preview\":{\"error\":\"";json+=s.preview_error;json+='"';
    json += ",\"leg\":";json+=String(s.preview_leg);
    json += ",\"joint\":";json+=String(s.preview_joint);
    json += ",\"phase\":";json+=String(s.preview_phase,3);
    json += ",\"min_support_margin_mm\":";json+=String(s.preview_min_support_margin_mm,2);
    json += ",\"pending\":";json+=s.preview_pending?"true":"false";
    json += ",\"stage\":\"";json+=s.preview_stage;json+='"';
    json += ",\"result\":\"";json+=s.preview_result;json+='"';
    json += ",\"frames_checked\":";json+=String(s.preview_frames_checked);
    json += ",\"max_slice_ms\":";json+=String(s.preview_max_slice_ms);
    json += ",\"duration_ms\":";json+=String(s.preflight_duration_ms);
    json += ",\"actual_cycle_s\":";json+=String(s.preflight_actual_cycle_s,2);json+='}';
    json += ",\"drive\":{\"forward\":";json+=String(s.drive_intent.forward,2);
    json += ",\"turn\":";json+=String(s.drive_intent.turn,2);
    json += ",\"straight_trim\":";json+=String(s.straight_trim,2);
    json += ",\"effective_turn\":";json+=String(s.effective_drive_turn,2);
    json += ",\"trot_pitch_offset_deg\":";json+=String(s.trot_pitch_offset_deg,2);
    json += ",\"trot_effective_pitch_deg\":";json+=String(s.trot_effective_pitch_deg,2);
    json += ",\"trot_pitch_revision\":";json+=String(s.trot_pitch_revision);
    json += ",\"stepping\":";json+=s.drive_intent.stepping?"true":"false";
    json += ",\"active\":";json+=s.drive_session_active?"true":"false";
    json += ",\"epoch\":";json+=String(s.drive_epoch);
    json += ",\"seq\":";json+=String(s.drive_seq);
    json += ",\"lease_remaining_ms\":";json+=String(s.drive_lease_remaining_ms);
    json += ",\"reason\":\"";json+=s.drive_session_reason;json+='"';
    json += ",\"moving_cg_enabled\":";json+=s.moving_cg_enabled?"true":"false";
    json += ",\"moving_cg_pending\":";json+=s.moving_cg_enable_pending?"true":"false";
    json += ",\"moving_cg_shift_mm\":";json+=String(s.moving_cg_shift_mm,3);
    json += ",\"moving_cg_error\":\"";json+=s.moving_cg_error;json+='"';
    json += ",\"phase_waiting\":";json+=s.cg_phase_waiting?"true":"false";
    json += ",\"cg_stage\":";json+=String(s.cg_stage);
    json += ",\"nominal_elapsed_s\":";json+=String(s.motion_nominal_elapsed_s,2);
    json += ",\"actual_elapsed_s\":";json+=String(s.motion_actual_elapsed_s,2);json+='}';
    json += ",\"profile\":{\"state\":\"";json+=s.control_profile_state;json+='"';
    json += ",\"saved\":";json+=s.control_profile_saved?"true":"false";
    json += ",\"revision\":";json+=String(s.control_profile_revision);
    json += ",\"planner_slew_mm_s\":";json+=String(s.control_profile.planner.body_slew_mm_s,2);
    json += ",\"wait_timeout_ms\":";json+=String(s.control_profile.planner.wait_timeout_ms);
    json += ",\"moving_angle_gain\":";json+=String(s.control_profile.moving.angle_gain,3);
    json += ",\"moving_rate_gain\":";json+=String(s.control_profile.moving.rate_gain_mm_per_rad_s,3);
    json += ",\"moving_tau_s\":";json+=String(s.control_profile.moving.filter_tau_s,3);
    json += ",\"moving_max_shift_mm\":";json+=String(s.control_profile.moving.max_abs_shift_mm,3);
    json += ",\"moving_slew_mm_s\":";json+=String(s.control_profile.moving.max_slew_mm_s,3);
    json += ",\"pitch_kp\":";json+=String(s.control_profile.pitch_kp,3);
    json += ",\"pitch_kd_s\":";json+=String(s.control_profile.pitch_kd_s,3);
    json += ",\"roll_kp\":";json+=String(s.control_profile.roll_kp,3);
    json += ",\"roll_kd_s\":";json+=String(s.control_profile.roll_kd_s,3);
    json += ",\"trot_ff_enabled\":";json+=s.control_profile.trot_feedforward.enabled?"true":"false";
    json += ",\"trot_ff_forward\":";json+=String(s.control_profile.trot_feedforward.forward_gain,3);
    json += ",\"trot_ff_backward\":";json+=String(s.control_profile.trot_feedforward.backward_gain,3);
    json += ",\"trot_ff_turn\":";json+=String(s.control_profile.trot_feedforward.turn_gain,3);
    json += ",\"trot_ff_max_mm\":";json+=String(s.control_profile.trot_feedforward.max_abs_mm,3);
    json += ",\"trot_ff_current_mm\":";json+=String(s.trot_feedforward_mm,3);json+="}}";
    json += ",\"imu\":";
    if (!s.imu_ok) json += "null";
    else {
        json += "{\"accel\":[";
        for (int i = 0; i < 3; ++i) {
            if (i) json += ',';
            json += String(s.imu.accel_mps2[i], 3);
        }
        json += "],\"gyro\":[";
        for (int i = 0; i < 3; ++i) {
            if (i) json += ',';
            json += String(s.imu.gyro_radps[i], 3);
        }
        json += "]}";
    }
    json += ",\"attitude_ok\":"; json += s.attitude_ok ? "true" : "false";
    json += ",\"attitude\":";
    if (!s.attitude_ok) json += "null";
    else {
        json += "{\"roll_deg\":"; json += String(s.attitude.roll_deg, 2);
        json += ",\"pitch_deg\":"; json += String(s.attitude.pitch_deg, 2);
        json += ",\"yaw_deg\":"; json += String(s.attitude.yaw_deg, 2);
        json += ",\"yaw_reference_ms\":"; json += String(s.attitude.yaw_reference_ms);
        json += '}';
    }
    json += ",\"channels\":[";
    for (std::uint8_t i = 0; i < s.channel_count; ++i) {
        if (i) json += ',';
        json += "{\"servo\":"; json += String(s.channels[i].servo);
        json += ",\"channel\":"; json += String(s.channels[i].channel);
        json += ",\"center_us\":"; json += String(s.channels[i].center_us);
        json += ",\"center_deg\":"; json += String(s.channels[i].center_deg);
        json += ",\"joint\":\""; json += s.channels[i].joint; json += '"';
        json += ",\"relative_available\":"; json += s.channels[i].relative_available ? "true" : "false";
        json += ",\"shank\":"; json += s.channels[i].shank ? "true" : "false";
        json += ",\"min_offset_deg\":"; json += String(s.channels[i].min_offset_deg, 1);
        json += ",\"max_offset_deg\":"; json += String(s.channels[i].max_offset_deg, 1);
        json += ",\"shank_q_available\":"; json += s.channels[i].shank_q_available ? "true" : "false";
        json += ",\"shank_q_photo_candidate\":"; json += s.channels[i].shank_q_photo_candidate ? "true" : "false";
        json += ",\"shank_direction\":"; json += String(static_cast<int>(s.channels[i].shank_direction));
        json += ",\"max_shank_q_deg\":"; json += String(s.channels[i].max_shank_q_deg, 3);
        json += ",\"shank_map_points\":[";
        for (std::uint8_t n = 0; n < s.channels[i].shank_map_point_count; ++n) {
            if (n) json += ',';
            json += "{\"u_deg\":"; json += String(s.channels[i].shank_map_points[n].u_deg, 3);
            json += ",\"q_deg\":"; json += String(s.channels[i].shank_map_points[n].q_deg, 3);
            json += '}';
        }
        json += ']';
        json += '}';
    }
    json += "]}";
    server_.sendHeader("Cache-Control", "no-store");
    server_.sendHeader("X-Content-Type-Options", "nosniff");
    server_.send(200, "application/json; charset=utf-8", json);
}

bool DebugWebServer::unsignedArg(const String& text, unsigned& value) {
    if (text.isEmpty() || text.length() > 5) return false;
    value = 0;
    for (unsigned i = 0; i < text.length(); ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
        value = value * 10U + static_cast<unsigned>(text[i] - '0');
    }
    return true;
}

bool DebugWebServer::uint32Arg(const String& text,std::uint32_t& value) {
    if(text.isEmpty()||text.length()>10)return false;
    std::uint64_t parsed=0;
    for(unsigned i=0;i<text.length();++i){if(text[i]<'0'||text[i]>'9')return false;
        parsed=parsed*10U+static_cast<unsigned>(text[i]-'0');if(parsed>0xFFFFFFFFULL)return false;}
    value=static_cast<std::uint32_t>(parsed);return true;
}

bool DebugWebServer::channelArg(std::uint8_t& channel) {
    unsigned value = 0;
    if (!server_.hasArg("channel") || !unsignedArg(server_.arg("channel"), value) ||
        value >= 16) return false;
    channel = static_cast<std::uint8_t>(value);
    return true;
}

void DebugWebServer::sendResult(int status, const DebugReply& reply) {
    // Reply strings are fixed firmware literals, never untrusted HTTP input.
    String json = "{\"ok\":";
    json += reply.ok ? "true" : "false";
    json += ",\"message\":\"";
    json += reply.message;
    json += "\"}";
    server_.sendHeader("Cache-Control", "no-store");
    server_.sendHeader("X-Content-Type-Options", "nosniff");
    server_.send(status, "application/json; charset=utf-8", json);
}

void DebugWebServer::action() {
    if (!server_.hasHeader("X-Applepy-Token") ||
        server_.header("X-Applepy-Token") != csrf_token_) {
        sendResult(403, {false, "Session token rejected; reload the page"});
        return;
    }
    const String op = server_.arg("cmd");
    Command command{};
    if (op == "arm") command.type = CommandType::Arm;
    else if (op == "arm_stand") command.type = CommandType::ArmStand;
    else if (op == "off") command.type = CommandType::Off;
    else if (op == "motion_stop") command.type = CommandType::MotionStop;
    else if (op == "motion_save") command.type = CommandType::MotionSave;
    else if (op == "motion_load") command.type = CommandType::MotionLoad;
    else if (op == "profile_save") command.type=CommandType::ProfileSave;
    else if (op == "profile_load") command.type=CommandType::ProfileLoad;
    else if (op == "balance_capture") command.type = CommandType::BalanceCapture;
    else if (op == "balance_on") command.type = CommandType::BalanceOn;
    else if (op == "balance_off") command.type = CommandType::BalanceOff;
    else if (op == "moving_cg_on") command.type=CommandType::MovingCgOn;
    else if (op == "moving_cg_off") command.type=CommandType::MovingCgOff;
    else if (op == "drive_begin") {
        std::uint32_t epoch=0,seq=0;unsigned step=0;
        float forward=0,turn=0;
        const String mode=server_.arg("mode");
        if(!uint32Arg(server_.arg("epoch"),epoch)||!uint32Arg(server_.arg("seq"),seq)||
           !parseServoOffset(server_.arg("forward").c_str(),forward)||
           !parseServoOffset(server_.arg("turn").c_str(),turn)||
           !unsignedArg(server_.arg("step"),step)||step>1||
           (mode!="walk"&&mode!="trot")){
            sendResult(400,{false,"Invalid drive begin fields"});return;}
        command.type=CommandType::DriveBegin;command.epoch=epoch;command.seq=seq;
        command.drive_mode=mode=="walk"?MotionMode::Walk:MotionMode::Trot;
        command.drive_intent.forward=forward;command.drive_intent.turn=turn;
        command.drive_intent.stepping=step==1;
    }
    else if (op == "drive_renew"||op == "drive_end") {
        std::uint32_t epoch=0,seq=0;
        if(!uint32Arg(server_.arg("epoch"),epoch)||!uint32Arg(server_.arg("seq"),seq)){
            sendResult(400,{false,"Invalid drive epoch/seq"});return;}
        command.type=op=="drive_renew"?CommandType::DriveRenew:CommandType::DriveEnd;
        command.epoch=epoch;command.seq=seq;
    }
    else if (op == "balance_roll_sign") {
        const String sign=server_.arg("sign");
        if(sign!="-1"&&sign!="0"&&sign!="1"&&sign!="+1") {
            sendResult(400,{false,"Roll sensor sign must be -1, 0, or +1"});return;
        }
        command.type=CommandType::BalanceRollSign;
        command.axis_sign=sign=="-1"?-1:(sign=="0"?0:1);
    }
    else if (op == "stand") command.type = CommandType::Stand;
    else if (op == "walk") command.type = CommandType::Walk;
    else if (op == "trot") command.type = CommandType::Trot;
    else if (op == "leg_test") {
        unsigned leg=0;
        if (!server_.hasArg("leg") || !unsignedArg(server_.arg("leg"),leg) || leg<1 || leg>4) {
            sendResult(400,{false,"Leg must be 1..4"}); return;
        }
        command.type=CommandType::LegTest; command.leg=static_cast<std::uint8_t>(leg);
    }
    else if (op == "motion_preview") {
        const String mode=server_.arg("mode");
        command.type=CommandType::MotionPreview;
        if(mode=="stand")command.motion_mode=MotionMode::Stand;
        else if(mode=="walk")command.motion_mode=MotionMode::Walk;
        else if(mode=="trot")command.motion_mode=MotionMode::Trot;
        else if(mode=="leg1")command.motion_mode=MotionMode::Leg1;
        else if(mode=="leg2")command.motion_mode=MotionMode::Leg2;
        else if(mode=="leg3")command.motion_mode=MotionMode::Leg3;
        else if(mode=="leg4")command.motion_mode=MotionMode::Leg4;
        else { sendResult(400,{false,"Invalid preview mode"}); return; }
    }
    else if (op == "motion_set") {
        const char* names[6]={"step","lift","period","height","forward","shift"};
        command.type=CommandType::MotionSet;
        for(int i=0;i<6;++i) {
            float value=0;
            if(!server_.hasArg(names[i]) || !parseServoOffset(server_.arg(names[i]).c_str(),value)) {
                sendResult(400,{false,"Invalid motion parameter decimal"}); return;
            }
            command.motion_values[i]=value;
        }
    }
    else if (op == "body_set") {
        const char* names[5]={"pitch","roll","x","cg","cg_shift"};
        command.type=CommandType::BodySet;
        for(int i=0;i<5;++i) {
            float value=0;
            if(!server_.hasArg(names[i]) ||
               !parseServoOffset(server_.arg(names[i]).c_str(),value)) {
                sendResult(400,{false,"Invalid body/CG parameter decimal"});return;
            }
            command.body_values[i]=value;
        }
    }
    else if (op == "pose_live_set") {
        const char* names[4]={"height","pitch","roll","x"};
        command.type=CommandType::PoseLiveSet;
        for(int i=0;i<4;++i) {
            float value=0;
            if(!server_.hasArg(names[i]) ||
               !parseServoOffset(server_.arg(names[i]).c_str(),value)) {
                sendResult(400,{false,"Invalid live pose decimal"});return;
            }
            command.pose_live_values[i]=value;
        }
    }
    else if (op == "control_set") {
        const char* names[11]={"planner_slew","wait_s","angle_gain","rate_gain",
                              "tau","max_shift","feedback_slew","pitch_kp","pitch_kd",
                              "roll_kp","roll_kd"};
        command.type=CommandType::ControlSet;
        for(int i=0;i<11;++i){float value=0;
            if(!server_.hasArg(names[i])||!parseServoOffset(server_.arg(names[i]).c_str(),value)){
                sendResult(400,{false,"Invalid control profile decimal"});return;}
            command.control_values[i]=value;}
    }
    else if (op == "trot_ff_set") {
        const char* names[5]={"enable","forward_gain","backward_gain","turn_gain","max_abs"};
        command.type=CommandType::TrotFeedforwardSet;
        for(int i=0;i<5;++i){float value=0;
            if(!server_.hasArg(names[i])||!parseServoOffset(server_.arg(names[i]).c_str(),value)){
                sendResult(400,{false,"Invalid Trot feedforward decimal"});return;}
            command.trot_ff_values[i]=value;}
    }
    else if (op == "straight_trim_set") {
        float value=0;
        if(!server_.hasArg("value")||
           !parseServoOffset(server_.arg("value").c_str(),value)||
           value < -0.30f || value > 0.30f) {
            sendResult(400,{false,"Straight trim must be -0.30..+0.30"});return;
        }
        command.type=CommandType::StraightTrimSet;
        command.straight_trim=value;
    }
    else if (op == "trot_pitch_set") {
        float value=0;
        if(!server_.hasArg("value")||
           !parseServoOffset(server_.arg("value").c_str(),value)||
           value < -0.5f || value > 0.0f ||
           std::fabs(value*10.0f-std::round(value*10.0f))>0.001f) {
            sendResult(400,{false,"Trot Pitch offset must be -0.5..0.0 degrees in 0.1 degree steps"});return;
        }
        command.type=CommandType::TrotPitchSet;
        command.trot_pitch_offset_deg=value;
    }
    else if (op == "range_normal") command.type = CommandType::RangeNormal;
    else if (op == "range_extended") {
        if (server_.arg("confirm") != "EXTENDED_RANGE_RISK") {
            sendResult(400, {false, "Confirm unloaded linkage and physical power cutoff before extending pulse range"});
            return;
        }
        command.type = CommandType::RangeExtended;
    }
    else if (op == "cal_start") {
        if (server_.arg("confirm") != "LOOSE_LINKAGES") {
            sendResult(400, {false, "Loosen all linkages and confirm physical power cutoff"});
            return;
        }
        command.type = CommandType::CalStart;
    }
    else if (op == "cal_save_all") command.type = CommandType::CalSaveAll;
    else {
        std::uint8_t ch = 0;
        if (!channelArg(ch)) {
            sendResult(400, {false, "Invalid physical channel"});
            return;
        }
        command.channel = ch;
        if (op == "off_channel") command.type = CommandType::OffChannel;
        else if (op == "pulse") {
            unsigned pulse = 0;
            if (!server_.hasArg("us") || !unsignedArg(server_.arg("us"), pulse) ||
                pulse < 500 || pulse > 2500) {
                sendResult(400, {false, "Pulse must be 500..2500 us; extended values require explicit range unlock"});
                return;
            }
            command.type = CommandType::Pwm;
            command.pulse_us = static_cast<std::uint16_t>(pulse);
        } else if (op == "offset") {
            float offset = 0;
            if (!server_.hasArg("deg") || !parseServoOffset(server_.arg("deg").c_str(), offset)) {
                sendResult(400, {false, "Offset must be a signed decimal in -180..180 degrees"});
                return;
            }
            command.type = CommandType::Offset;
            command.offset_deg = offset;
        } else if (op == "shank") {
            float q = 0;
            if (!server_.hasArg("q_deg") ||
                !parseShankAngle(server_.arg("q_deg").c_str(), q)) {
                sendResult(400, {false, "Shank q must be a non-negative decimal angle"});
                return;
            }
            command.type = CommandType::Shank;
            command.shank_q_deg = q;
        } else if (op == "nudge") {
            const String delta = server_.arg("delta");
            if (delta != "+1" && delta != "-1") {
                sendResult(400, {false, "Nudge must be +1 or -1 degree"});
                return;
            }
            command.type = CommandType::Nudge;
            command.delta_deg = delta == "+1" ? 1 : -1;
        } else if (op == "save") command.type = CommandType::Save;
        else {
            sendResult(400, {false, "Unknown action"});
            return;
        }
    }
    const DebugReply reply = app_.execute(command);
    sendResult(reply.ok ? 200 : 409, reply);
}

} // namespace dog
