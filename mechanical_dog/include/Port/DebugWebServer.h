#pragma once

#include <WebServer.h>
#include "UserApp/RobotApp.h"

namespace dog {

// Thin HTTP adapter. All actuator writes are dispatched through RobotApp.
class DebugWebServer {
public:
    explicit DebugWebServer(RobotApp& app);
    bool begin();
    void handleClient();

private:
    void page();
    void state();
    void action();
    void sendResult(int status, const DebugReply& reply);
    bool channelArg(std::uint8_t& channel);
    static bool unsignedArg(const String& text, unsigned& value);
    static bool uint32Arg(const String& text, std::uint32_t& value);

    RobotApp& app_;
    WebServer server_{80};
    char csrf_token_[17] = {};
    bool started_ = false;
};

} // namespace dog
