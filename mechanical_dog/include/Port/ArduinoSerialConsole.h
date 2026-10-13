#pragma once

#include <Arduino.h>
#include "Interface/IConsole.h"

namespace dog {

class ArduinoSerialConsole final : public IConsole {
public:
    explicit ArduinoSerialConsole(HardwareSerial& serial) : serial_(serial) {}
    void line(const char* text) override { serial_.println(text); }

private:
    HardwareSerial& serial_;
};

} // namespace dog
