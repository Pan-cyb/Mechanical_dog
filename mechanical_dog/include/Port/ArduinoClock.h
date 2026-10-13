#pragma once

#include <Arduino.h>
#include "Interface/IClock.h"

namespace dog {

class ArduinoClock final : public IClock {
public:
    std::uint32_t millis() const override { return ::millis(); }
    void delayMs(std::uint32_t duration) override { ::delay(duration); }
};

} // namespace dog
