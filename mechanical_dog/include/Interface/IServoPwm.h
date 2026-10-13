#pragma once

#include <cstdint>

namespace dog {

class IServoPwm {
public:
    virtual ~IServoPwm() = default;
    virtual bool begin() = 0;
    virtual bool allOff() = 0;
    virtual bool offChannel(std::uint8_t channel) = 0;
    virtual bool setPulseUs(std::uint8_t channel, std::uint16_t pulse_us) = 0;
};

} // namespace dog
