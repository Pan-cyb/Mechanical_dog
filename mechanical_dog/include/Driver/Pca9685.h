#pragma once

#include <cstdint>
#include "Interface/IClock.h"
#include "Interface/IRegisterBus.h"
#include "Interface/IServoPwm.h"

namespace dog {

class Pca9685 final : public IServoPwm {
public:
    Pca9685(IRegisterBus& bus, IClock& clock, std::uint8_t address,
            std::uint16_t frequency_hz);

    bool begin() override;
    bool allOff() override;
    bool offChannel(std::uint8_t channel) override;
    bool setPulseUs(std::uint8_t channel, std::uint16_t pulse_us) override;

private:
    IRegisterBus& bus_;
    IClock& clock_;
    const std::uint8_t address_;
    const std::uint16_t frequency_hz_;
    bool ready_ = false;
};

} // namespace dog
