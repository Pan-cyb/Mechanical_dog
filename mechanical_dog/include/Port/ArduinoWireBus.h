#pragma once

#include <Wire.h>
#include "Interface/IRegisterBus.h"

namespace dog {

class ArduinoWireBus final : public IRegisterBus {
public:
    explicit ArduinoWireBus(TwoWire& wire) : wire_(wire) {}
    bool read(std::uint8_t address, std::uint8_t reg,
              std::uint8_t* data, std::size_t length) override;
    bool write(std::uint8_t address, std::uint8_t reg,
               const std::uint8_t* data, std::size_t length) override;

private:
    TwoWire& wire_;
};

} // namespace dog
