#include "Port/ArduinoWireBus.h"

namespace dog {

bool ArduinoWireBus::read(std::uint8_t address, std::uint8_t reg,
                          std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0 || length > 32) return false;
    wire_.beginTransmission(address);
    if (wire_.write(reg) != 1 || wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(address, static_cast<std::uint8_t>(length), true) != length)
        return false;
    for (std::size_t i = 0; i < length; ++i) {
        const int value = wire_.read();
        if (value < 0) return false;
        data[i] = static_cast<std::uint8_t>(value);
    }
    return true;
}

bool ArduinoWireBus::write(std::uint8_t address, std::uint8_t reg,
                           const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0 || length > 31) return false;
    wire_.beginTransmission(address);
    if (wire_.write(reg) != 1 || wire_.write(data, length) != length) {
        (void)wire_.endTransmission(true);
        return false;
    }
    return wire_.endTransmission(true) == 0;
}

} // namespace dog
