#pragma once

#include <cstddef>
#include <cstdint>

namespace dog {

class IRegisterBus {
public:
    virtual ~IRegisterBus() = default;
    virtual bool read(std::uint8_t address, std::uint8_t reg,
                      std::uint8_t* data, std::size_t length) = 0;
    virtual bool write(std::uint8_t address, std::uint8_t reg,
                       const std::uint8_t* data, std::size_t length) = 0;
};

} // namespace dog
