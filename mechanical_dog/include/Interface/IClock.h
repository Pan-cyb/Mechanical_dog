#pragma once

#include <cstdint>

namespace dog {

class IClock {
public:
    virtual ~IClock() = default;
    virtual std::uint32_t millis() const = 0;
    virtual void delayMs(std::uint32_t duration) = 0;
};

} // namespace dog
