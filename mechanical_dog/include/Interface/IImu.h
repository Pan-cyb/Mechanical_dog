#pragma once

#include <cstdint>

namespace dog {

struct ImuSample {
    float accel_mps2[3] = {};
    float gyro_radps[3] = {};
    std::uint32_t timestamp_ms = 0;
};

class IImu {
public:
    virtual ~IImu() = default;
    virtual bool begin() = 0;
    virtual bool read(ImuSample& sample) = 0;
};

} // namespace dog
