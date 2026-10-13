#pragma once

#include <cstdint>

namespace dog {

// Storage is keyed by raw PCA channel, not by leg/joint identity.
class ICalibrationStore {
public:
    virtual ~ICalibrationStore() = default;
    virtual bool load(std::uint8_t channel, std::uint8_t& center_deg) = 0;
    virtual bool save(std::uint8_t channel, std::uint8_t center_deg) = 0;
};

} // namespace dog
