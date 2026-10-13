#pragma once

#include <Preferences.h>
#include "Interface/ICalibrationStore.h"

namespace dog {

class PreferencesCalibrationStore final : public ICalibrationStore {
public:
    explicit PreferencesCalibrationStore(
        std::uint8_t fallback_channel = 0xFF,
        std::uint8_t legacy_channel = 0xFF);
    bool begin();
    bool load(std::uint8_t channel, std::uint8_t& center_deg) override;
    bool save(std::uint8_t channel, std::uint8_t center_deg) override;

private:
    static void key(std::uint8_t channel, char* output);
    Preferences preferences_;
    const std::uint8_t fallback_channel_;
    const std::uint8_t legacy_channel_;
    bool ready_ = false;
};

} // namespace dog
