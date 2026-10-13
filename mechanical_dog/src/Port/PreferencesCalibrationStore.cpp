#include "Port/PreferencesCalibrationStore.h"

#include <cstdio>

namespace dog {

PreferencesCalibrationStore::PreferencesCalibrationStore(
    std::uint8_t fallback_channel, std::uint8_t legacy_channel)
    : fallback_channel_(fallback_channel), legacy_channel_(legacy_channel) {}

bool PreferencesCalibrationStore::begin() {
    ready_ = preferences_.begin("applepycal", false);
    return ready_;
}

void PreferencesCalibrationStore::key(std::uint8_t channel, char* output) {
    std::snprintf(output, 5, "c%u", static_cast<unsigned>(channel));
}

bool PreferencesCalibrationStore::load(std::uint8_t channel, std::uint8_t& center_deg) {
    if (!ready_ || channel >= 16) return false;
    char name[5];
    key(channel, name);
    if (preferences_.isKey(name)) {
        center_deg = preferences_.getUChar(name, 90);
        return true;
    }
    if (channel != fallback_channel_ || legacy_channel_ >= 16 ||
        legacy_channel_ == fallback_channel_) return false;
    key(legacy_channel_, name);
    if (!preferences_.isKey(name)) return false;
    center_deg = preferences_.getUChar(name, 90);
    return true;
}

bool PreferencesCalibrationStore::save(std::uint8_t channel, std::uint8_t center_deg) {
    if (!ready_ || channel >= 16) return false;
    char name[5];
    key(channel, name);
    // NVS is committed only on explicit SAVE; a nudge never writes flash.
    return preferences_.putUChar(name, center_deg) == 1;
}

} // namespace dog
