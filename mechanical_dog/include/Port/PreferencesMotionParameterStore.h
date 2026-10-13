#pragma once

#include <Preferences.h>
#include "Interface/IMotionParameterStore.h"

namespace dog {

// Separate namespace/key from applepycal/c{channel}; gait persistence never
// rewrites or reinterprets the eight centre calibration records.
class PreferencesMotionParameterStore final : public IMotionParameterStore {
public:
    bool begin();
    MotionStoreResult load(MotionParameterRecord& record) override;
    MotionStoreResult save(const MotionParameterRecord& record) override;

private:
    Preferences preferences_;
    bool ready_ = false;
};

} // namespace dog
