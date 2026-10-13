#include "Port/PreferencesMotionParameterStore.h"

namespace dog {
namespace {
constexpr const char* kNamespace = "applepygait";
constexpr const char* kKey = "motion";
}

bool PreferencesMotionParameterStore::begin() {
    ready_ = preferences_.begin(kNamespace, false);
    return ready_;
}

MotionStoreResult PreferencesMotionParameterStore::load(MotionParameterRecord& record) {
    if (!ready_) return MotionStoreResult::IoError;
    if (!preferences_.isKey(kKey)) return MotionStoreResult::Missing;
    if (preferences_.getBytesLength(kKey) != sizeof(record))
        return MotionStoreResult::Invalid;
    return preferences_.getBytes(kKey, &record, sizeof(record)) == sizeof(record)
        ? MotionStoreResult::Ok : MotionStoreResult::IoError;
}

MotionStoreResult PreferencesMotionParameterStore::save(
        const MotionParameterRecord& record) {
    if (!ready_) return MotionStoreResult::IoError;
    return preferences_.putBytes(kKey, &record, sizeof(record)) == sizeof(record)
        ? MotionStoreResult::Ok : MotionStoreResult::IoError;
}

} // namespace dog
