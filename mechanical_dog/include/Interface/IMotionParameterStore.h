#pragma once

#include <cstddef>
#include <cstdint>

namespace dog {

constexpr std::uint32_t kMotionParameterMagic = 0x31544750U; // "PGT1"
constexpr std::uint16_t kMotionParameterVersion = 1;

struct MotionParameterRecord {
    std::uint32_t magic = kMotionParameterMagic;
    std::uint16_t version = kMotionParameterVersion;
    std::uint16_t length = sizeof(MotionParameterRecord);
    float values[6] = {};
    std::uint32_t checksum = 0;
};

enum class MotionStoreResult { Ok, Missing, Invalid, IoError };

class IMotionParameterStore {
public:
    virtual ~IMotionParameterStore() = default;
    virtual MotionStoreResult load(MotionParameterRecord& record) = 0;
    virtual MotionStoreResult save(const MotionParameterRecord& record) = 0;
};

} // namespace dog
