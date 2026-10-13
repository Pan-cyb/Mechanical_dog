#pragma once
#include <cstdint>

namespace dog {
constexpr std::uint32_t kControlProfileMagic=0x31465041U; // APF1
constexpr std::uint16_t kControlProfileVersion=1;
struct ControlProfileRecord {
    std::uint32_t magic=kControlProfileMagic;
    std::uint16_t version=kControlProfileVersion;
    std::uint16_t length=sizeof(ControlProfileRecord);
    std::uint32_t flags=0;
    float values[23]={};
    std::uint32_t checksum=0;
};
enum class ControlProfileStoreResult{Ok,Missing,Invalid,IoError};
class IControlProfileStore {
public:
    virtual ~IControlProfileStore()=default;
    virtual ControlProfileStoreResult load(ControlProfileRecord& record)=0;
    virtual ControlProfileStoreResult save(const ControlProfileRecord& record)=0;
};
} // namespace dog
