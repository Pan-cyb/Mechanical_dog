#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace preferences_stub {

struct State {
    bool begin_result = true;
    unsigned begin_count = 0;
    unsigned put_count = 0;
    std::map<std::string, std::uint8_t> values;
    std::map<std::string, unsigned> get_count;
    std::map<std::string, unsigned> put_key_count;
};

inline State state;

inline void reset() { state = State{}; }

}  // namespace preferences_stub

class Preferences {
public:
    bool begin(const char*, bool) {
        ++preferences_stub::state.begin_count;
        return preferences_stub::state.begin_result;
    }

    bool isKey(const char* key) const {
        return preferences_stub::state.values.count(key) != 0U;
    }

    std::uint8_t getUChar(const char* key, std::uint8_t default_value) const {
        ++preferences_stub::state.get_count[key];
        const auto found = preferences_stub::state.values.find(key);
        return found == preferences_stub::state.values.end() ? default_value
                                                              : found->second;
    }

    std::size_t putUChar(const char* key, std::uint8_t value) {
        ++preferences_stub::state.put_count;
        ++preferences_stub::state.put_key_count[key];
        preferences_stub::state.values[key] = value;
        return 1U;
    }
};

