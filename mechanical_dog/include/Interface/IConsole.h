#pragma once

namespace dog {

class IConsole {
public:
    virtual ~IConsole() = default;
    virtual void line(const char* text) = 0;
};

} // namespace dog
