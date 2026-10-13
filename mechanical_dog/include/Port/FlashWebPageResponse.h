#pragma once

#include <cstddef>

namespace dog {

class FlashWebPageTransport {
public:
    virtual ~FlashWebPageTransport() = default;
    virtual void sendHeader(const char* name, const char* value) = 0;
    virtual bool beginResponse(int status, const char* content_type,
                               std::size_t content_length) = 0;
    virtual bool writeFlash(const char* data, std::size_t length) = 0;
    virtual bool writeRam(const char* data, std::size_t length) = 0;
};

enum class FlashWebPageResult {
    Ok,
    InvalidArgument,
    PlaceholderMissing,
    PlaceholderRepeated,
    TransportError,
};

// Streams a flash-resident template without materialising the page in RAM.
// Exactly one placeholder is required so a missing or ambiguous CSRF token is
// rejected before any response headers are sent.
FlashWebPageResult sendFlashWebPage(
    FlashWebPageTransport& transport, const char* page,
    std::size_t page_length, const char* placeholder,
    std::size_t placeholder_length, const char* replacement,
    std::size_t replacement_length, std::size_t flash_chunk_length = 1024U);

}  // namespace dog

