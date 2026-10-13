#include "Port/FlashWebPageResponse.h"

#if defined(ARDUINO)
#include <pgmspace.h>
#endif

namespace dog {
namespace {

char readFlashByte(const char* address) {
#if defined(ARDUINO)
    return static_cast<char>(pgm_read_byte(address));
#else
    return *address;
#endif
}

bool flashMatches(const char* page, std::size_t position,
                  const char* placeholder, std::size_t placeholder_length) {
    for (std::size_t i = 0; i < placeholder_length; ++i) {
        if (readFlashByte(page + position + i) != placeholder[i]) return false;
    }
    return true;
}

bool writeFlashRange(FlashWebPageTransport& transport, const char* data,
                     std::size_t length, std::size_t chunk_length) {
    while (length != 0U) {
        const std::size_t count = length < chunk_length ? length : chunk_length;
        if (!transport.writeFlash(data, count)) return false;
        data += count;
        length -= count;
    }
    return true;
}

}  // namespace

FlashWebPageResult sendFlashWebPage(
    FlashWebPageTransport& transport, const char* page,
    std::size_t page_length, const char* placeholder,
    std::size_t placeholder_length, const char* replacement,
    std::size_t replacement_length, std::size_t flash_chunk_length) {
    if (page == nullptr || placeholder == nullptr || replacement == nullptr ||
        placeholder_length == 0U || placeholder_length > page_length ||
        flash_chunk_length == 0U) {
        return FlashWebPageResult::InvalidArgument;
    }

    std::size_t placeholder_position = page_length;
    for (std::size_t i = 0; i <= page_length - placeholder_length; ++i) {
        if (!flashMatches(page, i, placeholder, placeholder_length)) continue;
        if (placeholder_position != page_length) {
            return FlashWebPageResult::PlaceholderRepeated;
        }
        placeholder_position = i;
        i += placeholder_length - 1U;
    }
    if (placeholder_position == page_length) {
        return FlashWebPageResult::PlaceholderMissing;
    }

    const std::size_t content_length =
        page_length - placeholder_length + replacement_length;
    transport.sendHeader("Cache-Control", "no-store");
    transport.sendHeader("X-Content-Type-Options", "nosniff");
    transport.sendHeader("X-Frame-Options", "DENY");
    transport.sendHeader("Referrer-Policy", "no-referrer");
    transport.sendHeader(
        "Content-Security-Policy",
        "default-src 'none'; style-src 'unsafe-inline'; script-src "
        "'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'");
    if (!transport.beginResponse(200, "text/html; charset=utf-8",
                                 content_length)) {
        return FlashWebPageResult::TransportError;
    }
    if (!writeFlashRange(transport, page, placeholder_position,
                         flash_chunk_length) ||
        !transport.writeRam(replacement, replacement_length) ||
        !writeFlashRange(
            transport, page + placeholder_position + placeholder_length,
            page_length - placeholder_position - placeholder_length,
            flash_chunk_length)) {
        return FlashWebPageResult::TransportError;
    }
    return FlashWebPageResult::Ok;
}

}  // namespace dog

