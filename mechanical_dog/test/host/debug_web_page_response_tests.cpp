#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <string>

#include "Port/FlashWebPageResponse.h"
#include "../../src/Port/DebugWebPage.h"

namespace {

constexpr std::size_t kArduinoStringCapacityMax = 65535U;
constexpr std::size_t kFlashChunkLength = 1024U;
constexpr char kPlaceholder[] = "__TOKEN__";
constexpr char kToken[] = "0123456789ABCDEF";

class LimitedString {
public:
    explicit LimitedString(const char* text) {
        const std::size_t length = std::strlen(text);
        copy(text, length);
    }

    void replace(const char* from, const char* to) {
        const std::size_t position = value_.find(from);
        if (position == std::string::npos) return;
        const std::size_t result_length =
            value_.size() - std::strlen(from) + std::strlen(to);
        if (result_length > capacity_ && !changeBuffer(result_length)) return;
        value_.replace(position, std::strlen(from), to);
    }

    const std::string& value() const { return value_; }

private:
    bool changeBuffer(std::size_t maximum_length) {
        const std::size_t allocation_size =
            (maximum_length + 16U) & ~std::size_t{0x0fU};
        if (allocation_size > kArduinoStringCapacityMax) return false;
        capacity_ = allocation_size - 1U;
        return true;
    }

    void copy(const char* text, std::size_t length) {
        if (length > capacity_ && !changeBuffer(length)) {
            value_.clear();
            capacity_ = 0U;
            return;
        }
        value_.assign(text, length);
    }

    std::string value_;
    std::size_t capacity_ = 0U;
};

class FakeLegacyWebServer {
public:
    void send(int status, const char* content_type, const LimitedString& body) {
        status_ = status;
        content_type_ = content_type;
        body_ = body.value();
    }

    int status_ = 0;
    std::string content_type_;
    std::string body_;
};

void serveLegacyPage(FakeLegacyWebServer& server, const char* page) {
    LimitedString html(page);
    html.replace(kPlaceholder, kToken);
    server.send(200, "text/html; charset=utf-8", html);
}

struct CapturedHeader {
    std::string name;
    std::string value;
};

class FakeTransport final : public dog::FlashWebPageTransport {
public:
    explicit FakeTransport(std::size_t expected_length = 0U) {
        body_.reserve(expected_length);
    }

    void sendHeader(const char* name, const char* value) override {
        assert(header_count_ < headers_.size());
        headers_[header_count_++] = {name, value};
    }

    bool beginResponse(int status, const char* content_type,
                       std::size_t content_length) override {
        ++begin_count_;
        status_ = status;
        content_type_ = content_type;
        content_length_ = content_length;
        return begin_ok_;
    }

    bool writeFlash(const char* data, std::size_t length) override {
        ++flash_write_count_;
        max_flash_write_ = std::max(max_flash_write_, length);
        if (!flash_writes_ok_) return false;
        body_.append(data, length);
        return true;
    }

    bool writeRam(const char* data, std::size_t length) override {
        ++ram_write_count_;
        max_ram_write_ = std::max(max_ram_write_, length);
        if (!ram_writes_ok_) return false;
        body_.append(data, length);
        return true;
    }

    std::array<CapturedHeader, 5> headers_{};
    std::size_t header_count_ = 0U;
    std::size_t begin_count_ = 0U;
    int status_ = 0;
    std::string content_type_;
    std::size_t content_length_ = 0U;
    std::size_t flash_write_count_ = 0U;
    std::size_t ram_write_count_ = 0U;
    std::size_t max_flash_write_ = 0U;
    std::size_t max_ram_write_ = 0U;
    std::string body_;
    bool begin_ok_ = true;
    bool flash_writes_ok_ = true;
    bool ram_writes_ok_ = true;
};

std::size_t countOccurrences(const std::string& text,
                             const std::string& needle) {
    std::size_t count = 0U;
    std::size_t position = 0U;
    while ((position = text.find(needle, position)) != std::string::npos) {
        ++count;
        position += needle.size();
    }
    return count;
}

std::string expectedBody(const char* page, std::size_t page_length) {
    std::string expected(page, page_length);
    const std::size_t position = expected.find(kPlaceholder);
    assert(position != std::string::npos);
    expected.replace(position, std::strlen(kPlaceholder), kToken);
    return expected;
}

void assertSecurityHeaders(const FakeTransport& transport) {
    const std::array<CapturedHeader, 5> expected{{
        {"Cache-Control", "no-store"},
        {"X-Content-Type-Options", "nosniff"},
        {"X-Frame-Options", "DENY"},
        {"Referrer-Policy", "no-referrer"},
        {"Content-Security-Policy",
         "default-src 'none'; style-src 'unsafe-inline'; script-src "
         "'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'"},
    }};
    assert(transport.header_count_ == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        assert(transport.headers_[i].name == expected[i].name);
        assert(transport.headers_[i].value == expected[i].value);
    }
}

dog::FlashWebPageResult send(FakeTransport& transport, const char* page,
                             std::size_t page_length) {
    return dog::sendFlashWebPage(
        transport, page, page_length, kPlaceholder,
        std::strlen(kPlaceholder), kToken, std::strlen(kToken),
        kFlashChunkLength);
}

void testLegacyWholePageStringFailureIsReproduced() {
    std::string oversized_page(kArduinoStringCapacityMax + 1U, 'x');
    oversized_page.replace(1024U, std::strlen(kPlaceholder), kPlaceholder);
    FakeLegacyWebServer server;
    serveLegacyPage(server, oversized_page.c_str());
    assert(oversized_page.size() > kArduinoStringCapacityMax);
    assert(server.status_ == 200);
    assert(server.content_type_ == "text/html; charset=utf-8");
    assert(server.body_.empty());
}

void testRealUtf8PageStreamsExactly() {
    const std::size_t page_length = sizeof(dog::kDebugWebPage) - 1U;
    const std::string expected = expectedBody(dog::kDebugWebPage, page_length);
    FakeTransport transport(expected.size());

    assert(send(transport, dog::kDebugWebPage, page_length) ==
           dog::FlashWebPageResult::Ok);
    assert(transport.begin_count_ == 1U);
    assert(transport.status_ == 200);
    assert(transport.content_type_ == "text/html; charset=utf-8");
    assert(transport.content_length_ == expected.size());
    assert(transport.body_.size() == transport.content_length_);
    assert(transport.body_ == expected);
    assert(countOccurrences(transport.body_, kToken) == 1U);
    assert(countOccurrences(transport.body_, kPlaceholder) == 0U);
    assert(transport.ram_write_count_ == 1U);
    assert(transport.max_ram_write_ == std::strlen(kToken));
    assert(transport.max_flash_write_ <= kFlashChunkLength);
    assertSecurityHeaders(transport);
}

void testLargePageAndChunkBoundaryMarkers() {
    constexpr std::size_t kLargeSize = 128U * 1024U + 333U;
    for (const std::size_t position :
         {kFlashChunkLength - 1U, kFlashChunkLength,
          kFlashChunkLength + 1U}) {
        std::string page(kLargeSize, 'x');
        page.replace(position, std::strlen(kPlaceholder), kPlaceholder);
        const std::string expected = expectedBody(page.data(), page.size());
        FakeTransport transport(expected.size());
        assert(send(transport, page.data(), page.size()) ==
               dog::FlashWebPageResult::Ok);
        assert(transport.content_length_ == expected.size());
        assert(transport.body_ == expected);
        assert(transport.max_flash_write_ <= kFlashChunkLength);
        assert(transport.max_ram_write_ == std::strlen(kToken));
    }
}

void testTemplateValidationRejectsBeforeResponse() {
    const std::string missing = "<html>no token</html>";
    FakeTransport missing_transport;
    assert(send(missing_transport, missing.data(), missing.size()) ==
           dog::FlashWebPageResult::PlaceholderMissing);
    assert(missing_transport.header_count_ == 0U);
    assert(missing_transport.begin_count_ == 0U);
    assert(missing_transport.body_.empty());

    const std::string repeated =
        std::string("a") + kPlaceholder + "b" + kPlaceholder + "c";
    FakeTransport repeated_transport;
    assert(send(repeated_transport, repeated.data(), repeated.size()) ==
           dog::FlashWebPageResult::PlaceholderRepeated);
    assert(repeated_transport.header_count_ == 0U);
    assert(repeated_transport.begin_count_ == 0U);
    assert(repeated_transport.body_.empty());

    FakeTransport invalid_transport;
    assert(dog::sendFlashWebPage(invalid_transport, nullptr, 0U,
                                 kPlaceholder, std::strlen(kPlaceholder),
                                 kToken, std::strlen(kToken),
                                 kFlashChunkLength) ==
           dog::FlashWebPageResult::InvalidArgument);
    assert(invalid_transport.begin_count_ == 0U);
}

void testRepeatedGetsOnlyProduceIdenticalResponses() {
    const std::size_t page_length = sizeof(dog::kDebugWebPage) - 1U;
    const std::size_t expected_length =
        page_length - std::strlen(kPlaceholder) + std::strlen(kToken);
    FakeTransport first(expected_length);
    FakeTransport second(expected_length);

    assert(send(first, dog::kDebugWebPage, page_length) ==
           dog::FlashWebPageResult::Ok);
    assert(send(second, dog::kDebugWebPage, page_length) ==
           dog::FlashWebPageResult::Ok);
    assert(first.body_ == second.body_);
}

void testTransportFailureStopsWithoutLaterWrites() {
    const std::string page = std::string(1500U, 'p') + kPlaceholder +
                             std::string(1500U, 's');

    FakeTransport begin_failure;
    begin_failure.begin_ok_ = false;
    assert(send(begin_failure, page.data(), page.size()) ==
           dog::FlashWebPageResult::TransportError);
    assert(begin_failure.begin_count_ == 1U);
    assert(begin_failure.flash_write_count_ == 0U);
    assert(begin_failure.ram_write_count_ == 0U);

    FakeTransport flash_failure;
    flash_failure.flash_writes_ok_ = false;
    assert(send(flash_failure, page.data(), page.size()) ==
           dog::FlashWebPageResult::TransportError);
    assert(flash_failure.flash_write_count_ == 1U);
    assert(flash_failure.ram_write_count_ == 0U);
    assert(flash_failure.body_.empty());

    FakeTransport ram_failure;
    ram_failure.ram_writes_ok_ = false;
    assert(send(ram_failure, page.data(), page.size()) ==
           dog::FlashWebPageResult::TransportError);
    assert(ram_failure.flash_write_count_ == 2U);
    assert(ram_failure.ram_write_count_ == 1U);
    assert(ram_failure.body_.size() == 1500U);
}

}  // namespace

int main() {
    testLegacyWholePageStringFailureIsReproduced();
    testRealUtf8PageStreamsExactly();
    testLargePageAndChunkBoundaryMarkers();
    testTemplateValidationRejectsBeforeResponse();
    testRepeatedGetsOnlyProduceIdenticalResponses();
    testTransportFailureStopsWithoutLaterWrites();
    std::cout << "debug web page response tests passed\n";
    return 0;
}
