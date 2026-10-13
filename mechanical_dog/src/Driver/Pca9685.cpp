#include "Driver/Pca9685.h"

namespace dog {
namespace {
constexpr std::uint8_t kMode1 = 0x00;
constexpr std::uint8_t kMode2 = 0x01;
constexpr std::uint8_t kLed0OnLow = 0x06;
constexpr std::uint8_t kAllLedOffHigh = 0xFD;
constexpr std::uint8_t kPrescale = 0xFE;
constexpr std::uint32_t kOscillatorHz = 25000000; // Typical internal oscillator.
}

Pca9685::Pca9685(IRegisterBus& bus, IClock& clock, std::uint8_t address,
                 std::uint16_t frequency_hz)
    : bus_(bus), clock_(clock), address_(address), frequency_hz_(frequency_hz) {}

bool Pca9685::allOff() {
    // One register write works even before MODE1 auto-increment is enabled.
    // FULL_OFF applies to all 16 channels, including CH15.
    const std::uint8_t off = 0x10;
    return bus_.write(address_, kAllLedOffHigh, &off, 1);
}

bool Pca9685::offChannel(std::uint8_t channel) {
    if (!ready_ || channel >= 16) return false;
    // LEDn_OFF_H bit 4 is FULL_OFF for this channel only. Do not touch the
    // ALL_LED_OFF_H broadcast register or any other channel's PWM registers.
    const std::uint8_t off = 0x10;
    const std::uint8_t off_high = static_cast<std::uint8_t>(
        kLed0OnLow + channel * 4 + 3);
    return bus_.write(address_, off_high, &off, 1);
}

bool Pca9685::begin() {
    ready_ = false;
    if (frequency_hz_ < 40 || frequency_hz_ > 1000) return false;

    std::uint8_t mode1 = 0;
    if (!bus_.read(address_, kMode1, &mode1, 1)) return false;
    if (!allOff()) return false;

    // Totem-pole output; MODE1 AI permits the four-byte channel writes.
    const std::uint8_t mode2 = 0x04;
    if (!bus_.write(address_, kMode2, &mode2, 1)) return false;

    const std::uint32_t divisor = 4096UL * frequency_hz_;
    const std::uint32_t prescale = (kOscillatorHz + divisor / 2) / divisor - 1;
    if (prescale < 3 || prescale > 255) return false;

    const std::uint8_t asleep = static_cast<std::uint8_t>((mode1 & 0x7F) | 0x10);
    const std::uint8_t scale = static_cast<std::uint8_t>(prescale);
    const std::uint8_t awake = static_cast<std::uint8_t>((mode1 & ~0x10) | 0x20);
    const std::uint8_t restart = static_cast<std::uint8_t>(awake | 0x80);
    if (!bus_.write(address_, kMode1, &asleep, 1) ||
        !bus_.write(address_, kPrescale, &scale, 1) ||
        !bus_.write(address_, kMode1, &awake, 1)) return false;
    clock_.delayMs(1);
    if (!bus_.write(address_, kMode1, &restart, 1) || !allOff()) return false;
    ready_ = true;
    return true;
}

bool Pca9685::setPulseUs(std::uint8_t channel, std::uint16_t pulse_us) {
    if (!ready_ || channel >= 16 || pulse_us < 500 || pulse_us > 2500) return false;
    // Round to the nearest 12-bit tick; never interpret an invalid width as
    // FULL_ON or FULL_OFF. The real oscillator must be calibrated on hardware.
    const std::uint32_t ticks = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(pulse_us) * frequency_hz_ * 4096ULL + 500000ULL) /
        1000000ULL);
    if (ticks == 0 || ticks >= 4096) return false;
    const std::uint8_t data[4] = {
        0, 0, static_cast<std::uint8_t>(ticks & 0xFF),
        static_cast<std::uint8_t>((ticks >> 8) & 0x0F)};
    return bus_.write(address_, static_cast<std::uint8_t>(kLed0OnLow + channel * 4),
                      data, sizeof(data));
}

} // namespace dog
