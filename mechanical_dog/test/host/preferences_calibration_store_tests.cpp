#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include <Preferences.h>

#include "Ctrl/ChannelCalibration.h"
#include "Ctrl/ServoController.h"
#include "Port/PreferencesCalibrationStore.h"

namespace {

class Clock final : public dog::IClock {
public:
    std::uint32_t millis() const override { return 0U; }
    void delayMs(std::uint32_t) override {}
};

class Pwm final : public dog::IServoPwm {
public:
    bool begin() override { return true; }
    bool allOff() override {
        ++off_count;
        return true;
    }
    bool offChannel(std::uint8_t) override { return true; }
    bool setPulseUs(std::uint8_t, std::uint16_t) override {
        ++output_count;
        return true;
    }

    unsigned off_count = 0U;
    unsigned output_count = 0U;
};

constexpr std::uint8_t kNewChannel = 12U;
constexpr std::uint8_t kLegacyChannel = 8U;
constexpr std::uint8_t kChannels[] = {kNewChannel};

void testPrimaryKeyAlwaysWins() {
    preferences_stub::reset();
    preferences_stub::state.values["c12"] = 101U;
    preferences_stub::state.values["c8"] = 77U;
    dog::PreferencesCalibrationStore store(kNewChannel, kLegacyChannel);
    assert(store.begin());
    std::uint8_t center = 0U;
    assert(store.load(kNewChannel, center) && center == 101U);
    assert(preferences_stub::state.get_count["c12"] == 1U);
    assert(preferences_stub::state.get_count["c8"] == 0U);
    assert(preferences_stub::state.put_count == 0U);
}

void testMissingPrimaryReadsLegacyWithoutWriting() {
    preferences_stub::reset();
    preferences_stub::state.values["c8"] = 88U;
    dog::PreferencesCalibrationStore store(kNewChannel, kLegacyChannel);
    assert(store.begin());
    std::uint8_t center = 0U;
    assert(store.load(kNewChannel, center) && center == 88U);
    assert(preferences_stub::state.get_count["c8"] == 1U);
    assert(preferences_stub::state.put_count == 0U);
    assert(preferences_stub::state.values.count("c12") == 0U);
}

void testInvalidPrimaryDoesNotFallBack() {
    preferences_stub::reset();
    preferences_stub::state.values["c12"] = 255U;
    preferences_stub::state.values["c8"] = 88U;
    dog::PreferencesCalibrationStore store(kNewChannel, kLegacyChannel);
    assert(store.begin());
    Clock clock;
    Pwm pwm;
    dog::ServoController servos(pwm, clock, 500U, 2500U, 0U, kChannels, 1U,
                                500U, 2500U);
    assert(servos.begin());
    dog::ChannelCalibration calibration(store, servos, kChannels, 1U, 500U,
                                        2500U);
    calibration.begin();
    assert(calibration.centerDeg(kNewChannel) == 90U);
    assert(preferences_stub::state.get_count["c12"] == 1U);
    assert(preferences_stub::state.get_count["c8"] == 0U);
    assert(preferences_stub::state.put_count == 0U && pwm.output_count == 0U);
}

void testMissingOrInvalidLegacyUsesDefault() {
    for (const bool legacy_present : {false, true}) {
        preferences_stub::reset();
        if (legacy_present) preferences_stub::state.values["c8"] = 250U;
        dog::PreferencesCalibrationStore store(kNewChannel, kLegacyChannel);
        assert(store.begin());
        Clock clock;
        Pwm pwm;
        dog::ServoController servos(pwm, clock, 500U, 2500U, 0U, kChannels,
                                    1U, 500U, 2500U);
        assert(servos.begin());
        dog::ChannelCalibration calibration(store, servos, kChannels, 1U,
                                            500U, 2500U);
        calibration.begin();
        assert(calibration.centerDeg(kNewChannel) == 90U);
        assert(preferences_stub::state.put_count == 0U &&
               pwm.output_count == 0U);
    }
}

void testExplicitSaveWritesOnlyNewKey() {
    preferences_stub::reset();
    preferences_stub::state.values["c8"] = 77U;
    dog::PreferencesCalibrationStore store(kNewChannel, kLegacyChannel);
    assert(store.begin());
    Clock clock;
    Pwm pwm;
    dog::ServoController servos(pwm, clock, 500U, 2500U, 0U, kChannels, 1U,
                                500U, 2500U);
    assert(servos.begin());
    dog::ChannelCalibration calibration(store, servos, kChannels, 1U, 500U,
                                        2500U);
    calibration.begin();
    assert(calibration.centerDeg(kNewChannel) == 77U);
    assert(preferences_stub::state.put_count == 0U && pwm.output_count == 0U);
    assert(calibration.save(kNewChannel) == dog::CalibrationResult::Ok);
    assert(preferences_stub::state.values["c12"] == 77U);
    assert(preferences_stub::state.values["c8"] == 77U);
    assert(preferences_stub::state.put_count == 1U);
    assert(preferences_stub::state.put_key_count["c12"] == 1U);
    assert(preferences_stub::state.put_key_count["c8"] == 0U);
}

void testFallbackIsNotAppliedToOtherChannels() {
    preferences_stub::reset();
    preferences_stub::state.values["c8"] = 73U;
    dog::PreferencesCalibrationStore store(kNewChannel, kLegacyChannel);
    assert(store.begin());
    std::uint8_t center = 42U;
    assert(!store.load(7U, center) && center == 42U);
    assert(preferences_stub::state.get_count["c8"] == 0U);
}

}  // namespace

int main() {
    testPrimaryKeyAlwaysWins();
    testMissingPrimaryReadsLegacyWithoutWriting();
    testInvalidPrimaryDoesNotFallBack();
    testMissingOrInvalidLegacyUsesDefault();
    testExplicitSaveWritesOnlyNewKey();
    testFallbackIsNotAppliedToOtherChannels();
    std::cout << "preferences calibration store tests passed\n";
    return 0;
}

