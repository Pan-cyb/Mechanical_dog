#pragma once

#include "Ctrl/ServoController.h"

namespace dog {

// A register-range experiment pauses normal IMU polling for ~200 ms. It may
// only run when no PWM output is active and the last software all-off was
// acknowledged. This does not replace physically disconnecting servo power.
inline bool canRunImuRangeTest(const ServoController& servos, bool imu_fresh) {
    return imu_fresh && servos.ready() && !servos.faulted() &&
           !servos.calibrating() && servos.calibrationMask() == 0 &&
           !servos.armed() && servos.activeChannel() == -1;
}

} // namespace dog
