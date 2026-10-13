#include "Ctrl/PyAppleFootTrajectory.h"

#include <cmath>

namespace dog {

bool pyAppleSwingFoot(float u, PyAppleFootPoint& out) {
    if (!std::isfinite(u) || u < 0.0f || u > 1.0f) return false;
    PyAppleFootPoint next{};
    if (u < 0.25f) next.x = 0.0f;
    else if (u < 0.75f)
        next.x = -16.0f*u*u*u + 24.0f*u*u - 9.0f*u + 1.0f;
    else next.x = 1.0f; // Includes the original source's undefined u=.75.
    if (u < 0.5f) next.z = -16.0f*u*u*u + 12.0f*u*u;
    else next.z = 4.0f*u*(1.0f-u);
    if (next.z < 0.0f) next.z = 0.0f;
    out = next;
    return true;
}

bool pyAppleSupportFoot(float u, PyAppleFootPoint& out) {
    if (!std::isfinite(u) || u < 0.0f || u > 1.0f) return false;
    PyAppleFootPoint next{};
    next.x = 1.0f-u;
    out = next;
    return true;
}

}  // namespace dog

