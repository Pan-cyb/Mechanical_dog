#pragma once

namespace dog {

struct PyAppleFootPoint {
    float x = 0.0f; // Normalized 0..1 progression.
    float z = 0.0f; // Normalized lift 0..1.
};

// Behavioral port of padog.py swing_curve_generate/support_curve_generate
// from the bundled Py-Apple V7.3 reference. The reference repository is
// treated as GPL-3.0; its file-level license text is inconsistent, documented
// in docs/ORIGINAL_V73_PARAMETERS.md. The u=.75 hole is filled by its
// continuous x=1 limit.
bool pyAppleSwingFoot(float u, PyAppleFootPoint& out);
bool pyAppleSupportFoot(float u, PyAppleFootPoint& out);

}  // namespace dog

