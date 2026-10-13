#pragma once

namespace dog::board {

// Lab-only SoftAP. Change the WPA2 passphrase before field use or sharing a
// firmware image; there is no hardware servo cut-off on Applepy_v1.
constexpr char kDebugApSsid[] = "Applepy-Debug";
constexpr char kDebugApPassword[] = "ApplepyLab-2026";

} // namespace dog::board
