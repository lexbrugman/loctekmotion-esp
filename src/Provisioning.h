#pragma once

#include "ConfigStore.h"

// First-boot / recovery provisioning: the WiFiManager captive portal that
// collects the Wi-Fi credentials plus the MQTT and device-identity settings,
// and the double-reset gesture that forces a way back into it.
namespace provisioning {

// Double-reset detection: pressing the board's RST button twice within
// cfg::kDoubleResetWindow of booting is a way back into the captive portal
// that doesn't depend on MQTT/HA (e.g. after a config mistake locks the device
// out of its broker); every board has a reset button. Call first thing in
// setup(), before run(): on a double reset the Wi-Fi credentials are cleared
// *now*, so run() raises the portal this boot. A power cycle, or the firmware
// restarting itself, never counts.
void checkDoubleReset();

// Blocking, runs once at boot: load the stored config into |config|, connect
// to Wi-Fi (raising the captive portal to collect Wi-Fi + MQTT settings on
// first boot) and persist any changes. Reboots if the portal times out.
void run(DeviceConfig& config);

// Forget the stored Wi-Fi credentials, so the next boot raises the portal.
void clearWifiSettings();

}  // namespace provisioning
