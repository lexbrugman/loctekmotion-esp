#pragma once

// Concentrates the handful of places where the ESP8266 and ESP32 Arduino
// cores diverge — header paths and class/global names for WiFi, HTTP OTA
// updates, and the TLS client — behind one small, board-agnostic surface.
// The rest of the firmware includes this (instead of the cores' headers
// directly) and never needs to know which board it's running on.

#include <memory>

#if defined(ESP8266)
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecureBearSSL.h>
#elif defined(ESP32)
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#else
#error "Unsupported platform: add WiFi/HTTPUpdate/TLS aliases for it in Platform.h"
#endif

namespace platform {

#if defined(ESP8266)
using SecureClient = BearSSL::WiFiClientSecure;
#else
using SecureClient = ::WiFiClientSecure;
#endif

// The global OTA-update driver. Same update()/rebootOnUpdate()/... API on
// both cores, just under a different name (ESPhttpUpdate vs httpUpdate) —
// `auto&` sidesteps spelling out the (also-different) concrete class names.
inline auto& updater() {
#if defined(ESP8266)
  return ESPhttpUpdate;
#else
  return httpUpdate;
#endif
}

// Keep the MQTT connection open while an OTA update runs, so HA keeps the
// device available and receives the install progress. The ESP8266 driver
// otherwise closes every other socket when the download starts (to free heap),
// which makes the broker publish our "offline" last will; the ESP32 driver
// leaves them alone.
inline void keepConnectionsDuringUpdate() {
#if defined(ESP8266)
  ESPhttpUpdate.closeConnectionsOnUpdate(false);
#endif
}

// A TLS client for fetching OTA assets from GitHub. Certificate validation
// is skipped (setInsecure) — no CA store is bundled with the firmware — so
// the connection is encrypted but the server is not authenticated.
std::unique_ptr<SecureClient> makeSecureClient();

// Seconds since boot, from a 64-bit microsecond clock so it doesn't wrap the
// way the 32-bit millis() does after 49.7 days.
uint64_t uptimeSeconds();

// A single flag in RTC memory: it survives a reset but, unlike flash, not a
// power loss (for provisioning::checkDoubleReset). The cores differ in how
// (ESP.rtcUserMemory* vs. an RTC_NOINIT_ATTR variable); this hides that.
bool readResetFlag();
void writeResetFlag(bool set);

// True when this boot could have been caused by the reset button rather than
// the firmware restarting itself (ESP.restart(), a crash, a watchdog). The
// ESP32's EN button reads back as a power-on, so power-ons count too — the
// RTC flag above already tells those apart.
bool resetByButton();

}  // namespace platform
