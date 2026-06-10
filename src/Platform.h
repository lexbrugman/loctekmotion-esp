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

// A TLS client suitable for fetching OTA assets from GitHub. Certificate
// validation is intentionally skipped (see UpdateManager.cpp for why).
std::unique_ptr<SecureClient> makeSecureClient();

// A single persistent flag backed by RTC memory: it survives a software or
// external (RST-button) reset, but — unlike flash-backed storage — is wiped
// by a power loss. That's exactly the distinction main.cpp's double-reset
// detection needs to tell a deliberate double press apart from someone
// simply power-cycling the desk (see config.h's kDoubleResetWindow). The
// underlying mechanism (ESP.rtcUserMemory* vs. an RTC_NOINIT_ATTR variable)
// differs between the cores; this hides that behind a plain bool.
bool readResetFlag();
void writeResetFlag(bool set);

}  // namespace platform
