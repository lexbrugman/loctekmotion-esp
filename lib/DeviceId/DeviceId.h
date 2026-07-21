#pragma once

#include <string>

// Helpers for turning free-form captive-portal input into a valid device
// identity. A device_id is the base for the MQTT client id, the HA device
// identifier / discovery topics, the mDNS name, and the Wi-Fi hostname.
//
// sanitizeDeviceId() lower-cases the id in place and reduces it to a safe
// [a-z0-9_] namespace token, so a fat-fingered portal entry becomes usable
// rather than a broken MQTT topic segment.
//
// '_' is legal for MQTT/mDNS but not in a Wi-Fi hostname: the ESP8266 core
// enforces RFC-952 (letters, digits, hyphens only) and rejects the name,
// making WiFiManager log "hostname: set failed!" so the hostname never takes.
// deviceHostname() returns an RFC-952 form of an id, mapping '_' to '-'.
//
// Pure, Arduino-free logic (std::string, not Arduino String), so both are
// exercised by host-side unit tests.

inline void sanitizeDeviceId(char* id) {
  for (char* c = id; *c; ++c) {
    if (*c >= 'A' && *c <= 'Z') *c += 'a' - 'A';
    else if (!((*c >= 'a' && *c <= 'z') || (*c >= '0' && *c <= '9') || *c == '_')) *c = '_';
  }
}

inline std::string deviceHostname(const char* id) {
  std::string host(id);
  for (char& c : host)
    if (c == '_') c = '-';
  return host;
}
