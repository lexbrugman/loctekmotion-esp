#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

// Runtime device configuration entered via the captive portal and persisted to
// LittleFS. Wi-Fi credentials themselves are stored by the ESP SDK / WiFiManager;
// this holds the MQTT broker details plus the device identity (letting more
// than one of these run against the same broker/HA instance — see config.h's
// kDeviceId/kDeviceName for the factory defaults used while these are unset).
struct DeviceConfig {
  char mqtt_host[64] = "";
  uint16_t mqtt_port = 1883;
  char mqtt_user[40] = "";
  char mqtt_pass[40] = "";
  char device_id[24] = "";
  char device_name[40] = "";

  bool hasBroker() const { return mqtt_host[0] != '\0'; }
};

namespace config_store {

// Copy |value| into a fixed-size DeviceConfig field (truncating, always
// terminated), or |fallback| if |value| is null or empty.
template <size_t N>
void setField(char (&field)[N], const char* value, const char* fallback = "") {
  strncpy(field, value && value[0] ? value : fallback, N - 1);
  field[N - 1] = '\0';
}

// Mount LittleFS (formats on first use). Returns false on failure.
bool begin();

// Load /config.json into out. Returns false if absent/invalid (out keeps
// its defaults).
bool load(DeviceConfig& out);

// Persist config as /config.json. Returns false on write failure.
bool save(const DeviceConfig& cfg);

}  // namespace config_store
