#include "ConfigStore.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

namespace {
constexpr char kPath[] = "/config.json";

void copyStr(char* dst, size_t cap, const char* src) {
  strncpy(dst, src ? src : "", cap - 1);
  dst[cap - 1] = '\0';
}
}  // namespace

namespace config_store {

bool begin() {
#if defined(ESP8266)
  return LittleFS.begin();  // formats automatically on first mount
#else
  return LittleFS.begin(/*formatOnFail=*/true);  // ESP32's default is false
#endif
}

bool load(DeviceConfig& out) {
  File f = LittleFS.open(kPath, "r");
  if (!f) return false;

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  copyStr(out.mqtt_host, sizeof(out.mqtt_host), doc["mqtt_host"] | "");
  out.mqtt_port = doc["mqtt_port"] | 1883;
  copyStr(out.mqtt_user, sizeof(out.mqtt_user), doc["mqtt_user"] | "");
  copyStr(out.mqtt_pass, sizeof(out.mqtt_pass), doc["mqtt_pass"] | "");
  copyStr(out.device_id, sizeof(out.device_id), doc["device_id"] | "");
  copyStr(out.device_name, sizeof(out.device_name), doc["device_name"] | "");
  return true;
}

bool save(const DeviceConfig& cfg) {
  JsonDocument doc;
  doc["mqtt_host"] = cfg.mqtt_host;
  doc["mqtt_port"] = cfg.mqtt_port;
  doc["mqtt_user"] = cfg.mqtt_user;
  doc["mqtt_pass"] = cfg.mqtt_pass;
  doc["device_id"] = cfg.device_id;
  doc["device_name"] = cfg.device_name;

  File f = LittleFS.open(kPath, "w");
  if (!f) return false;
  const bool ok = serializeJson(doc, f) > 0;
  f.close();
  return ok;
}

}  // namespace config_store
