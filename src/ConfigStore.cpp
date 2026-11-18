#include "ConfigStore.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "JsonFile.h"

namespace {
constexpr char kPath[] = "/config.json";
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
  JsonDocument doc;
  if (!json_file::load(kPath, doc)) return false;

  setField(out.mqtt_host, doc["mqtt_host"] | "");
  out.mqtt_port = doc["mqtt_port"] | out.mqtt_port;  // DeviceConfig's default
  setField(out.mqtt_user, doc["mqtt_user"] | "");
  setField(out.mqtt_pass, doc["mqtt_pass"] | "");
  setField(out.device_id, doc["device_id"] | "");
  setField(out.device_name, doc["device_name"] | "");
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
  return json_file::save(kPath, doc);
}

}  // namespace config_store
