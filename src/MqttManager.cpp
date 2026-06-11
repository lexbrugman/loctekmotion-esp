#include "MqttManager.h"

#include "config.h"

namespace {

// PubSubClient's callback is a plain function pointer, so we bridge it back to
// the single MqttManager instance.
MqttManager* g_self = nullptr;

// Shared Home Assistant "device" object so all entities group under one device.
// id/name come from the provisioned identity (see DeviceConfig) so multiple
// devices on the same broker each get their own HA device entry.
String deviceBlock(const char* device_id, const char* device_name) {
  String d = "\"device\":{\"identifiers\":[\"";
  d += device_id;
  d += "\"],\"name\":\"";
  d += device_name;
  d += "\",\"manufacturer\":\"";
  d += cfg::kManufacturer;
  d += "\",\"model\":\"";
  d += cfg::kModel;
  d += "\",\"sw_version\":\"";
  d += cfg::kSwVersion;
  d += "\"}";
  return d;
}

}  // namespace

void MqttManager::begin(const DeviceConfig& config, CommandHandler handler) {
  config_ = config;
  handler_ = std::move(handler);
  g_self = this;
  // <kTopicNamespace>/<device id> — keyed off the provisioned identity so
  // each device gets its own MQTT namespace (see DeviceConfig::device_id).
  base_ = String(cfg::kTopicNamespace) + "/" + config_.device_id;
  avail_top_ = base_ + "/status";
  cmd_prefix_ = base_ + "/cmd/";
  height_top_ = base_ + "/height";
  position_top_ = base_ + "/position";
  target_top_ = base_ + "/target";
  wifi_top_ = base_ + "/wifi";
  uptime_top_ = base_ + "/uptime";
  log_top_ = base_ + "/log";
  ota_channel_top_ = base_ + "/ota_channel";
  childlock_top_ = base_ + "/childlock";
  movement_avail_top_ = base_ + "/movement_available";
  calibration_top_ = base_ + "/calibration";

  mqtt_.setServer(config_.mqtt_host, config_.mqtt_port);
  mqtt_.setBufferSize(1024);  // discovery payloads exceed the 256 default
  mqtt_.setKeepAlive(30);
  mqtt_.setCallback([](char* topic, uint8_t* payload, unsigned int len) {
    if (g_self) g_self->onMessage(topic, payload, len);
  });
}

void MqttManager::loop() {
  if (mqtt_.connected()) {
    mqtt_.loop();
    return;
  }
  const uint32_t now = millis();
  if (!first_attempt_ && now - last_attempt_ < cfg::kMqttRetryInterval) return;
  first_attempt_ = false;
  last_attempt_ = now;
  reconnect();
}

bool MqttManager::reconnect() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!config_.hasBroker()) return false;  // not provisioned yet

  const char* user = config_.mqtt_user[0] ? config_.mqtt_user : nullptr;
  const char* pass = config_.mqtt_pass[0] ? config_.mqtt_pass : nullptr;

  // Last Will: broker marks us offline if we drop unexpectedly.
  const bool ok = mqtt_.connect(config_.device_id, user, pass, avail_top_.c_str(),
                                0, true, "offline");
  if (ok) announce();
  return ok;
}

void MqttManager::announce() {
  mqtt_.publish(avail_top_.c_str(), "online", true);
  mqtt_.subscribe((base_ + "/cmd/#").c_str());

  // --- Sensors ---
  publishDiscoveryEntity(
      "sensor", "height", "Height",
      "\"state_topic\":\"" + base_ +
          "/height\",\"unit_of_measurement\":\"cm\",\"device_class\":\"distance\","
          "\"state_class\":\"measurement\",\"suggested_display_precision\":1,"
          "\"icon\":\"mdi:desk\"");
  publishDiscoveryEntity(
      "sensor", "wifi_signal", "WiFi signal",
      "\"state_topic\":\"" + base_ +
          "/wifi\",\"unit_of_measurement\":\"dBm\",\"device_class\":\"signal_"
          "strength\",\"state_class\":\"measurement\",\"entity_category\":"
          "\"diagnostic\"");
  publishDiscoveryEntity(
      "sensor", "uptime", "Uptime",
      "\"state_topic\":\"" + base_ +
          "/uptime\",\"unit_of_measurement\":\"s\",\"device_class\":\"duration\","
          "\"state_class\":\"total_increasing\",\"entity_category\":"
          "\"diagnostic\",\"icon\":\"mdi:timer-outline\"");
  publishDiscoveryEntity(
      "sensor", "ota_channel", "OTA channel",
      "\"state_topic\":\"" + base_ +
          "/ota_channel\",\"entity_category\":\"diagnostic\",\"icon\":"
          "\"mdi:source-branch\"");
  // Constant for the life of the build, so publish it once here rather than
  // from main.cpp's periodic telemetry loop (retained, like availability).
  mqtt_.publish(ota_channel_top_.c_str(), cfg::kOtaChannel, true);

  // --- Motion calibration (learned kinematics; see MotionModel) ---
  // Six diagnostic sensors fed from one retained JSON topic, for watching the
  // per-desk calibration converge. Off by default in HA — opt-in via the
  // entity registry, since they're debugging aids rather than daily-use state.
  struct Cal { const char* key; const char* name; const char* unit; };
  static const Cal cal_sensors[] = {
      {"terminal_speed_up", "Cruise speed up", "cm/s"},
      {"terminal_speed_down", "Cruise speed down", "cm/s"},
      {"decel_up", "Coast deceleration up", "cm/s²"},
      {"decel_down", "Coast deceleration down", "cm/s²"},
      {"tap_gain_up", "Tap gain up", "cm/s²"},
      {"tap_gain_down", "Tap gain down", "cm/s²"},
  };
  for (const auto& c : cal_sensors) {
    publishDiscoveryEntity(
        "sensor", c.key, c.name,
        "\"state_topic\":\"" + calibration_top_ + "\",\"value_template\":\"{{ value_json." +
            c.key + " }}\",\"unit_of_measurement\":\"" + c.unit +
            "\",\"state_class\":\"measurement\",\"suggested_display_precision\":2,"
            "\"entity_category\":\"diagnostic\",\"enabled_by_default\":false,"
            "\"icon\":\"mdi:tune-variant\"");
  }

  // --- Cover (position-based) ---
  // Gated on movement_avail_top_: the desk ignores movement commands while
  // child-locked, so grey this out rather than let it silently do nothing.
  // name:null + has_entity_name: this entity *is* the device, so it should
  // show up as just the device's name (e.g. "Study desk") rather than
  // "<device name> Desk".
  publishDiscoveryEntity(
      "cover", "desk", nullptr,
      "\"has_entity_name\":true,\"command_topic\":\"" + base_ +
          "/cmd/cover\",\"payload_open\":\"OPEN\","
      "\"payload_close\":\"CLOSE\",\"payload_stop\":\"STOP\",\"position_topic\":\"" +
          base_ + "/position\",\"set_position_topic\":\"" + base_ +
          "/cmd/position\",\"position_open\":100,\"position_closed\":0,"
          "\"device_class\":\"blind\",\"icon\":\"mdi:desk\"",
      movement_avail_top_);

  // --- Number (absolute target height) ---
  publishDiscoveryEntity(
      "number", "target_height", "Target height",
      "\"command_topic\":\"" + base_ + "/cmd/target\",\"state_topic\":\"" +
          base_ + "/target\",\"min\":" + String(cfg::kMinHeight, 1) +
          ",\"max\":" + String(cfg::kMaxHeight, 1) +
          ",\"step\":0.1,\"unit_of_measurement\":\"cm\",\"device_class\":"
          "\"distance\",\"mode\":\"box\",\"icon\":\"mdi:arrow-up-down\"",
      movement_avail_top_);

  // --- Switches ---
  publishDiscoveryEntity(
      "switch", "childlock", "Child lock",
      "\"command_topic\":\"" + base_ + "/cmd/childlock\",\"state_topic\":\"" +
          base_ +
          "/childlock\",\"icon\":\"mdi:account-lock\",\"entity_category\":"
          "\"config\"");
  // Was previously a button; clear its retained discovery config so it
  // doesn't linger as a duplicate entity.
  mqtt_.publish((String(cfg::kDiscoveryPrefix) + "/button/" + config_.device_id +
                "/childlock/config")
                    .c_str(),
                "", true);

  // --- Buttons ---
  // movement: true for buttons that issue movement commands the desk ignores
  // while child-locked (gated on movement_avail_top_, like the cover/number).
  struct Btn { const char* obj; const char* name; const char* icon; bool cfg_cat; bool movement; };
  static const Btn buttons[] = {
      {"preset1", "Preset 1", "mdi:numeric-1-box", false, true},
      {"preset2", "Preset 2", "mdi:numeric-2-box", false, true},
      {"sit", "Sit", "mdi:chair-rolling", false, true},
      {"stand", "Stand", "mdi:human-handsup", false, true},
      {"memory", "Memory", "mdi:alpha-m-box", true, true},
      {"alarm", "Alarm", "mdi:alarm", false, true},
      {"wake", "Wake screen", "mdi:gesture-tap-button", true, false},
      {"calibration_reset", "Reset calibration", "mdi:restore", true, false},
      {"update", "Firmware update", "mdi:cloud-download", true, false},
      {"configure", "Wi-Fi setup", "mdi:wifi-cog", true, false},
      {"restart", "Restart", "mdi:restart", true, false},
  };
  for (const auto& b : buttons) {
    String spec = "\"command_topic\":\"" + base_ + "/cmd/" + b.obj +
                  "\",\"payload_press\":\"PRESS\",\"icon\":\"" + b.icon + "\"";
    if (b.cfg_cat) spec += ",\"entity_category\":\"config\"";
    publishDiscoveryEntity("button", b.obj, b.name, spec,
                           b.movement ? movement_avail_top_ : String());
  }
}

void MqttManager::publishDiscoveryEntity(const char* component, const char* object,
                                         const char* name, const String& specifics,
                                         const String& extra_availability_topic) {
  String topic = String(cfg::kDiscoveryPrefix) + "/" + component + "/" +
                 config_.device_id + "/" + object + "/config";

  // name == nullptr -> "name":null, so this entity (the device's primary
  // feature) takes the device's own name in the UI instead of its own.
  String payload = "{\"name\":";
  payload += name ? (String("\"") + name + "\"") : String("null");
  payload += ",\"unique_id\":\"";
  payload += config_.device_id;
  payload += "_";
  payload += object;
  payload += "\",";
  if (extra_availability_topic.length()) {
    // Available only while both the device overall and (e.g.) movement
    // commands are.
    payload += "\"availability_mode\":\"all\",\"availability\":[{\"topic\":\"";
    payload += avail_top_;
    payload += "\"},{\"topic\":\"";
    payload += extra_availability_topic;
    payload += "\"}],";
  } else {
    payload += "\"availability_topic\":\"";
    payload += avail_top_;
    payload += "\",";
  }
  payload += deviceBlock(config_.device_id, config_.device_name);
  payload += ",";
  payload += specifics;
  payload += "}";

  mqtt_.publish(topic.c_str(), payload.c_str(), true);
}

void MqttManager::onMessage(char* topic, const uint8_t* payload, unsigned int length) {
  if (!handler_) return;
  String t(topic);
  if (!t.startsWith(cmd_prefix_)) return;
  String object = t.substring(cmd_prefix_.length());

  // concat(ptr, length) is binary-safe; the String(const char*) constructor
  // would assume a null-terminated buffer and isn't appropriate here.
  String msg;
  msg.reserve(length);
  msg.concat(reinterpret_cast<const char*>(payload), length);

  handler_(object, msg);
}

// --- Publishers ---
void MqttManager::publishHeight(float cm) {
  if (mqtt_.connected())
    mqtt_.publish(height_top_.c_str(), String(cm, 1).c_str(), true);
}
void MqttManager::publishPosition(int percent) {
  if (mqtt_.connected())
    mqtt_.publish(position_top_.c_str(), String(percent).c_str(), true);
}
void MqttManager::publishTarget(float cm) {
  if (mqtt_.connected())
    mqtt_.publish(target_top_.c_str(), String(cm, 1).c_str(), true);
}
void MqttManager::publishWifi(int rssi) {
  if (mqtt_.connected())
    mqtt_.publish(wifi_top_.c_str(), String(rssi).c_str(), true);
}
void MqttManager::publishUptime(unsigned long seconds) {
  if (mqtt_.connected())
    mqtt_.publish(uptime_top_.c_str(), String(seconds).c_str(), true);
}
void MqttManager::publishLog(const String& message) {
  if (mqtt_.connected()) mqtt_.publish(log_top_.c_str(), message.c_str());
}
void MqttManager::publishChildLock(bool locked) {
  if (mqtt_.connected())
    mqtt_.publish(childlock_top_.c_str(), locked ? "ON" : "OFF", true);
}
void MqttManager::publishMovementAvailable(bool available) {
  if (mqtt_.connected())
    mqtt_.publish(movement_avail_top_.c_str(), available ? "online" : "offline", true);
}
void MqttManager::publishCalibration(const MotionModel::State& s) {
  if (!mqtt_.connected()) return;
  String p;
  p.reserve(192);
  p += "{\"terminal_speed_up\":";
  p += String(s.terminal_speed_up, 3);
  p += ",\"terminal_speed_down\":";
  p += String(s.terminal_speed_down, 3);
  p += ",\"decel_up\":";
  p += String(s.decel_up, 3);
  p += ",\"decel_down\":";
  p += String(s.decel_down, 3);
  p += ",\"tap_gain_up\":";
  p += String(s.tap_gain_up, 3);
  p += ",\"tap_gain_down\":";
  p += String(s.tap_gain_down, 3);
  p += "}";
  mqtt_.publish(calibration_top_.c_str(), p.c_str(), true);
}
