#include "MqttManager.h"

#include <cmath>

#include "config.h"

namespace {

// PubSubClient's callback is a plain function pointer, so we bridge it back to
// the single MqttManager instance.
MqttManager* g_self = nullptr;

// The learned motion calibration, published as one retained JSON object (see
// publishCalibration) and exposed as one diagnostic sensor per key. Sharing
// this table keeps the JSON keys and the sensors' value_templates in step.
// unit_suffix is appended to kHeightUnit so every height-derived unit traces
// back to the one constant (cm -> cm/s, cm/s²).
struct CalibrationSensor {
  const char* key;
  const char* name;
  const char* unit_suffix;
  float MotionModel::State::*value;
};
constexpr CalibrationSensor kCalibrationSensors[] = {
    {"terminal_speed_up", "Cruise speed up", "/s", &MotionModel::State::terminal_speed_up},
    {"terminal_speed_down", "Cruise speed down", "/s", &MotionModel::State::terminal_speed_down},
    {"decel_up", "Coast deceleration up", "/s²", &MotionModel::State::decel_up},
    {"decel_down", "Coast deceleration down", "/s²", &MotionModel::State::decel_down},
    {"tap_gain_up", "Tap gain up", "/s²", &MotionModel::State::tap_gain_up},
    {"tap_gain_down", "Tap gain down", "/s²", &MotionModel::State::tap_gain_down},
};

// Suffixes of MqttManager::Topic, in enum order.
constexpr const char* kTopicSuffixes[] = {
    "height", "target",    "fw_latest", "fw_installed",       "wifi",        "uptime",
    "log",    "childlock", "alarm",     "movement_available", "calibration",
};

}  // namespace

void MqttManager::begin(const DeviceConfig& config, CommandHandler handler) {
  config_ = config;
  handler_ = std::move(handler);
  g_self = this;
  // <kTopicNamespace>/<device id> — keyed off the provisioned identity so
  // each device gets its own MQTT namespace (see DeviceConfig::device_id).
  const String base = String(cfg::kTopicNamespace) + "/" + config_.device_id;
  avail_top_ = base + "/status";
  cmd_prefix_ = base + "/cmd/";
  static_assert(sizeof(kTopicSuffixes) / sizeof(kTopicSuffixes[0]) ==
                    static_cast<size_t>(Topic::kCount),
                "one suffix per Topic");
  for (size_t i = 0; i < static_cast<size_t>(Topic::kCount); i++) {
    topics_[i] = base + "/" + kTopicSuffixes[i];
  }

  mqtt_.setServer(config_.mqtt_host, config_.mqtt_port);
  mqtt_.setBufferSize(1024);  // discovery payloads exceed the 256 default
  mqtt_.setKeepAlive(30);
  mqtt_.setCallback([](char* topic, uint8_t* payload, unsigned int len) {
    if (g_self) g_self->onMessage(topic, payload, len);
  });
}

void MqttManager::loop(bool allow_reconnect) {
  if (mqtt_.connected()) {
    mqtt_.loop();
    publishUptimeIfDue();
    return;
  }
  if (!allow_reconnect) return;
  const uint32_t now = millis();
  if (retry_.armed() && !retry_.due(now)) return;
  retry_.arm(now, cfg::kMqttRetryInterval);
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
  published_.clear();
  mqtt_.publish(avail_top_.c_str(), "online", true);
  mqtt_.subscribe((cmd_prefix_ + "#").c_str());

  // Every entity this firmware defines. Each gets a state and/or command
  // topic, an icon and a category from its row; fill() adds the keys
  // specific to it. movement marks entities that issue movement commands the
  // desk ignores while child-locked: they're gated on the
  // movement-availability topic as well (see publishDiscoveryEntity).
  struct Entity {
    const char* component;
    const char* object;
    const char* name;
    Topic state;          // Topic::kCount = no state topic
    const char* command;  // command-topic suffix, or nullptr
    const char* icon;     // or nullptr
    const char* category;  // "config" / "diagnostic", or nullptr
    bool movement;
    void (*fill)(JsonDocument& doc, const MqttManager& m);
  };
  constexpr Topic kNoState = Topic::kCount;
  const auto press = [](JsonDocument& d, const MqttManager&) { d["payload_press"] = "PRESS"; };
  static const Entity kEntities[] = {
      {"sensor", "height", "Height", Topic::kHeight, nullptr, "mdi:desk", nullptr, false,
       [](JsonDocument& d, const MqttManager&) {
         d["unit_of_measurement"] = cfg::kHeightUnit;
         d["device_class"] = "distance";
         d["state_class"] = "measurement";
         d["suggested_display_precision"] = 1;
       }},
      {"sensor", "wifi_signal", "WiFi signal", Topic::kWifi, nullptr, nullptr, "diagnostic",
       false,
       [](JsonDocument& d, const MqttManager&) {
         d["unit_of_measurement"] = "dBm";
         d["device_class"] = "signal_strength";
         d["state_class"] = "measurement";
       }},
      // The device has no clock, so it publishes its uptime and HA turns that
      // into the moment it restarted. Only sent on connect and when HA
      // restarts (see publishUptimeIfDue), so the state changes only on a
      // restart.
      {"sensor", "last_restart", "Last restart", Topic::kUptime, nullptr, nullptr,
       "diagnostic", false,
       [](JsonDocument& d, const MqttManager&) {
         d["device_class"] = "timestamp";
         d["value_template"] =
             "{{ (now() - timedelta(seconds=value|int)).replace(microsecond=0).isoformat() }}";
       }},
      {"number", "target_height", "Target height", Topic::kTarget, "target",
       "mdi:arrow-up-down", nullptr, true,
       [](JsonDocument& d, const MqttManager&) {
         d["min"] = cfg::kMinHeight;
         d["max"] = cfg::kMaxHeight;
         d["step"] = 0.1;
         d["unit_of_measurement"] = cfg::kHeightUnit;
         d["device_class"] = "distance";
         d["mode"] = "slider";
       }},
      // installed_version is not a discovery-time field for this platform
      // (HA's MQTT update schema silently drops unknown keys) - it has to
      // come from state_topic's JSON payload, published after the table.
      {"update", "firmware", "Firmware", Topic::kFirmwareInstalled, "update", nullptr,
       "config", false,
       [](JsonDocument& d, const MqttManager& m) {
         d["latest_version_topic"] = m.topic(Topic::kFirmwareLatest);
         d["payload_install"] = "PRESS";
         d["device_class"] = "firmware";
         d["title"] = cfg::kOtaChannel;
         d["release_url"] = cfg::kOtaReleaseUrl;
       }},
      {"switch", "childlock", "Child lock", Topic::kChildLock, "childlock",
       "mdi:account-lock", "config", false, nullptr},
      // Sit-stand reminder alarm: the desk reflects it on its display, so its
      // real state is trackable like child lock (see DisplayState).
      {"switch", "alarm", "Alarm", Topic::kAlarm, "alarm", "mdi:alarm", nullptr, true,
       nullptr},
      // Halts a slider seek, or cancels preset/sit/stand travel the desk is
      // making on its own (see DeskMotionPlanner::halt).
      {"button", "stop", "Stop", kNoState, "stop", "mdi:stop", nullptr, true, press},
      {"button", "preset1", "Preset 1", kNoState, "preset1", "mdi:numeric-1-box", nullptr,
       true, press},
      {"button", "preset2", "Preset 2", kNoState, "preset2", "mdi:numeric-2-box", nullptr,
       true, press},
      {"button", "sit", "Sit", kNoState, "sit", "mdi:chair-rolling", nullptr, true, press},
      {"button", "stand", "Stand", kNoState, "stand", "mdi:human-handsup", nullptr, true,
       press},
      {"button", "memory", "Memory", kNoState, "memory", "mdi:alpha-m-box", "config", true,
       press},
      {"button", "wake", "Wake screen", kNoState, "wake", "mdi:gesture-tap-button", "config",
       false, press},
      {"button", "calibration_reset", "Reset calibration", kNoState, "calibration_reset",
       "mdi:restore", "config", false, press},
      // Re-reads the OTA channel's version now instead of waiting for the
      // next periodic poll; refreshes the Firmware entity without installing.
      {"button", "check_update", "Check for updates", kNoState, "check_update",
       "mdi:cloud-search", "config", false, press},
      {"button", "configure", "Wi-Fi setup", kNoState, "configure", "mdi:wifi-cog", "config",
       false, press},
      {"button", "restart", "Restart", kNoState, "restart", "mdi:restart", "config", false,
       press},
  };
  for (const Entity& e : kEntities) {
    JsonDocument doc;
    if (e.state != kNoState) doc["state_topic"] = topic(e.state);
    if (e.command) doc["command_topic"] = String(cmd_prefix_ + e.command);
    if (e.icon) doc["icon"] = e.icon;
    if (e.category) doc["entity_category"] = e.category;
    if (e.fill) e.fill(doc, *this);
    publishDiscoveryEntity(e.component, e.object, e.name, doc,
                           e.movement ? topic(Topic::kMovementAvailable) : String());
  }

  // Motion calibration (learned kinematics; see MotionModel): diagnostic
  // sensors fed from one retained JSON topic (see kCalibrationSensors), for
  // watching the per-desk calibration converge. Off by default in HA — opt-in
  // via the entity registry, since they're debugging aids rather than
  // daily-use state.
  for (const auto& c : kCalibrationSensors) {
    JsonDocument doc;
    doc["state_topic"] = topic(Topic::kCalibration);
    doc["value_template"] = String(String("{{ value_json.") + c.key + " }}");
    doc["unit_of_measurement"] = String(String(cfg::kHeightUnit) + c.unit_suffix);
    doc["state_class"] = "measurement";
    doc["suggested_display_precision"] = 2;
    doc["entity_category"] = "diagnostic";
    doc["enabled_by_default"] = false;
    doc["icon"] = "mdi:tune-variant";
    publishDiscoveryEntity("sensor", c.key, c.name, doc);
  }

  // The installed version is constant for the life of the build, so publish
  // it once here (retained, like availability) rather than periodically.
  publishFirmwareState();

  // After every publishDiscoveryEntity() above: subscribing replays all
  // retained configs under this node ID, and onDiscoveryMessage() deletes
  // the ones not in published_.
  mqtt_.subscribe((String(cfg::kDiscoveryPrefix) + "/+/" + config_.device_id +
                   "/+/config").c_str());
  // HA's birth message, see onDiscoveryMessage().
  mqtt_.subscribe((String(cfg::kDiscoveryPrefix) + "/status").c_str());

  uptime_countdown_.arm(millis(), cfg::kHaSettleDelay);

  if (connect_handler_) connect_handler_();
}

void MqttManager::publishDiscoveryEntity(const char* component, const char* object,
                                         const char* name, JsonDocument& doc,
                                         const String& extra_availability_topic) {
  String topic = String(cfg::kDiscoveryPrefix) + "/" + component + "/" +
                 config_.device_id + "/" + object + "/config";

  // name == nullptr -> "name":null, so this entity (the device's primary
  // feature) takes the device's own name in the UI instead of its own.
  if (name) {
    doc["name"] = name;
  } else {
    doc["name"] = nullptr;
  }
  doc["unique_id"] = String(String(config_.device_id) + "_" + object);
  if (extra_availability_topic.length()) {
    // Available only while both the device overall and (e.g.) movement
    // commands are.
    doc["availability_mode"] = "all";
    JsonArray availability = doc["availability"].to<JsonArray>();
    availability.add<JsonObject>()["topic"] = avail_top_;
    availability.add<JsonObject>()["topic"] = extra_availability_topic;
  } else {
    doc["availability_topic"] = avail_top_;
  }

  // Shared Home Assistant "device" object so all entities group under one
  // device. id/name come from the provisioned identity (see DeviceConfig) so
  // multiple devices on the same broker each get their own HA device entry.
  JsonObject device = doc["device"].to<JsonObject>();
  device["identifiers"].to<JsonArray>().add(config_.device_id);
  device["name"] = config_.device_name;
  device["manufacturer"] = cfg::kManufacturer;
  device["model"] = cfg::kModel;
  device["sw_version"] = cfg::kSwVersion;

  String payload;
  serializeJson(doc, payload);
  if (!mqtt_.publish(topic.c_str(), payload.c_str(), true)) {
    Serial.printf("[mqtt] discovery publish failed: %s\n", topic.c_str());
  }
  published_.push_back(String(component) + "/" + object);
}

void MqttManager::onMessage(char* topic, const uint8_t* payload, unsigned int length) {
  String t(topic);
  const String discovery_prefix = String(cfg::kDiscoveryPrefix) + "/";
  if (t.startsWith(discovery_prefix)) {
    onDiscoveryMessage(t.substring(discovery_prefix.length()), payload, length);
    return;
  }

  if (!handler_) return;
  if (!t.startsWith(cmd_prefix_)) return;
  String object = t.substring(cmd_prefix_.length());

  // concat(ptr, length) is binary-safe; the String(const char*) constructor
  // would assume a null-terminated buffer and isn't appropriate here.
  String msg;
  msg.reserve(length);
  msg.concat(reinterpret_cast<const char*>(payload), length);

  handler_(object, msg);
}

void MqttManager::onDiscoveryMessage(const String& subtopic, const uint8_t* payload,
                                     unsigned int length) {
  if (subtopic == "status") {
    // HA's birth message: it has (re)started and lost the restart time.
    if (length == 6 && memcmp(payload, "online", 6) == 0) {
      uptime_countdown_.arm(millis(), cfg::kHaSettleDelay);
    }
    return;
  }

  // An empty config is a deletion, possibly our own.
  if (length == 0) return;

  // <component>/<node id>/<object>/config
  const int node_start = subtopic.indexOf('/') + 1;
  const int object_start = subtopic.indexOf('/', node_start) + 1;
  const int config_start = subtopic.indexOf('/', object_start);
  if (node_start <= 0 || object_start <= 0 || config_start < 0) return;
  if (subtopic.substring(config_start) != "/config") return;
  if (subtopic.substring(node_start, object_start - 1) != config_.device_id) return;

  const String component = subtopic.substring(0, node_start - 1);
  const String object = subtopic.substring(object_start, config_start);
  for (const String& entity : published_)
    if (entity == component + "/" + object) return;

  // topic/payload point into PubSubClient's buffer, which publishing reuses;
  // subtopic is a copy, so it's safe to build the topic from it.
  const String topic = String(cfg::kDiscoveryPrefix) + "/" + subtopic;
  mqtt_.publish(topic.c_str(), "", true);
  Serial.printf("[mqtt] removed retired HA entity %s/%s\n", component.c_str(),
                object.c_str());
}

// Not retained: HA converts the uptime to a timestamp using the time it
// receives it, so a replayed value would give the wrong restart time.
void MqttManager::publishUptimeIfDue() {
  if (!uptime_countdown_.due(millis())) return;
  uptime_countdown_.disarm();
  publish(Topic::kUptime,
          String(static_cast<unsigned long>(platform::uptimeSeconds())).c_str(),
          /*retained=*/false);
}

// --- Publishers ---
// PubSubClient::publish() is itself a no-op returning false while
// disconnected, so none of these need to check.
void MqttManager::publishHeight(float cm) {
  publish(Topic::kHeight, String(cm, 1).c_str());
}
void MqttManager::publishTarget(float cm) {
  publish(Topic::kTarget, String(cm, 1).c_str());
}
void MqttManager::publishFirmwareLatest(const String& version) {
  publish(Topic::kFirmwareLatest, version.c_str());
}
void MqttManager::publishFirmwareState(int percent) {
  JsonDocument doc;
  doc["installed_version"] = cfg::kSwVersion;
  doc["in_progress"] = percent >= 0;
  if (percent >= 0) {
    doc["update_percentage"] = percent;
  } else {
    doc["update_percentage"] = nullptr;  // resets HA's progress state
  }
  String payload;
  serializeJson(doc, payload);
  publish(Topic::kFirmwareInstalled, payload.c_str());
}
void MqttManager::publishWifi(int rssi) {
  publish(Topic::kWifi, String(rssi).c_str());
}
void MqttManager::publishLog(const String& message) {
  publish(Topic::kLog, message.c_str(), /*retained=*/false);
}
void MqttManager::publishChildLock(bool locked) {
  publish(Topic::kChildLock, locked ? "ON" : "OFF");
}
void MqttManager::publishAlarm(bool on) {
  publish(Topic::kAlarm, on ? "ON" : "OFF");
}
void MqttManager::publishMovementAvailable(bool available) {
  publish(Topic::kMovementAvailable, available ? "online" : "offline");
}
bool MqttManager::publishCalibration(const MotionModel::State& s) {
  JsonDocument doc;
  for (const auto& c : kCalibrationSensors) {
    // Three decimals is ample for these, and keeps the payload short.
    doc[c.key] = std::round(s.*c.value * 1000.0f) / 1000.0f;
  }
  String payload;
  serializeJson(doc, payload);
  return publish(Topic::kCalibration, payload.c_str());
}
