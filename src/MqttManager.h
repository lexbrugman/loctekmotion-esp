#pragma once

#include <Arduino.h>
#include <PubSubClient.h>

#include <functional>

#include "ConfigStore.h"
#include "MotionModel.h"
#include "Platform.h"

// MqttManager owns the broker connection and the Home Assistant integration:
// Last-Will availability, retained MQTT discovery for every entity, telemetry
// publishing, and routing of inbound command topics to a single handler.
//
// It is fully non-blocking: begin() once, loop() every iteration. Reconnects
// (and a full discovery re-announce) happen automatically after any drop.
class MqttManager {
 public:
  // object = the command id (e.g. "cover", "target", "preset1"); payload = the
  // raw message payload.
  using CommandHandler = std::function<void(const String& object, const String& payload)>;

  void begin(const DeviceConfig& config, CommandHandler handler);
  void loop();
  bool connected() { return mqtt_.connected(); }

  // --- Telemetry / state publishers (no-ops while disconnected) ---
  void publishHeight(float cm);
  void publishPosition(int percent);
  void publishTarget(float cm);
  void publishWifi(int rssi);
  void publishUptime(unsigned long seconds);
  void publishLog(const String& message);
  void publishChildLock(bool locked);
  void publishMovementAvailable(bool available);
  // Learned motion calibration as one retained JSON blob; the six diagnostic
  // sensors announce() declares (disabled by default in HA) each pick one key
  // out of it via value_template.
  void publishCalibration(const MotionModel::State& s);

 private:
  bool reconnect();
  void announce();  // availability + discovery + subscriptions
  // extra_availability_topic, if non-empty, additionally gates the entity on
  // that topic (alongside the device's overall availability) — used for
  // entities that issue movement commands the desk ignores while child-locked.
  void publishDiscoveryEntity(const char* component, const char* object,
                              const char* name, const String& specifics,
                              const String& extra_availability_topic = "");
  void onMessage(char* topic, const uint8_t* payload, unsigned int length);

  WiFiClient net_;
  PubSubClient mqtt_{net_};
  CommandHandler handler_;
  DeviceConfig config_;
  uint32_t last_attempt_ = 0;
  bool first_attempt_ = true;

  String base_;        // cfg::kTopicNamespace + "/" + config_.device_id
  String avail_top_;   // base_ + "/status"
  String cmd_prefix_;  // base_ + "/cmd/"

  // Telemetry topics, pre-built once in begin() so the publish hot path (which
  // can run every loop iteration) doesn't concatenate Strings each time.
  String height_top_;
  String position_top_;
  String target_top_;
  String wifi_top_;
  String uptime_top_;
  String log_top_;
  String ota_channel_top_;
  String childlock_top_;
  String movement_avail_top_;
  String calibration_top_;
};
