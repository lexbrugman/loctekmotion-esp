#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <functional>

#include "ConfigStore.h"
#include "DeferredActions.h"
#include "MotionModel.h"
#include "Platform.h"

// MqttManager owns the broker connection and the Home Assistant integration:
// Last-Will availability, retained MQTT discovery for every entity, telemetry
// publishing, and routing of inbound command topics to a single handler.
//
// Retained discovery configs under this device's node ID that the running
// firmware did not publish are for entities it doesn't define; they are
// deleted on connect so Home Assistant removes those entities.
//
// begin() once, loop() every iteration. Reconnects (and a full discovery
// re-announce) happen automatically after any drop. Everything is
// non-blocking except the connection attempt itself, which can stall for
// several seconds when the broker is unreachable — loop() takes a flag to
// hold attempts off while that would do harm.
class MqttManager {
 public:
  // object = the command id (e.g. "target", "stop", "preset1"); payload = the
  // raw message payload.
  using CommandHandler = std::function<void(const String& object, const String& payload)>;

  // Fired on every successful (re)connection, once discovery has been
  // announced — the place to publish state the broker may no longer hold.
  using ConnectHandler = std::function<void()>;

  // |config| is read on every reconnect, and the MQTT client keeps a pointer
  // to its host, so it must outlive the manager.
  void begin(const DeviceConfig& config, CommandHandler handler);
  void onConnect(ConnectHandler handler) { connect_handler_ = std::move(handler); }
  // allow_reconnect = false keeps servicing a live connection but skips the
  // (blocking) reconnect attempt this iteration.
  void loop(bool allow_reconnect = true);
  bool connected() { return mqtt_.connected(); }

  // --- Telemetry / state publishers (no-ops while disconnected, courtesy of
  // PubSubClient) ---
  void publishHeight(float cm);
  void publishTarget(float cm);
  // Version published on the OTA server, independent of whether it's
  // installed here — feeds the HA update entity's latest_version.
  void publishFirmwareLatest(const String& version);
  // The Firmware entity's state while no install runs: the installed version
  // and the entity's link. Also resets a progress state an interrupted
  // install left behind.
  void publishFirmwareState(const String& url);
  // Install progress (0-100) on the Firmware entity.
  void publishFirmwareProgress(int percent);
  void publishWifi(int rssi);
  void publishLog(const String& message);
  void publishChildLock(bool locked);
  void publishAlarm(bool on);
  void publishMovementAvailable(bool available);
  // Learned motion calibration as one retained JSON blob; the six diagnostic
  // sensors announce() declares (disabled by default in HA) each pick one key
  // out of it via value_template.
  // Returns whether the publish went out, so the caller can retry.
  bool publishCalibration(const MotionModel::State& s);

 private:
  bool reconnect();
  void announce();  // availability + discovery + subscriptions
  // Publishes one retained discovery config. |doc| arrives holding the
  // entity-specific keys; the keys every entity shares (name, unique_id,
  // availability, the device block) are added to it here. Building the
  // payload as a JSON document escapes every value — notably the device
  // name, which is free-form portal input.
  // extra_availability_topic, if non-empty, additionally gates the entity on
  // that topic (alongside the device's overall availability) — used for
  // entities that issue movement commands the desk ignores while child-locked.
  void publishDiscoveryEntity(const char* component, const char* object,
                              const char* name, JsonDocument& doc,
                              const String& extra_availability_topic = "");
  void onMessage(char* topic, const uint8_t* payload, unsigned int length);
  // subtopic is the topic below cfg::kDiscoveryPrefix.
  void onDiscoveryMessage(const String& subtopic, const uint8_t* payload,
                          unsigned int length);
  // Whether announce() publishes a discovery config for <component>/<object>.
  static bool definesEntity(const String& component, const String& object);
  // Publishes the uptime once uptime_countdown_ is due; see announce().
  void publishUptimeIfDue();

  WiFiClient net_;
  PubSubClient mqtt_{net_};
  CommandHandler handler_;
  ConnectHandler connect_handler_;
  const DeviceConfig* config_ = nullptr;  // see begin()
  Countdown retry_;  // throttles reconnect attempts; unarmed until the first
  Countdown uptime_countdown_;

  // Every topic below lives under cfg::kTopicNamespace + "/" + device id.
  String avail_top_;   // <base>/status
  String cmd_prefix_;  // <base>/cmd/

  // State topics, each <base>/<suffix> (suffixes in MqttManager.cpp's
  // kTopicSuffixes, same order). Pre-built once in begin() so the publish hot
  // path (which can run every loop iteration) doesn't concatenate Strings.
  enum class Topic : uint8_t {
    kHeight,
    kTarget,
    kFirmwareLatest,
    kFirmwareInstalled,
    kWifi,
    kUptime,  // seconds since boot, see publishUptimeIfDue()
    kLog,
    kChildLock,
    kAlarm,
    kMovementAvailable,
    kCalibration,
    kCount,
  };
  static constexpr Topic kNoState = Topic::kCount;  // Entity::state when it has none
  const String& topic(Topic t) const { return topics_[static_cast<size_t>(t)]; }
  // Every state publish goes through here.
  bool publish(Topic t, const char* payload, bool retained = true) {
    return mqtt_.publish(topic(t).c_str(), payload, retained);
  }
  String topics_[static_cast<size_t>(Topic::kCount)];

  // The entity table (see MqttManager.cpp): one row per entity announce()
  // publishes, other than the calibration sensors.
  struct Entity;
  static const Entity kEntities[];
};
