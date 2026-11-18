#include <Arduino.h>

#include <cmath>
#include <cstdlib>

#include "ConfigStore.h"
#include "DeferredActions.h"
#include "DeskController.h"
#include "HaState.h"
#include "MqttManager.h"
#include "Platform.h"
#include "Provisioning.h"
#include "UpdateManager.h"
#include "config.h"

namespace {

DeviceConfig config;

DeskController desk;
MqttManager mqtt;
UpdateManager updater;
HaState ha(desk, mqtt);

// Deferred (heavy/terminal) actions, run from loop() rather than the MQTT
// callback to avoid reentrancy and watchdog resets.
enum Action : uint8_t {
  kInstall,
  kCheckUpdate,
  kRestart,
  kWifiReset,
  kCalibrationReset,
};
DeferredActions actions;

// Restart and Wi-Fi-reset are themselves deferred a little further once
// triggered: ESP.restart() tears the link down immediately, so we give the
// "restarting" log message kRestartFlushDelay to actually reach the broker
// first — without blocking the loop while we wait.
Countdown restart_countdown;
Countdown wifi_reset_countdown;

// Strict numeric parse for movement payloads. String::toFloat() returns 0.0
// for garbage, which here would mean "drive to minimum height" — reject
// anything that doesn't parse instead.
bool parseFloat(const String& payload, float& out) {
  char* end = nullptr;
  const float value = strtof(payload.c_str(), &end);
  if (end == payload.c_str() || !std::isfinite(value)) return false;
  out = value;
  return true;
}

// Inbound command topics (<base>/cmd/<name>), one handler each; the names
// match the command topics MqttManager announces in discovery.
struct Command {
  const char* name;
  void (*run)(const String& payload);
};

const Command kCommands[] = {
    {"stop", [](const String&) { desk.stop(); }},
    {"target",
     [](const String& payload) {
       float value;
       if (!parseFloat(payload, value)) return;
       desk.moveToHeight(value);
       ha.onTargetCommand();
     }},
    {"preset1", [](const String&) { desk.press(desk_cmd::Preset1); }},
    {"preset2", [](const String&) { desk.press(desk_cmd::Preset2); }},
    {"sit", [](const String&) { desk.press(desk_cmd::Sit); }},
    {"stand", [](const String&) { desk.press(desk_cmd::Stand); }},
    {"memory", [](const String&) { desk.press(desk_cmd::Memory); }},
    {"alarm",
     [](const String& payload) {
       const bool on = payload == "ON";
       if (!ha.requestAlarm(on)) return;
       // Arming is a tap; disarming only registers as a 3 s held press (a
       // tap would cycle the reminder interval instead).
       if (on) desk.press(desk_cmd::Alarm);
       else desk.alarmOff();
     }},
    {"childlock",
     [](const String& payload) {
       if (ha.requestChildLock(payload == "ON")) desk.childLock();
     }},
    {"wake", [](const String&) { desk.press(desk_cmd::Wake); }},
    {"calibration_reset", [](const String&) { actions.request(kCalibrationReset); }},
    {"update", [](const String&) { actions.request(kInstall); }},
    {"check_update", [](const String&) { actions.request(kCheckUpdate); }},
    {"configure", [](const String&) { actions.request(kWifiReset); }},
    {"restart", [](const String&) { actions.request(kRestart); }},
};

void handleCommand(const String& object, const String& payload) {
  for (const Command& c : kCommands) {
    if (object == c.name) {
      c.run(payload);
      return;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nLoctekMotion desk controller starting"));

  provisioning::checkDoubleReset();
  provisioning::run(config);
  WiFi.setAutoReconnect(true);

  desk.begin();
  desk.onHeight([](float cm) { ha.onHeight(cm); });
  desk.onSeekEnded([](const DeskController::SeekReport* r) { ha.onSeekEnded(r); });

  mqtt.begin(config, handleCommand);
  mqtt.onConnect([] { ha.onConnect(); });
  updater.begin(cfg::kSwVersion, [](const String& m) {
    Serial.print(F("[ota] "));
    Serial.println(m);
    mqtt.publishLog(m);
  }, [](const String& v) { ha.onLatestFirmware(v); });
  updater.onProgress([](int percent) { ha.onUpdateProgress(percent); });

  // Fetch the current height once at boot.
  desk.press(desk_cmd::Wake);
}

void loop() {
  desk.loop();

  // Connecting to the broker and talking to the OTA server both block for up
  // to several seconds. The desk only keeps moving while drive frames keep
  // arriving at their cadence, so stalling the loop mid-move would cut the
  // drive short and abort the seek — hold that work off until the planner
  // has nothing in flight.
  const bool desk_busy = desk.busy();
  mqtt.loop(/*allow_reconnect=*/!desk_busy);
  if (!desk_busy) updater.loop();

  ha.loop();

  const uint32_t now = millis();

  if (actions.take(kCalibrationReset)) {
    desk.resetCalibration();
    ha.onCalibrationChanged();
    mqtt.publishLog("calibration reset; starting fresh from the seeds");
  }
  // Blocking, like the periodic check above: a request made mid-move stays
  // pending and runs once the desk is done. (Re-read busy(): a command that
  // arrived in mqtt.loop() may just have started a move.)
  if (!desk.busy()) {
    if (actions.take(kCheckUpdate)) {
      updater.checkVersion();  // refreshes the Firmware entity; never installs
    }
    if (actions.take(kInstall)) {
      updater.install();  // reboots on success
    }
  }
  if (actions.take(kWifiReset)) {
    mqtt.publishLog("clearing Wi-Fi config; rebooting into setup portal");
    wifi_reset_countdown.arm(now, cfg::kRestartFlushDelay);
  }
  if (actions.take(kRestart)) {
    mqtt.publishLog("restarting");
    restart_countdown.arm(now, cfg::kRestartFlushDelay);
  }
  if (wifi_reset_countdown.due(now)) {
    provisioning::clearWifiSettings();
    ESP.restart();
  }
  if (restart_countdown.due(now)) {
    ESP.restart();
  }
}
