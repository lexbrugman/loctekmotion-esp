#include <Arduino.h>
#include <WiFiManager.h>

#include <cmath>
#include <cstdlib>

#include "ConfigStore.h"
#include "DeskController.h"
#include "MqttManager.h"
#include "OptimisticSwitch.h"
#include "Platform.h"
#include "UpdateManager.h"
#include "config.h"

namespace {

DeviceConfig config;

// Discovery topics, the MQTT client id and the HA device identifier are all
// keyed off device_id, so it must stay safe for use as an MQTT topic segment.
// Lower-case it and replace anything outside [a-z0-9_] with '_' — silently
// fixing fat-fingered portal input rather than producing a broken namespace.
void sanitizeDeviceId(char* id) {
  for (char* c = id; *c; ++c) {
    if (*c >= 'A' && *c <= 'Z') *c += 'a' - 'A';
    else if (!((*c >= 'a' && *c <= 'z') || (*c >= '0' && *c <= '9') || *c == '_')) *c = '_';
  }
}

// SSID for the provisioning AP: kPortalSsidPrefix plus the last 3 bytes of the
// MAC address (lower-case hex, no separators) — short, always-available, and
// unique per device, so multiple desks' setup networks are tellable apart
// (e.g. "loctekmotion-a1b2c3").
String portalSsid() {
  String mac = WiFi.macAddress();  // "AA:BB:CC:DD:EE:FF"
  mac.replace(":", "");
  mac.toLowerCase();
  return String(cfg::kPortalSsidPrefix) + mac.substring(mac.length() - 6);
}

DeskController desk(cfg::kPinUartRx, cfg::kPinUartTx, cfg::kPinScreen, cfg::kUartBaud,
                    cfg::kMinHeight, cfg::kMaxHeight);
MqttManager mqtt;
UpdateManager updater;

uint32_t last_telemetry = 0;

// Deferred (heavy/terminal) actions, run from loop() rather than the MQTT
// callback to avoid reentrancy and watchdog resets.
bool want_forced_update = false;
bool want_restart = false;
bool want_wifi_reset = false;
bool want_calibration_reset = false;

// Restart and Wi-Fi-reset are themselves deferred a little further once
// triggered: ESP.restart() tears the link down immediately, so we give the
// "restarting" log message kRestartFlushDelay to actually reach the broker
// first — without blocking the loop while we wait.
bool restart_pending = false;
uint32_t restart_at = 0;
bool wifi_reset_pending = false;
uint32_t wifi_reset_at = 0;

// Set once at boot if this *isn't* a double reset (see setup()): the moment
// (in millis()) at which the RTC-memory flag should be cleared again, so a
// later, unrelated reset starts its own fresh window rather than being
// mistaken for a follow-up to this boot. 0 means "nothing to clear".
uint32_t reset_flag_clear_at = 0;

void closeDoubleResetWindow(uint32_t now) {
  if (reset_flag_clear_at && now >= reset_flag_clear_at) {
    reset_flag_clear_at = 0;
    platform::writeResetFlag(false);
  }
}

// Learned motion calibration (retained JSON; see MqttManager's calibration
// sensors). Values only change when a seek completes or calibration is reset,
// so publish on those events plus once per broker connection — the flag goes
// back to pending on each, and the loop retries until a publish succeeds.
bool calibration_pending = true;

void publishCalibrationState() {
  if (!calibration_pending || !mqtt.connected()) return;
  calibration_pending = false;
  mqtt.publishCalibration(desk.learnedState());
}

// One-line summary per completed seek: how the coast prediction compared to
// reality and what the corrections had to clean up — the ground truth for
// judging the motion model's convergence from the HA log.
void publishSeekReport(const DeskController::SeekReport& r) {
  String m;
  m.reserve(160);
  m += "seek ";
  m += String(r.start_height, 1);
  m += "->";
  m += String(r.target, 1);
  m += ": settled ";
  m += String(r.settled_height, 1);
  m += " (err ";
  m += String(r.settled_height - r.target, 2);
  m += "), coast pred ";
  m += String(r.predicted_coast, 2);
  m += " obs ";
  m += String(r.observed_coast, 2);
  m += " @ ";
  m += String(r.stop_speed, 2);
  m += r.stop_speed_measured ? " cm/s measured, taps " : " cm/s assumed, taps ";
  m += String(r.correction_taps);
  m += ", ";
  m += String(static_cast<float>(r.duration_ms) / 1000.0f, 1);
  m += "s";
  mqtt.publishLog(m);
  calibration_pending = true;  // the seek's learning updates just landed
}

// The desk streams its display ~9x/s while moving, mostly repeating the same
// value — republish (retained) only when it actually changes.
float last_published_height = -1.0f;

void publishHeightState(float cm) {
  if (!mqtt.connected() || cm == last_published_height) return;
  last_published_height = cm;
  mqtt.publishHeight(cm);
  const float frac = desk.position();
  if (frac >= 0.0f) mqtt.publishPosition(static_cast<int>(lroundf(frac * 100.0f)));
}

// Switch state trackers (see OptimisticSwitch): retained publish-on-change,
// bridging the desk's slow read-back after a toggle with an optimistic hold —
// child lock registers only after its held command, and an alarm disarm is
// confirmed only once the display detection windows have run their course.
OptimisticSwitch lock_switch;
OptimisticSwitch alarm_switch;
// Movement availability uses the change-tracking half only (it's derived
// state, never commanded): the desk ignores movement commands while
// child-locked, so the movement-related entities grey out instead of
// silently doing nothing. Tracks the real lock state, not the optimistic
// switch above, since it reflects whether commands will actually work.
OptimisticSwitch movement_available;

void publishChildLockState() {
  if (!mqtt.connected()) return;
  bool locked;
  if (lock_switch.sync(desk.childLocked(), millis(), locked))
    mqtt.publishChildLock(locked);
}

void publishAlarmState() {
  if (!mqtt.connected()) return;
  bool on;
  if (alarm_switch.sync(desk.alarmOn(), millis(), on)) mqtt.publishAlarm(on);
}

void publishMovementAvailability() {
  if (!mqtt.connected()) return;
  bool available;
  if (movement_available.sync(!desk.childLocked(), millis(), available))
    mqtt.publishMovementAvailable(available);
}

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

void handleCommand(const String& object, const String& payload) {
  float value = 0.0f;
  if (object == "cover") {
    if (payload == "OPEN") desk.moveUp();
    else if (payload == "CLOSE") desk.moveDown();
    else if (payload == "STOP") desk.stop();
  } else if (object == "position") {
    if (parseFloat(payload, value)) desk.moveToPosition(value / 100.0f);
  } else if (object == "target") {
    if (parseFloat(payload, value)) {
      desk.moveToHeight(value);
      mqtt.publishTarget(value);
    }
  } else if (object == "preset1") {
    desk.preset1();
  } else if (object == "preset2") {
    desk.preset2();
  } else if (object == "sit") {
    desk.sit();
  } else if (object == "stand") {
    desk.stand();
  } else if (object == "memory") {
    desk.memory();
  } else if (object == "alarm") {
    bool publish;
    if (alarm_switch.request(payload == "ON", desk.alarmOn(), millis(),
                             cfg::kAlarmToggleSettle, publish)) {
      // Arming is a tap; disarming only registers as a 3 s held press (a
      // tap would cycle the reminder interval instead).
      if (publish) desk.alarm();
      else desk.alarmOff();
    }
    mqtt.publishAlarm(publish);
  } else if (object == "childlock") {
    bool publish;
    if (lock_switch.request(payload == "ON", desk.childLocked(), millis(),
                            cfg::kChildLockToggleSettle, publish)) {
      desk.childLock();
    }
    mqtt.publishChildLock(publish);
  } else if (object == "wake") {
    desk.wakeScreen();
  } else if (object == "calibration_reset") {
    want_calibration_reset = true;
  } else if (object == "update") {
    want_forced_update = true;
  } else if (object == "configure") {
    want_wifi_reset = true;
  } else if (object == "restart") {
    want_restart = true;
  }
}

// Blocking, runs once at boot: connect to Wi-Fi (raising a captive portal to
// collect Wi-Fi + MQTT settings on first boot) and persist any changes.
void provision() {
  config_store::begin();
  config_store::load(config);

  // Fall back to the compile-time factory defaults until the portal's Device
  // ID / Device name fields are filled in (keeps existing single-device setups
  // working unchanged, and gives the portal sensible starting values).
  if (config.device_id[0] == '\0')
    strncpy(config.device_id, cfg::kDeviceId, sizeof(config.device_id) - 1);
  if (config.device_name[0] == '\0')
    strncpy(config.device_name, cfg::kDeviceName, sizeof(config.device_name) - 1);

  char port_str[8];
  snprintf(port_str, sizeof(port_str), "%u", config.mqtt_port);

  WiFiManager wm;
  // Reflects whatever device_id is in effect when this boot started; a change
  // saved during this run's portal session takes effect on the next reboot.
  wm.setHostname(config.device_id);
  wm.setConfigPortalTimeout(cfg::kPortalTimeout);

  WiFiManagerParameter p_id(
      "device_id", "Device ID (a-z0-9_, must be unique if running multiple)",
      config.device_id, sizeof(config.device_id) - 1);
  WiFiManagerParameter p_name("device_name", "Device name", config.device_name,
                              sizeof(config.device_name) - 1);
  WiFiManagerParameter p_host("mqtt_host", "MQTT host", config.mqtt_host,
                              sizeof(config.mqtt_host) - 1);
  WiFiManagerParameter p_port("mqtt_port", "MQTT port", port_str, sizeof(port_str) - 1);
  WiFiManagerParameter p_user("mqtt_user", "MQTT user", config.mqtt_user,
                              sizeof(config.mqtt_user) - 1);
  WiFiManagerParameter p_pass("mqtt_pass", "MQTT password", config.mqtt_pass,
                              sizeof(config.mqtt_pass) - 1);
  wm.addParameter(&p_id);
  wm.addParameter(&p_name);
  wm.addParameter(&p_host);
  wm.addParameter(&p_port);
  wm.addParameter(&p_user);
  wm.addParameter(&p_pass);

  bool saved = false;
  wm.setSaveConfigCallback([&saved]() { saved = true; });

  const String ssid = portalSsid();
  if (!wm.autoConnect(ssid.c_str(), cfg::kPortalPassword)) {
    Serial.println(F("Wi-Fi provisioning timed out; rebooting"));
    ESP.restart();
  }

  if (saved) {
    strncpy(config.device_id, p_id.getValue(), sizeof(config.device_id) - 1);
    sanitizeDeviceId(config.device_id);
    if (config.device_id[0] == '\0')
      strncpy(config.device_id, cfg::kDeviceId, sizeof(config.device_id) - 1);
    strncpy(config.device_name, p_name.getValue(), sizeof(config.device_name) - 1);
    if (config.device_name[0] == '\0')
      strncpy(config.device_name, cfg::kDeviceName, sizeof(config.device_name) - 1);

    strncpy(config.mqtt_host, p_host.getValue(), sizeof(config.mqtt_host) - 1);
    const long port = atol(p_port.getValue());
    if (port < 1 || port > 65535) {
      Serial.printf("Ignoring out-of-range MQTT port \"%s\"; keeping %u\n",
                    p_port.getValue(), config.mqtt_port);
    } else {
      config.mqtt_port = static_cast<uint16_t>(port);
    }
    strncpy(config.mqtt_user, p_user.getValue(), sizeof(config.mqtt_user) - 1);
    strncpy(config.mqtt_pass, p_pass.getValue(), sizeof(config.mqtt_pass) - 1);
    config_store::save(config);
    Serial.println(F("Saved provisioning config"));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nLoctekMotion desk controller starting"));

  // Double-reset detection (see config.h's kDoubleResetWindow): if the flag
  // from a previous boot is still set, the user has just pressed RST for the
  // second time within the window — clear Wi-Fi creds *now*, before
  // provision() tries to use them, so its autoConnect() fails immediately
  // and raises the captive portal this boot rather than after another
  // reboot. Otherwise, set the flag and arm its clearing for later in
  // loop() once the window passes uneventfully.
  if (platform::readResetFlag()) {
    platform::writeResetFlag(false);
    Serial.println(F("Double reset detected; clearing Wi-Fi config"));
    WiFiManager().resetSettings();
  } else {
    platform::writeResetFlag(true);
    reset_flag_clear_at = millis() + cfg::kDoubleResetWindow;
  }

  provision();
  WiFi.setAutoReconnect(true);

  desk.begin();
  desk.onHeight(publishHeightState);
  desk.onSeekDone(publishSeekReport);

  mqtt.begin(config, handleCommand);
  updater.begin(cfg::kSwVersion, [](const String& m) {
    Serial.print(F("[ota] "));
    Serial.println(m);
    mqtt.publishLog(m);
  });

  // Fetch the current height once at boot.
  desk.wakeScreen();
}

void loop() {
  desk.loop();
  mqtt.loop();
  updater.loop();

  publishChildLockState();
  publishAlarmState();
  publishMovementAvailability();
  publishCalibrationState();

  const uint32_t now = millis();
  closeDoubleResetWindow(now);

  if (now - last_telemetry >= cfg::kTelemetryInterval) {
    last_telemetry = now;
    mqtt.publishWifi(WiFi.RSSI());
    mqtt.publishUptime(now / 1000);
  }

  if (want_calibration_reset) {
    want_calibration_reset = false;
    desk.resetCalibration();
    calibration_pending = true;
    mqtt.publishLog("calibration reset; starting fresh from seed margins");
  }
  if (want_forced_update) {
    want_forced_update = false;
    updater.checkNow(true);  // reboots on success
  }
  if (want_wifi_reset) {
    want_wifi_reset = false;
    mqtt.publishLog("clearing Wi-Fi config; rebooting into setup portal");
    wifi_reset_pending = true;
    wifi_reset_at = now + cfg::kRestartFlushDelay;
  }
  if (want_restart) {
    want_restart = false;
    mqtt.publishLog("restarting");
    restart_pending = true;
    restart_at = now + cfg::kRestartFlushDelay;
  }
  // Give the log messages above a moment to actually reach the broker —
  // ESP.restart() tears the link down immediately — without blocking the
  // loop while we wait.
  if (wifi_reset_pending && static_cast<int32_t>(now - wifi_reset_at) >= 0) {
    WiFiManager wm;
    wm.resetSettings();
    ESP.restart();
  }
  if (restart_pending && static_cast<int32_t>(now - restart_at) >= 0) {
    ESP.restart();
  }
}
