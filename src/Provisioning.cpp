#include "Provisioning.h"

#include <Arduino.h>
#include <Ticker.h>
#include <WiFiManager.h>

#include <cstdlib>

#include "DeviceId.h"
#include "Platform.h"
#include "config.h"

namespace {

Ticker double_reset_window;

void closeDoubleResetWindow() { platform::writeResetFlag(false); }

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

}  // namespace

namespace provisioning {

using config_store::setField;

void checkDoubleReset() {
  // The flag alone isn't proof of a second button press: it also survives the
  // device restarting *itself* — after a portal timeout, a crash or a
  // watchdog reset — and none of those may cost the user their Wi-Fi
  // credentials. Only a boot the reset button could have caused counts.
  const bool flagged = platform::readResetFlag();
  platform::writeResetFlag(false);
  if (flagged && platform::resetByButton()) {
    Serial.println(F("Double reset detected; clearing Wi-Fi config"));
    clearWifiSettings();
    return;
  }
  platform::writeResetFlag(true);
  // A timer rather than a check in loop(): the window has to close on time
  // even while run() sits in the captive portal for minutes (e.g. the router
  // is down), or a single RST press there would count as the second one.
  double_reset_window.once_ms(cfg::kDoubleResetWindow, closeDoubleResetWindow);
}

void run(DeviceConfig& config) {
  config_store::begin();
  config_store::load(config);

  // Fall back to the compile-time factory defaults until the portal's Device
  // ID / Device name fields are filled in (keeps existing single-device setups
  // working unchanged, and gives the portal sensible starting values).
  if (config.device_id[0] == '\0') setField(config.device_id, cfg::kDeviceId);
  if (config.device_name[0] == '\0') setField(config.device_name, cfg::kDeviceName);

  char port_str[8];
  snprintf(port_str, sizeof(port_str), "%u", config.mqtt_port);

  WiFiManager wm;
  // Reflects whatever device_id is in effect when this boot started; a change
  // saved during this run's portal session takes effect on the next reboot.
  wm.setHostname(deviceHostname(config.device_id).c_str());
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
    // Belt and braces: this restart must never look like half of a double
    // reset (see checkDoubleReset), whatever state the window is in.
    platform::writeResetFlag(false);
    ESP.restart();
  }

  if (saved) {
    setField(config.device_id, p_id.getValue());
    sanitizeDeviceId(config.device_id);
    if (config.device_id[0] == '\0') setField(config.device_id, cfg::kDeviceId);
    setField(config.device_name, p_name.getValue(), cfg::kDeviceName);

    setField(config.mqtt_host, p_host.getValue());
    const long port = atol(p_port.getValue());
    if (port < 1 || port > 65535) {
      Serial.printf("Ignoring out-of-range MQTT port \"%s\"; keeping %u\n",
                    p_port.getValue(), config.mqtt_port);
    } else {
      config.mqtt_port = static_cast<uint16_t>(port);
    }
    setField(config.mqtt_user, p_user.getValue());
    setField(config.mqtt_pass, p_pass.getValue());
    config_store::save(config);
    Serial.println(F("Saved provisioning config"));
  }
}

void clearWifiSettings() {
  WiFiManager wm;
  wm.resetSettings();
}

}  // namespace provisioning
