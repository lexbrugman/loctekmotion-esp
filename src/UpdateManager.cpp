#include "UpdateManager.h"

#include "Platform.h"
#include "config.h"

namespace {

String firmwareUrl() {
  return String(cfg::kOtaBaseUrl) + "/" + cfg::kFirmwareAsset;
}

String versionUrl() {
  return String(cfg::kOtaBaseUrl) + "/" + cfg::kVersionAsset;
}

}  // namespace

void UpdateManager::begin(const char* current_version, LogFn log,
                          VersionFn on_remote_version) {
  current_version_ = current_version;
  log_ = std::move(log);
  version_cb_ = std::move(on_remote_version);
}

void UpdateManager::report(const String& message) {
  if (log_) log_(message);
}

void UpdateManager::loop() {
  const uint32_t now = millis();
  // First automatic check shortly after boot, then on a fixed interval.
  if (!first_check_done_) {
    if (now < cfg::kOtaStartupDelay) return;  // let Wi-Fi/MQTT settle first
    first_check_done_ = true;
    last_check_ = now;
    checkVersion();
    return;
  }
  if (now - last_check_ >= cfg::kUpdateCheckInterval) {
    last_check_ = now;
    checkVersion();
  }
}

String UpdateManager::fetchRemoteVersion() {
  auto client = platform::makeSecureClient();
  HTTPClient https;
  https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  https.setTimeout(8000);

  String version;
  if (https.begin(*client, versionUrl())) {
    const int code = https.GET();
    if (code == HTTP_CODE_OK) {
      version = https.getString();
      version.trim();
    } else {
      report(String("version check HTTP ") + code);
    }
    https.end();
  } else {
    report("version check: connection failed");
  }
  return version;
}

String UpdateManager::pollRemoteVersion() {
  const String remote = fetchRemoteVersion();
  if (remote.length() == 0) return String();  // fetch failed; caller bails
  if (version_cb_) version_cb_(remote);        // keep the Firmware entity in sync
  return remote;
}

bool UpdateManager::checkVersion() {
  if (WiFi.status() != WL_CONNECTED) {
    report("version check skipped: no Wi-Fi");
    return false;
  }

  const String remote = pollRemoteVersion();
  if (remote.length() == 0) return false;  // try again later
  if (remote == current_version_) {
    report(String("up to date (") + current_version_ + ")");
  } else {
    report(String("update available: ") + current_version_ + " -> " + remote);
  }
  return true;
}

bool UpdateManager::install() {
  if (WiFi.status() != WL_CONNECTED) {
    report("install skipped: no Wi-Fi");
    return false;
  }

  // Guard the flash on a version comparison: fetching firmware.bin always
  // reflashes whatever the server holds (the bare update() overload does no
  // negotiation), so without this an install onto the current version would
  // pointlessly rewrite flash and reboot.
  const String remote = pollRemoteVersion();
  if (remote.length() == 0) return false;  // don't flash blind
  if (remote == current_version_) {
    report(String("already up to date (") + current_version_ + ")");
    return true;  // nothing to install; no reflash, no reboot
  }

  report(String("installing ") + current_version_ + " -> " + remote);

  auto client = platform::makeSecureClient();
  auto& updater = platform::updater();
  updater.rebootOnUpdate(true);
  updater.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  // Only two outcomes are reachable here: success reboots before returning, so
  // anything that does return is a failure. HTTP_UPDATE_NO_UPDATES can't occur
  // — the bare update() overload sends no version header for the server to
  // match against, and we already gated on a version mismatch above.
  if (updater.update(*client, firmwareUrl()) == HTTP_UPDATE_OK) {
    report("update applied; rebooting");  // typically never reached
    return true;
  }
  report(String("update failed: ") + updater.getLastErrorString());
  return false;
}
