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

bool UpdateManager::checkVersion() {
  if (WiFi.status() != WL_CONNECTED) {
    report("version check skipped: no Wi-Fi");
    return false;
  }

  const String remote = fetchRemoteVersion();
  if (remote.length() == 0) {
    return false;  // couldn't determine remote version; try again later
  }
  if (version_cb_) version_cb_(remote);
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

  report("installing latest firmware");

  auto client = platform::makeSecureClient();
  auto& updater = platform::updater();
  updater.rebootOnUpdate(true);
  updater.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  const t_httpUpdate_return ret = updater.update(*client, firmwareUrl());
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      report(String("update failed: ") + updater.getLastErrorString());
      return false;
    case HTTP_UPDATE_NO_UPDATES:
      report("server reports no update");
      return true;
    case HTTP_UPDATE_OK:
      report("update applied; rebooting");  // typically never reached
      return true;
  }
  return false;
}
