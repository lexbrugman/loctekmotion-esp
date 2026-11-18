#include "UpdateManager.h"

#include "Platform.h"
#include "config.h"

namespace {

// Report install progress in steps this size (percent): enough for a smooth
// progress bar without slowing the flash or flooding the broker. The publishes
// also keep the MQTT connection alive while the install blocks the loop —
// they count as client activity toward the broker's keepalive.
constexpr int kProgressStep = 5;

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
  first_check_.arm(millis(), cfg::kOtaStartupDelay);
}

void UpdateManager::report(const String& message) {
  if (log_) log_(message);
}

void UpdateManager::loop() {
  const uint32_t now = millis();
  // First automatic check shortly after startup, then on a fixed interval.
  if (first_check_.armed()) {
    if (!first_check_.due(now)) return;  // let Wi-Fi/MQTT settle first
    first_check_.disarm();
    last_check_ = now;
    checkVersion();
    return;
  }
  if (now - last_check_ >= cfg::kUpdateCheckInterval) {
    last_check_ = now;
    checkVersion();
  }
}

String UpdateManager::remoteVersion(const char* action) {
  if (WiFi.status() != WL_CONNECTED) {
    report(String(action) + " skipped: no Wi-Fi");
    return String();
  }

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
  if (version.length() && version_cb_) version_cb_(version);  // keep HA in sync
  return version;
}

bool UpdateManager::checkVersion() {
  const String remote = remoteVersion("version check");
  if (remote.length() == 0) return false;  // try again later
  if (remote == current_version_) {
    report(String("up to date (") + current_version_ + ")");
  } else {
    report(String("update available: ") + current_version_ + " -> " + remote);
  }
  return true;
}

bool UpdateManager::install() {
  // Guard the flash on a version comparison: fetching the image always
  // reflashes whatever the server holds (the bare update() overload does no
  // negotiation), so without this an install onto the current version would
  // pointlessly rewrite flash and reboot.
  const String remote = remoteVersion("install");
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
  platform::keepConnectionsDuringUpdate();
  last_progress_ = 0;
  if (progress_cb_) progress_cb_(0);
  updater.onProgress([this](int done, int total) {
    if (!progress_cb_ || total <= 0) return;
    const int percent = static_cast<int>(static_cast<int64_t>(done) * 100 / total);
    if (percent < last_progress_ + kProgressStep && percent < 100) return;
    if (percent == last_progress_) return;
    last_progress_ = percent;
    progress_cb_(percent);
  });

  // Only two outcomes are reachable here: success reboots before returning, so
  // anything that does return is a failure. HTTP_UPDATE_NO_UPDATES can't occur
  // — the bare update() overload sends no version header for the server to
  // match against, and we already gated on a version mismatch above.
  if (updater.update(*client, firmwareUrl()) == HTTP_UPDATE_OK) {
    report("update applied; rebooting");  // typically never reached
    return true;
  }
  report(String("update failed: ") + updater.getLastErrorString());
  if (progress_cb_) progress_cb_(-1);
  return false;
}
