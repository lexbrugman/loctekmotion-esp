#pragma once

#include <Arduino.h>

#include <functional>

#include "DeferredActions.h"

// UpdateManager implements pull-based OTA from the rolling GitHub release
// this build tracks (its OTA channel — see cfg::kOtaBaseUrl in config.h).
//
// Two assets are expected on the channel's release:
//   - version.txt           : a single line with the published build's
//                             version (the short commit SHA)
//   - firmware-<board>.bin  : the app image for this board
//                             (cfg::kFirmwareAsset)
//
// checkVersion() only compares the remote version.txt against the running
// version and reports the result (see VersionFn) - it never installs.
// install() does the same comparison and, only when they differ, downloads
// and flashes the board's image; on success the device reboots automatically
// and that call never returns.
//
// Both talk to the server synchronously and block for up to several seconds,
// loop()'s periodic check included — callers hold them off while the desk is
// moving (see main.cpp).
class UpdateManager {
 public:
  using LogFn = std::function<void(const String& message)>;
  // Fired whenever a remote version.txt check resolves, whether or not it
  // differs from current_version_ — feeds the HA update entity's
  // latest_version, which should track "what's published" independent of
  // whether this device has installed it yet.
  using VersionFn = std::function<void(const String& version)>;
  // Install progress: 0-100 while an install runs, -1 when it failed. (A
  // successful install reboots instead; the new firmware reports itself.)
  using ProgressFn = std::function<void(int percent)>;

  void begin(const char* current_version, LogFn log = nullptr,
             VersionFn on_remote_version = nullptr);
  void onProgress(ProgressFn cb) { progress_cb_ = std::move(cb); }

  // Periodic auto-check; call every loop() iteration in which blocking is
  // acceptable — a skipped check simply runs on the next call.
  void loop();

  // Fetch the published version and report it (via VersionFn / log). Never
  // installs. Returns false only if the version couldn't be determined.
  bool checkVersion();

  // Install the latest firmware if it differs from the running version. A
  // no-op (no reflash, no reboot) when already up to date. On a real update
  // the device reboots and this never returns; returns false on a handled
  // failure.
  bool install();

 private:
  // Fetch the channel's published version and push it to version_cb_ (the HA
  // latest_version), returning it for the caller to act on - or "" (and a
  // log line naming |action|) if there's no Wi-Fi or the fetch failed.
  String remoteVersion(const char* action);
  void report(const String& message);

  String current_version_;
  LogFn log_;
  VersionFn version_cb_;
  ProgressFn progress_cb_;
  int last_progress_ = 0;  // last percentage reported, for throttling
  // The first automatic check waits cfg::kOtaStartupDelay after begin(), so
  // it doesn't compete with Wi-Fi/MQTT while they're still settling.
  Countdown first_check_;
  uint32_t last_check_ = 0;
};
