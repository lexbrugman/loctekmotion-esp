#pragma once

#include <Arduino.h>

#include <functional>

// UpdateManager implements pull-based OTA: on request, the device downloads
// the latest release binary published to the GitHub repository and reflashes
// itself.
//
// Two assets are expected on the "latest" release:
//   - version.txt   : a single line with the release version (e.g. "1.2.0")
//   - firmware.bin  : the compiled image
//
// checkVersion() only compares the remote version.txt against the running
// version and reports the result (see VersionFn) - it never installs.
// install() does the same comparison and, only when they differ, downloads
// and flashes firmware.bin; on success the device reboots automatically and
// that call never returns.
class UpdateManager {
 public:
  using LogFn = std::function<void(const String& message)>;
  // Fired whenever a remote version.txt check resolves, whether or not it
  // differs from current_version_ — feeds the HA update entity's
  // latest_version, which should track "what's published" independent of
  // whether this device has installed it yet.
  using VersionFn = std::function<void(const String& version)>;

  void begin(const char* current_version, LogFn log = nullptr,
             VersionFn on_remote_version = nullptr);

  // Periodic auto-check; safe to call every loop().
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
  String fetchRemoteVersion();
  // Fetch the channel's published version and push it to version_cb_ (the HA
  // latest_version), returning it for the caller to act on - or "" if it
  // couldn't be fetched. Shared by checkVersion() and install().
  String pollRemoteVersion();
  void report(const String& message);

  String current_version_;
  LogFn log_;
  VersionFn version_cb_;
  uint32_t last_check_ = 0;
  bool first_check_done_ = false;
};
