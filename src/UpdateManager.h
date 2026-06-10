#pragma once

#include <Arduino.h>

#include <functional>

// UpdateManager implements pull-based OTA: the device downloads the latest
// release binary published to the GitHub repository and reflashes itself.
//
// Two assets are expected on the "latest" release:
//   - version.txt   : a single line with the release version (e.g. "1.2.0")
//   - firmware.bin  : the compiled image
//
// A check first compares the remote version.txt against the running version and
// only downloads firmware.bin when they differ (unless forced). On a successful
// flash the device reboots automatically and this call never returns.
class UpdateManager {
 public:
  using LogFn = std::function<void(const String& message)>;

  void begin(const char* current_version, LogFn log = nullptr);

  // Periodic auto-check; safe to call every loop().
  void loop();

  // Run a check now. With force=true the binary is fetched regardless of the
  // remote version. Returns true only on "no update needed" or a handled error
  // (a successful update reboots and does not return).
  bool checkNow(bool force);

 private:
  String fetchRemoteVersion();
  void report(const String& message);

  String current_version_;
  LogFn log_;
  uint32_t last_check_ = 0;
  bool first_check_done_ = false;
};
