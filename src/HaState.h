#pragma once

#include <Arduino.h>

#include "DeskController.h"
#include "MqttManager.h"
#include "OptimisticSwitch.h"

// The state Home Assistant should be showing, and keeping it there: height
// and slider, child lock / alarm / movement availability, calibration, the
// published firmware version and Wi-Fi signal. Everything is published
// retained and only on change, and sent again on every broker (re)connection
// (onConnect) in case the broker lost it.
//
// Wire the event handlers to the desk and MQTT callbacks, and call loop()
// every iteration.
class HaState {
 public:
  HaState(DeskController& desk, MqttManager& mqtt) : desk_(desk), mqtt_(mqtt) {}

  void loop();

  // --- Events ---
  void onHeight(float cm);
  void onSeekEnded(const DeskController::SeekReport* report);
  void onConnect();
  // A target height was just commanded (after DeskController::moveToHeight).
  void onTargetCommand();
  // Version read from the OTA channel (independent of what's installed).
  void onLatestFirmware(const String& version);
  // Install progress (see UpdateManager::ProgressFn), shown on the Firmware
  // entity.
  void onUpdateProgress(int percent) { mqtt_.publishFirmwareState(percent); }
  // The learned calibration changed outside a seek (it was reset).
  void onCalibrationChanged() { calibration_pending_ = true; }

  // --- Switch commands from HA ---
  // Publish what HA should show now — the requested state, or the real one
  // when the desk is already there — and return whether the desk command
  // still has to be sent.
  bool requestAlarm(bool on);
  bool requestChildLock(bool locked);

 private:
  void publishSeekReport(const DeskController::SeekReport& r);
  void publishSwitches();
  void publishCalibration();

  DeskController& desk_;
  MqttManager& mqtt_;

  // The desk streams its display ~9x/s while moving, mostly repeating the
  // same value — republish (retained) only when it actually changes.
  float last_published_height_ = -1.0f;

  // Switch state trackers (see OptimisticSwitch): retained publish-on-change,
  // bridging the desk's slow read-back after a toggle with an optimistic
  // hold — child lock registers only after its held command, and an alarm
  // disarm is confirmed only once the display detection windows have run
  // their course.
  OptimisticSwitch lock_switch_;
  OptimisticSwitch alarm_switch_;
  // Movement availability uses the change-tracking half only (it's derived
  // state, never commanded): the desk ignores movement commands while
  // child-locked, so the movement-related entities grey out instead of
  // silently doing nothing. Tracks the real lock state, not the optimistic
  // switch above, since it reflects whether commands will actually work.
  OptimisticSwitch movement_available_;

  // Learned motion calibration (retained JSON; see MqttManager's calibration
  // sensors). Values only change when a seek completes or calibration is
  // reset, so publish on those events plus once per broker connection — the
  // flag goes back to pending on each, and loop() retries until a publish
  // succeeds.
  bool calibration_pending_ = true;

  // Version last read from the OTA channel ("" until the first successful
  // check), kept so it can be republished after a broker reconnect.
  String latest_firmware_;

  uint32_t last_telemetry_ = 0;  // Wi-Fi signal, every cfg::kTelemetryInterval
};
