#include "HaState.h"

#include "Platform.h"
#include "config.h"

namespace {

// A version that names a commit (a short SHA), as opposed to e.g. "dev" for
// a local build.
bool isCommit(const String& version) {
  if (version.length() < 7) return false;
  for (unsigned i = 0; i < version.length(); i++) {
    const char c = version[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

}  // namespace

void HaState::loop() {
  publishSwitches();
  publishCalibration();

  const uint32_t now = millis();
  if (now - last_telemetry_ >= cfg::kTelemetryInterval) {
    last_telemetry_ = now;
    mqtt_.publishWifi(WiFi.RSSI());
  }
}

void HaState::onHeight(float cm) {
  if (!mqtt_.connected() || cm == last_published_height_) return;
  last_published_height_ = cm;
  mqtt_.publishHeight(cm);
  // The slider follows the live height, except while a slider target is
  // pursued: then it holds the setpoint onTargetCommand echoed.
  if (!desk_.pursuingTarget()) mqtt_.publishTarget(cm);
}

// A slider seek ended, however it ended. Hand the slider off from the held
// setpoint back to the live height: a completed seek settles on its final
// height, but an aborted one (STOP, a timeout, a travel limit, no height
// reports) may stop anywhere, and without this the thumb would keep showing
// a target the desk never reached until its height next changed. Any coast
// after an abort is followed by onHeight as usual.
void HaState::onSeekEnded(const DeskController::SeekReport* report) {
  if (report) {
    publishSeekReport(*report);
    calibration_pending_ = true;  // the seek's learning updates just landed
  }
  if (desk_.hasHeight()) mqtt_.publishTarget(desk_.height());
}

// After every broker (re)connection: send the current state again, since a
// broker that restarted without persistence holds none of it — without a
// movement-availability message, HA would keep the movement entities
// unavailable until the child lock next toggled. What was never published
// is left to loop()'s publish-on-change.
void HaState::onConnect() {
  calibration_pending_ = true;
  last_published_height_ = -1.0f;
  if (desk_.hasHeight()) onHeight(desk_.height());

  bool state;
  if (lock_switch_.republish(state)) mqtt_.publishChildLock(state);
  if (alarm_switch_.republish(state)) mqtt_.publishAlarm(state);
  if (movement_available_.republish(state)) mqtt_.publishMovementAvailable(state);

  if (latest_firmware_.length()) mqtt_.publishFirmwareLatest(latest_firmware_);
  publishFirmware();
  mqtt_.publishWifi(WiFi.RSSI());
  last_telemetry_ = millis();
}

// Echo what the desk will actually do, not the raw request: the target as
// clamped to the travel range while it's pursued, or — when the request was
// already within the deadband, so nothing moves and no seek ends to hand the
// slider back — the current height.
void HaState::onTargetCommand() {
  if (desk_.pursuingTarget()) {
    mqtt_.publishTarget(desk_.target());
  } else if (desk_.hasHeight()) {
    mqtt_.publishTarget(desk_.height());
  }
}

void HaState::onLatestFirmware(const String& version) {
  latest_firmware_ = version;
  mqtt_.publishFirmwareLatest(version);
  publishFirmware();
}

// A failed install (-1) puts the entity back in its idle state.
void HaState::onUpdateProgress(int percent) {
  if (percent < 0) publishFirmware();
  else mqtt_.publishFirmwareProgress(percent);
}

void HaState::publishFirmware() {
  mqtt_.publishFirmwareState(releaseUrl());
}

// While an update is available, a GitHub comparison of the installed and the
// published commit; otherwise the channel's release page. A comparison needs
// a commit at both ends, which a local "dev" build doesn't have.
String HaState::releaseUrl() const {
  if (latest_firmware_ != cfg::kSwVersion && isCommit(cfg::kSwVersion) &&
      isCommit(latest_firmware_)) {
    return String(cfg::kOtaCompareUrl) + cfg::kSwVersion + "..." + latest_firmware_;
  }
  return cfg::kOtaReleaseUrl;
}

bool HaState::requestAlarm(bool on) {
  bool publish;
  const bool send = alarm_switch_.request(on, desk_.alarmOn(), millis(),
                                          cfg::kAlarmToggleSettle, publish);
  mqtt_.publishAlarm(publish);
  return send;
}

bool HaState::requestChildLock(bool locked) {
  bool publish;
  const bool send = lock_switch_.request(locked, desk_.childLocked(), millis(),
                                         cfg::kChildLockToggleSettle, publish);
  mqtt_.publishChildLock(publish);
  return send;
}

void HaState::publishSwitches() {
  // sync() records what it returns as published, so only call it while a
  // publish can actually go out.
  if (!mqtt_.connected()) return;
  const uint32_t now = millis();
  bool state;
  if (lock_switch_.sync(desk_.childLocked(), now, state)) mqtt_.publishChildLock(state);
  if (alarm_switch_.sync(desk_.alarmOn(), now, state)) mqtt_.publishAlarm(state);
  if (movement_available_.sync(!desk_.childLocked(), now, state))
    mqtt_.publishMovementAvailable(state);
}

void HaState::publishCalibration() {
  if (!calibration_pending_ || !mqtt_.connected()) return;
  if (mqtt_.publishCalibration(desk_.learnedState())) calibration_pending_ = false;
}

// One-line summary per completed seek: how the coast prediction compared to
// reality and what the corrections had to clean up — the ground truth for
// judging the motion model's convergence from the HA log.
void HaState::publishSeekReport(const DeskController::SeekReport& r) {
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
  mqtt_.publishLog(m);
}
