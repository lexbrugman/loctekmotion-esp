#include "DeskController.h"

#include <Arduino.h>

#include "MotionStore.h"
#include "config.h"

namespace {

DeskMotionPlanner::Settings plannerSettings() {
  DeskMotionPlanner::Settings s;
  s.wake_settle = cfg::kWakeSettle;
  s.command_interval = cfg::kCommandInterval;
  s.move_timeout = cfg::kMoveTimeout;
  s.height_stale_timeout = cfg::kHeightStaleTimeout;
  s.first_report_timeout = cfg::kFirstReportTimeout;
  s.motion_window = cfg::kMotionWindow;
  s.target_timeout = cfg::kTargetTimeout;
  s.wake_retry_interval = cfg::kWakeRetryInterval;
  s.target_deadband = cfg::kTargetDeadband;
  s.coarse_target_deadband = cfg::kCoarseTargetDeadband;
  s.seek_settle_delay = cfg::kSeekSettleDelay;
  s.stable_duration = cfg::kStableDuration;
  s.correction_tap_max = cfg::kCorrectionTapMax;
  s.terminal_speed_drive_min = cfg::kTerminalSpeedDriveMin;
  s.fine_height_limit = cfg::kFineHeightLimit;
  s.speed_window = cfg::kSpeedWindow;
  s.speed_min_span = cfg::kSpeedMinSpan;
  s.speed_max_age = cfg::kSpeedMaxAge;
  return s;
}

MotionModel::Tunables motionTunables() {
  MotionModel::Tunables t;
  t.seed_terminal_speed = cfg::kTerminalSpeedSeed;
  t.seed_decel = cfg::kDecelSeed;
  t.seed_tap_gain = cfg::kTapGainSeed;
  t.speed_learning_rate = cfg::kSpeedLearningRate;
  t.decel_learning_rate = cfg::kDecelLearningRate;
  t.tap_learning_rate = cfg::kTapLearningRate;
  t.min_terminal_speed = cfg::kTerminalSpeedMin;
  t.max_terminal_speed = cfg::kTerminalSpeedMax;
  t.min_decel = cfg::kDecelMin;
  t.max_decel = cfg::kDecelMax;
  t.min_tap_gain = cfg::kTapGainMin;
  t.max_tap_gain = cfg::kTapGainMax;
  return t;
}

}  // namespace

DeskController::DeskController()
    : serial_(cfg::kPinUartRx, cfg::kPinUartTx),
      display_(cfg::kAlarmClearTimeout),
      planner_(cfg::kMinHeight, cfg::kMaxHeight, plannerSettings(), motionTunables(),
               [this](const desk_cmd::Frame& frame) { send(frame); }) {}

void DeskController::begin() {
  pinMode(cfg::kPinScreen, OUTPUT);
  digitalWrite(cfg::kPinScreen, HIGH);  // keep the handset awake (PIN20 high)
  serial_.begin(cfg::kUartBaud);

  DeskMotionPlanner::LearnedState s{};
  if (motion_store::load(s)) planner_.setLearnedState(s);
  saved_state_ = planner_.learnedState();
}

void DeskController::loop() {
  // Drain the RX line and decode any complete frames.
  Stream& link = serial_.stream();
  while (link.available() > 0) {
    switch (decoder_.feed(static_cast<uint8_t>(link.read()))) {
      case DeskHeightDecoder::Reading::kHeight:
        display_.onHeight(millis(), decoder_.height());
        if (height_cb_) height_cb_(decoder_.height());
        break;
      case DeskHeightDecoder::Reading::kLock:
        display_.onLockDisplay();
        break;
      case DeskHeightDecoder::Reading::kAlarm:
        display_.onAlarmDisplay(millis());
        break;
      case DeskHeightDecoder::Reading::kNone:
        break;
    }
  }

  const uint32_t now = millis();
  display_.tick(now);

  planner_.tick(now, decoder_.hasHeight(), decoder_.height(),
                now - display_.lastHeightAt());

  // Sampled post-tick, so the end of a seek is seen exactly once — whether it
  // ended inside tick() or through stop() called from elsewhere.
  const bool pursuing = planner_.pursuingTarget();
  if (was_pursuing_target_ && !pursuing) {
    const DeskMotionPlanner::LearnedState state = planner_.learnedState();
    if (state != saved_state_ && motion_store::save(state)) saved_state_ = state;
    // The planner has a report only when the seek genuinely completed —
    // aborts (STOP, timeouts, travel limits) end it without one.
    DeskMotionPlanner::SeekReport report;
    const bool completed = planner_.takeSeekReport(report);
    if (seek_ended_cb_) seek_ended_cb_(completed ? &report : nullptr);
  }
  was_pursuing_target_ = pursuing;
}

void DeskController::send(const desk_cmd::Frame& frame) {
  if (frame.data == nullptr || frame.len == 0) return;
  serial_.stream().write(frame.data, frame.len);
}

void DeskController::moveToHeight(float target_cm) {
  planner_.moveToHeight(target_cm, millis(), decoder_.hasHeight(), decoder_.height());
}

void DeskController::childLock() {
  planner_.holdCommand(desk_cmd::ChildLock, millis(), cfg::kChildLockHold);
}

void DeskController::alarmOff() {
  planner_.holdCommand(desk_cmd::Alarm, millis(), cfg::kAlarmOffHold);
}

void DeskController::resetCalibration() {
  motion_store::reset();
  planner_.resetLearnedState();
  saved_state_ = planner_.learnedState();  // the seeds, now that nothing is stored
}
