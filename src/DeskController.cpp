#include "DeskController.h"

#include <Arduino.h>

#include "MotionStore.h"
#include "config.h"

DeskController::DeskController(uint8_t rx_pin, uint8_t tx_pin, uint8_t screen_pin,
                               uint32_t baud, float min_height, float max_height)
    : serial_(rx_pin, tx_pin),
      screen_pin_(screen_pin),
      baud_(baud),
      min_height_(min_height),
      max_height_(max_height),
      planner_(min_height, max_height,
               DeskMotionPlanner::Timing{
                   cfg::kWakeSettle,
                   cfg::kCommandInterval,
                   cfg::kMoveTimeout,
                   cfg::kHeightStaleTimeout,
                   cfg::kTargetTimeout,
                   cfg::kWakeRetryInterval,
                   cfg::kTargetDeadband,
                   cfg::kCoarseTargetDeadband,
                   cfg::kChildLockHold,
                   cfg::kSeekSettleDelay,
                   cfg::kStableDuration,
                   cfg::kCorrectionTapMax,
                   cfg::kTerminalSpeedDriveMin,
                   cfg::kFineHeightLimit,
                   cfg::kSpeedWindow,
                   cfg::kSpeedMinSpan,
                   cfg::kSpeedMaxAge,
               },
               MotionModel::Tunables{
                   cfg::kTerminalSpeedSeed,
                   cfg::kDecelSeed,
                   cfg::kTapGainSeed,
                   cfg::kSpeedLearningRate,
                   cfg::kDecelLearningRate,
                   cfg::kTapLearningRate,
                   cfg::kTerminalSpeedMin,
                   cfg::kTerminalSpeedMax,
                   cfg::kDecelMin,
                   cfg::kDecelMax,
                   cfg::kTapGainMin,
                   cfg::kTapGainMax,
               },
               [this](const desk_cmd::Frame& frame) { send(frame); }) {}

void DeskController::begin() {
  pinMode(screen_pin_, OUTPUT);
  digitalWrite(screen_pin_, HIGH);  // keep the handset awake (PIN20 high)
  serial_.begin(baud_);

  DeskMotionPlanner::LearnedState s{};
  if (motion_store::load(s)) planner_.setLearnedState(s);
}

void DeskController::loop() {
  // Drain the RX line and decode any complete frames.
  Stream& link = serial_.stream();
  while (link.available() > 0) {
    uint8_t b = static_cast<uint8_t>(link.read());
    if (decoder_.feed(b)) {
      last_height_at_ = millis();
      // A normal height reading means the display isn't showing "LOC".
      child_locked_ = false;
      if (height_cb_) height_cb_(decoder_.height());
    }
    if (decoder_.isLockDisplay()) child_locked_ = true;
  }

  const uint32_t now = millis();
  planner_.tick(now, decoder_.has_height(), decoder_.height(), now - last_height_at_);

  // Persist learned values once per completed seek (sampled post-tick, so the
  // seeking -> idle edge is seen exactly once).
  const bool seeking = planner_.seeking();
  if (was_seeking_ && !seeking) {
    motion_store::save(planner_.learnedState());
    // The planner has a report only when the seek genuinely completed —
    // aborts (STOP, timeouts, travel limits) cross this edge without one.
    DeskMotionPlanner::SeekReport report;
    if (seek_cb_ && planner_.takeSeekReport(report)) seek_cb_(report);
  }
  was_seeking_ = seeking;
}

float DeskController::position() const {
  if (!decoder_.has_height()) return -1.0f;
  float frac = (decoder_.height() - min_height_) / (max_height_ - min_height_);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  return frac;
}

void DeskController::send(const desk_cmd::Frame& frame) {
  if (frame.data == nullptr || frame.len == 0) return;
  serial_.stream().write(frame.data, frame.len);
}

void DeskController::moveToHeight(float target_cm) {
  planner_.moveToHeight(target_cm, millis(), decoder_.has_height(), decoder_.height());
}

void DeskController::moveToPosition(float fraction) {
  if (fraction < 0.0f) fraction = 0.0f;
  if (fraction > 1.0f) fraction = 1.0f;
  moveToHeight(min_height_ + fraction * (max_height_ - min_height_));
}

void DeskController::resetCalibration() {
  motion_store::reset();
  planner_.resetLearnedState();
}
