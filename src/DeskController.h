#pragma once

#include <Arduino.h>

#include <cstdint>
#include <functional>

#include "DeskCommands.h"
#include "DeskHeightDecoder.h"
#include "DeskMotionPlanner.h"
#include "DeskUart.h"
#include "MotionStore.h"

// DeskController owns the serial link to the control box and the screen-wake
// pin, decodes the height stream, and adapts DeskMotionPlanner's movement
// decisions (high-level intents -> safely timed UART frames) to the real
// hardware. Everything is non-blocking: call begin() once and loop() every
// iteration.
class DeskController {
 public:
  using HeightCallback = std::function<void(float height)>;
  using SeekReport = DeskMotionPlanner::SeekReport;
  using SeekCallback = std::function<void(const SeekReport&)>;

  DeskController(uint8_t rx_pin, uint8_t tx_pin, uint8_t screen_pin, uint32_t baud,
                 float min_height, float max_height);

  void begin();
  void loop();

  // Register a callback fired whenever a fresh height value is decoded.
  void onHeight(HeightCallback cb) { height_cb_ = std::move(cb); }

  // Register a callback fired once per *completed* seek (see SeekReport);
  // aborted seeks don't fire it. Runs from loop() after the learned state has
  // been persisted, so learnedState() already reflects the seek's updates.
  void onSeekDone(SeekCallback cb) { seek_cb_ = std::move(cb); }

  // Current learned motion calibration, for telemetry.
  DeskMotionPlanner::LearnedState learnedState() const { return planner_.learnedState(); }

  bool has_height() const { return decoder_.has_height(); }
  float height() const { return decoder_.height(); }

  // Position as a 0..1 fraction of travel (min..max). Returns -1 if unknown.
  float position() const;

  // --- Continuous movement (cover open/close/stop) ---
  void moveUp() { planner_.moveUp(millis()); }
  void moveDown() { planner_.moveDown(millis()); }
  void stop() { planner_.stop(); }
  bool moving() const { return planner_.moving(); }

  // --- Go to an absolute height (cm), clamped to the configured range ---
  void moveToHeight(float target_cm);
  // --- Go to a 0..1 position fraction ---
  void moveToPosition(float fraction);

  // --- One-shot commands (wake handled automatically) ---
  void preset1() { issue(desk_cmd::Preset1); }
  void preset2() { issue(desk_cmd::Preset2); }
  void sit() { issue(desk_cmd::Sit); }
  void stand() { issue(desk_cmd::Stand); }
  void memory() { issue(desk_cmd::Memory); }
  // Arms the sit-stand reminder alarm (a tap; on the desk a further tap
  // cycles the interval); the display shows "=XX" with a blinking '=' — see
  // loop()'s alarm tracking and alarmOn().
  void alarm() { issue(desk_cmd::Alarm); }
  // Disarms the alarm: the desk only accepts this as a ~3 s held button
  // press — a tap would merely cycle the reminder interval.
  void alarmOff();
  // Best-known alarm state, derived from the desk's own display (see loop()).
  bool alarmOn() const { return alarm_on_; }
  // Child lock only registers on the desk when the command is held, like a
  // long handset button-press — a single tap (issue()) looks like Memory to
  // it, since both share the same raw frame. See DeskMotionPlanner::holdCommand.
  void childLock();
  // Best-known child-lock state — see loop()'s "LOC"/height detection for how
  // this is derived.
  bool childLocked() const { return child_locked_; }
  void wakeScreen() { issue(desk_cmd::Wake); }

  // Wipe the persisted learned state and reset in-memory values back to the
  // compile-time seeds. The next seek will start fresh calibration.
  void resetCalibration();

 private:
  void send(const desk_cmd::Frame& frame);
  void issue(const desk_cmd::Frame& frame) { planner_.issue(frame, millis()); }

  DeskUart serial_;
  uint8_t screen_pin_;
  uint32_t baud_;
  float min_height_;
  float max_height_;

  DeskHeightDecoder decoder_;
  HeightCallback height_cb_;
  SeekCallback seek_cb_;
  uint32_t last_height_at_ = 0;
  DeskMotionPlanner planner_;
  bool was_seeking_ = false;

  // Child-lock state, derived from the desk's own display (see loop()): a
  // "LOC" frame means locked, any successfully-decoded height means unlocked.
  bool child_locked_ = false;

  // Alarm state, derived from the display's "=XX" frames. Unlike child lock,
  // a height frame is *expected* while the alarm is on (movement shows the
  // live height), so clearing needs the static-height rule in loop().
  bool alarm_on_ = false;
  uint32_t last_alarm_frame_at_ = 0;
  float last_seen_height_ = -1.0f;      // last decoded value...
  uint32_t last_height_change_at_ = 0;  // ...and when it last differed
};
