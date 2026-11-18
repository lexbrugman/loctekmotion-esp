#pragma once

#include <Arduino.h>

#include <functional>

#include "DeskCommands.h"
#include "DeskHeightDecoder.h"
#include "DeskMotionPlanner.h"
#include "DeskUart.h"
#include "DisplayState.h"

// DeskController owns the serial link to the control box and the screen-wake
// pin, decodes the height stream, and adapts DeskMotionPlanner's movement
// decisions (high-level intents -> safely timed UART frames) to the real
// hardware. Everything is non-blocking: call begin() once and loop() every
// iteration.
class DeskController {
 public:
  using HeightCallback = std::function<void(float height)>;
  using SeekReport = DeskMotionPlanner::SeekReport;
  // |report| is the completed seek's report, or nullptr when the seek was
  // aborted or never started (see onSeekEnded).
  using SeekEndedCallback = std::function<void(const SeekReport* report)>;

  // Pins, link speed, travel range and every tunable come from config.h.
  DeskController();

  void begin();
  void loop();

  // Register a callback fired whenever a fresh height value is decoded.
  void onHeight(HeightCallback cb) { height_cb_ = std::move(cb); }

  // Register a callback fired once each time a slider seek ends —
  // pursuingTarget() going false — however it ends: completed (with its
  // SeekReport), aborted (STOP, a timeout, a travel limit), or given up
  // while still waiting for a first height report (both without one). Runs
  // from loop() after any new learned state has been persisted, so
  // learnedState() already reflects the seek's updates.
  void onSeekEnded(SeekEndedCallback cb) { seek_ended_cb_ = std::move(cb); }

  // Current learned motion calibration, for telemetry.
  DeskMotionPlanner::LearnedState learnedState() const { return planner_.learnedState(); }

  bool hasHeight() const { return decoder_.hasHeight(); }
  float height() const { return decoder_.height(); }

  // Halts every move, including one the desk is making on its own (a preset,
  // sit/stand) — see DeskMotionPlanner::halt.
  void stop() { planner_.halt(millis()); }
  // True while the planner has anything in flight that depends on loop()
  // running on time (see DeskMotionPlanner::busy) — hold off blocking work.
  bool busy() const { return planner_.busy(); }
  // True while driving to a slider-commanded absolute height (see planner).
  bool pursuingTarget() const { return planner_.pursuingTarget(); }
  // The clamped target being pursued; only meaningful while pursuingTarget().
  float target() const { return planner_.target(); }

  // --- Go to an absolute height (cm), clamped to the configured range ---
  void moveToHeight(float target_cm);

  // --- Key presses ---
  // A single tap of a handset key (desk_cmd::Preset1, Sit, Wake, ...); wakes
  // the desk first. A tap of desk_cmd::Alarm arms the sit-stand reminder (a
  // further tap cycles its interval).
  void press(const desk_cmd::Frame& frame) { planner_.issue(frame, millis()); }
  // Disarms the alarm: the desk only accepts this as a ~3 s held button
  // press — a tap would merely cycle the reminder interval.
  void alarmOff();
  // Best-known alarm state, derived from the desk's own display (see
  // DisplayState).
  bool alarmOn() const { return display_.alarmOn(); }
  // Child lock only registers on the desk when the command is held, like a
  // long handset button-press — a single tap (press()) looks like Memory to
  // it, since both share the same raw frame. See DeskMotionPlanner::holdCommand.
  void childLock();
  // Best-known child-lock state, derived from the desk's own display (see
  // DisplayState).
  bool childLocked() const { return display_.childLocked(); }

  // Wipe the persisted learned state and reset in-memory values back to the
  // compile-time seeds. The next seek will start fresh calibration.
  void resetCalibration();

 private:
  void send(const desk_cmd::Frame& frame);

  DeskUart serial_;

  DeskHeightDecoder decoder_;
  HeightCallback height_cb_;
  SeekEndedCallback seek_ended_cb_;
  DisplayState display_;  // child lock / alarm, read off the display frames
  DeskMotionPlanner planner_;
  bool was_pursuing_target_ = false;
  // Learned state as last persisted, so flash is only rewritten when a seek
  // actually changed it.
  DeskMotionPlanner::LearnedState saved_state_{};
};
