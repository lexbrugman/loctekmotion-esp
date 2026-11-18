#pragma once

#include <cstdint>

// Child-lock and sit-stand alarm state, inferred from what the handset
// display shows — the desk reports neither directly.
//
//   child lock - the display shows "LOC" in place of the height while
//   locked, so a "LOC" frame means locked and any height frame means
//   unlocked.
//
//   alarm - while the reminder is armed the display shows "=XX" (blinking
//   '=') instead of the height; movement still works and shows the live
//   height, returning to "=XX" shortly after the desk stops, and the display
//   never sleeps (the blink keeps it streaming). A height frame alone
//   therefore can't mean "alarm off" the way it means "not locked": the state
//   clears only on positive contrary evidence — a height that has stayed
//   static for clear_timeout with no "=" frame for equally long, or a fully
//   dark display (neither height nor "=" frames) for that long.
//
// Pure logic (callers pass the time in), free of any Arduino dependency, so
// it can be exercised by host-side unit tests.
class DisplayState {
 public:
  explicit DisplayState(uint32_t clear_timeout) : clear_timeout_(clear_timeout) {}

  // A frame showing a valid height.
  void onHeight(uint32_t at, float height) {
    last_height_at_ = at;
    child_locked_ = false;
    if (height != last_height_) {
      last_height_ = height;
      last_height_change_at_ = at;
    }
    // With the alarm armed, the display only shows the height during (and
    // briefly after) movement before returning to "=XX" — so a height that
    // has stayed static this long, with no "=" frame for equally long, is
    // positive evidence the alarm was switched off (e.g. from the handset).
    if (alarm_on_ && at - last_alarm_frame_at_ > clear_timeout_ &&
        at - last_height_change_at_ > clear_timeout_) {
      alarm_on_ = false;
    }
  }

  // A frame showing "LOC".
  void onLockDisplay() { child_locked_ = true; }

  // A frame showing "=XX".
  void onAlarmDisplay(uint32_t at) {
    alarm_on_ = true;
    last_alarm_frame_at_ = at;
  }

  // Call every loop iteration: applies the dark-display rule.
  void tick(uint32_t now) {
    if (alarm_on_ && now - last_alarm_frame_at_ > clear_timeout_ &&
        now - last_height_at_ > clear_timeout_) {
      alarm_on_ = false;
    }
  }

  bool childLocked() const { return child_locked_; }
  bool alarmOn() const { return alarm_on_; }
  // When the last height frame arrived (0 before the first).
  uint32_t lastHeightAt() const { return last_height_at_; }

 private:
  uint32_t clear_timeout_;
  bool child_locked_ = false;
  bool alarm_on_ = false;
  uint32_t last_height_at_ = 0;
  uint32_t last_alarm_frame_at_ = 0;
  float last_height_ = -1.0f;           // last decoded value...
  uint32_t last_height_change_at_ = 0;  // ...and when it last differed
};
