#pragma once

#include <cstdint>

// Learned kinematic model of the desk, per travel direction:
//
//   terminal speed  - steady cruise speed of a long drive (cm/s)
//   deceleration    - braking rate after the drive stops (cm/s²), giving a
//                     coast distance of v²/(2·decel) from any speed v
//   tap gain        - distance covered by a short correction tap of duration
//                     t (seconds): distance = gain·t². Short taps never reach
//                     terminal speed, so distance grows quadratically with
//                     duration — a linear ms-per-cm model is structurally
//                     wrong for them.
//
// Up and down are learned independently: up fights gravity under load, down
// is assisted by it, and their accel/decel profiles genuinely differ.
//
// Every learned value updates via EMA and is clamped to a plausible range;
// setState() applies the same clamps so a corrupt flash restore degrades to
// the seeds instead of wedging the planner.
class MotionModel {
 public:
  struct Tunables {
    float seed_terminal_speed;  // cm/s
    float seed_decel;           // cm/s²
    float seed_tap_gain;        // cm/s²
    float speed_learning_rate;
    float decel_learning_rate;
    float tap_learning_rate;
    float min_terminal_speed, max_terminal_speed;
    float min_decel, max_decel;
    float min_tap_gain, max_tap_gain;
  };

  // Snapshot of all learned values — the persistence exchange format.
  struct State {
    float terminal_speed_up = 0.0f;
    float terminal_speed_down = 0.0f;
    float decel_up = 0.0f;
    float decel_down = 0.0f;
    float tap_gain_up = 0.0f;
    float tap_gain_down = 0.0f;
  };

  explicit MotionModel(Tunables tunables) : tunables_(tunables) { reset(); }

  // --- Predictions ---
  float terminalSpeed(bool up) const {
    return up ? state_.terminal_speed_up : state_.terminal_speed_down;
  }
  // How far the desk keeps coasting after the drive stops at speed |speed|.
  float coastDistance(bool up, float speed) const;
  // Tap drive duration (ms) expected to close |error_cm|, capped at max_ms.
  uint32_t tapDurationMs(bool up, float error_cm, uint32_t max_ms) const;

  // --- Learning (callers gate on measurement quality; see DeskMotionPlanner) ---
  // A long drive's speed just before the stop — a terminal-speed observation.
  void learnTerminalSpeed(bool up, float measured);
  // Observed coast: the drive stopped at |speed_at_stop| and travelled a
  // further |coast_cm| before settling.
  void learnCoast(bool up, float speed_at_stop, float coast_cm);
  // A correction tap of |duration_ms| moved the desk |moved_cm|.
  void learnTap(bool up, uint32_t duration_ms, float moved_cm);

  // --- Persistence / reset ---
  State state() const { return state_; }
  void setState(const State& s);  // sanitizes implausible values back to seeds
  void reset();                   // back to the compile-time seeds

 private:
  float& decel(bool up) { return up ? state_.decel_up : state_.decel_down; }
  float& tapGain(bool up) { return up ? state_.tap_gain_up : state_.tap_gain_down; }
  float& terminal(bool up) {
    return up ? state_.terminal_speed_up : state_.terminal_speed_down;
  }

  Tunables tunables_;
  State state_;
};
