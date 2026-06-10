#include "MotionModel.h"

#include <cmath>

namespace {

float clampTo(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

// Clamp to range, but treat anything outside it as "implausible / corrupt"
// and fall back to the seed rather than pinning at the boundary — a restore
// of 0 should mean "start over", not "assume the desk barely decelerates".
float sanitize(float v, float lo, float hi, float seed) {
  return (v < lo || v > hi) ? seed : v;
}

}  // namespace

void MotionModel::reset() {
  state_.terminal_speed_up = tunables_.seed_terminal_speed;
  state_.terminal_speed_down = tunables_.seed_terminal_speed;
  state_.decel_up = tunables_.seed_decel;
  state_.decel_down = tunables_.seed_decel;
  state_.tap_gain_up = tunables_.seed_tap_gain;
  state_.tap_gain_down = tunables_.seed_tap_gain;
}

void MotionModel::setState(const State& s) {
  const Tunables& t = tunables_;
  state_.terminal_speed_up = sanitize(s.terminal_speed_up, t.min_terminal_speed,
                                      t.max_terminal_speed, t.seed_terminal_speed);
  state_.terminal_speed_down = sanitize(s.terminal_speed_down, t.min_terminal_speed,
                                        t.max_terminal_speed, t.seed_terminal_speed);
  state_.decel_up = sanitize(s.decel_up, t.min_decel, t.max_decel, t.seed_decel);
  state_.decel_down = sanitize(s.decel_down, t.min_decel, t.max_decel, t.seed_decel);
  state_.tap_gain_up =
      sanitize(s.tap_gain_up, t.min_tap_gain, t.max_tap_gain, t.seed_tap_gain);
  state_.tap_gain_down =
      sanitize(s.tap_gain_down, t.min_tap_gain, t.max_tap_gain, t.seed_tap_gain);
}

float MotionModel::coastDistance(bool up, float speed) const {
  const float d = up ? state_.decel_up : state_.decel_down;
  return (speed * speed) / (2.0f * d);
}

uint32_t MotionModel::tapDurationMs(bool up, float error_cm, uint32_t max_ms) const {
  const float gain = up ? state_.tap_gain_up : state_.tap_gain_down;
  // distance = gain·t²  ->  t = sqrt(distance / gain), in seconds.
  const float t_ms = std::sqrt(error_cm / gain) * 1000.0f;
  if (t_ms < 1.0f) return 1;
  if (t_ms > static_cast<float>(max_ms)) return max_ms;
  return static_cast<uint32_t>(t_ms);
}

void MotionModel::learnTerminalSpeed(bool up, float measured) {
  float& v = terminal(up);
  v += tunables_.speed_learning_rate * (measured - v);
  v = clampTo(v, tunables_.min_terminal_speed, tunables_.max_terminal_speed);
}

void MotionModel::learnCoast(bool up, float speed_at_stop, float coast_cm) {
  if (coast_cm <= 0.0f || speed_at_stop <= 0.0f) return;
  const float observed = (speed_at_stop * speed_at_stop) / (2.0f * coast_cm);
  float& d = decel(up);
  d += tunables_.decel_learning_rate * (observed - d);
  d = clampTo(d, tunables_.min_decel, tunables_.max_decel);
}

void MotionModel::learnTap(bool up, uint32_t duration_ms, float moved_cm) {
  if (duration_ms == 0 || moved_cm <= 0.0f) return;
  const float t_s = static_cast<float>(duration_ms) / 1000.0f;
  const float observed = moved_cm / (t_s * t_s);
  float& g = tapGain(up);
  g += tunables_.tap_learning_rate * (observed - g);
  g = clampTo(g, tunables_.min_tap_gain, tunables_.max_tap_gain);
}
