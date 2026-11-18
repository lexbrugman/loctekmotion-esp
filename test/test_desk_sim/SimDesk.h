#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>

#include "DeskCommands.h"
#include "DeskCrc.h"

// A simulated LoctekMotion control box: motor kinematics plus the display
// stream it sends back, at the byte level. It receives the same command
// frames the firmware sends and emits the same 7-segment height reports the
// real box does, so tests can run the real decoder and planner against it
// end to end.
//
// The physics are deliberately simple — accelerate toward a cruise speed
// while drive frames keep arriving, brake at a constant rate once they stop —
// because that is the shape MotionModel assumes. The numbers in Params are
// plausible guesses, not measurements of a real desk: tests check that the
// planner converges on *a* desk, not that the seeds match any particular one.
class SimDesk {
 public:
  struct Params {
    // Not an aggregate: fill the fields in by name, never positionally.
    constexpr Params() {}

    float min_height = 65.5f;    // travel limits (cm)
    float max_height = 131.0f;
    float start_height = 75.0f;
    float speed_up = 2.8f;       // cruise speed (cm/s)
    float speed_down = 3.0f;
    float accel_up = 6.0f;       // ramp to cruise (cm/s²)
    float accel_down = 8.0f;
    float decel_up = 3.0f;       // braking once the drive stops (cm/s²)
    float decel_down = 3.0f;
    // The box keeps driving this long after the last drive frame, so frames
    // at the firmware's cadence drive continuously.
    uint32_t drive_hold = 150;
    // After the display wakes, frames are ignored for this long; the frame
    // that woke it is ignored too.
    uint32_t wake_latency = 100;
    uint32_t report_interval = 108;  // display report cadence while awake
    uint32_t sleep_after = 10000;    // display sleeps once idle this long
    // Display at/above this height shows whole centimetres (see decoder).
    float fine_height_limit = 100.0f;
  };

  using ByteSink = std::function<void(uint8_t)>;

  SimDesk(Params params, uint32_t now)
      : p_(params), pos_(params.start_height), last_t_(now) {}

  // A command frame from the firmware, as written to the UART.
  void receive(uint32_t now, const uint8_t* data, size_t len) {
    advance(now, nullptr);
    if (!awake(now)) {
      awake_since_ = now;  // this frame only wakes the display
      last_activity_ = now;
      return;
    }
    last_activity_ = now;
    if (now - awake_since_ < p_.wake_latency) return;
    if (is(data, len, desk_cmd::kUp)) drive(now, +1);
    else if (is(data, len, desk_cmd::kDown)) drive(now, -1);
  }

  // Run the physics up to |now|, emitting display bytes to |sink| (may be
  // null). Call often — every simulated millisecond.
  void advance(uint32_t now, const ByteSink& sink) {
    while (last_t_ != now) {
      ++last_t_;
      step(last_t_);
      if (!rx_cut_ && awake(last_t_) && last_t_ - last_report_ >= p_.report_interval) {
        last_report_ = last_t_;
        emitReport(sink);
      }
    }
  }

  // Stop the display stream reaching the firmware (a dead RX line); the box
  // itself keeps responding to commands.
  void cutRx() { rx_cut_ = true; }

  float position() const { return pos_; }
  float velocity() const { return vel_; }
  bool atRest(uint32_t now) const { return vel_ == 0.0f && !driving(now); }
  // What the display shows now.
  float displayed() const { return quantize(pos_); }

 private:
  static bool is(const uint8_t* data, size_t len, const std::array<uint8_t, 8>& frame) {
    return len == frame.size() && std::memcmp(data, frame.data(), len) == 0;
  }

  bool awake(uint32_t now) const {
    return hasWoken() && (vel_ != 0.0f || now - last_activity_ < p_.sleep_after);
  }
  bool hasWoken() const { return awake_since_ != kNever; }
  bool driving(uint32_t now) const {
    return drive_dir_ != 0 && now - last_drive_at_ < p_.drive_hold;
  }

  void drive(uint32_t now, int dir) {
    drive_dir_ = dir;
    last_drive_at_ = now;
  }

  // One millisecond of motion.
  void step(uint32_t now) {
    constexpr float dt = 0.001f;
    const bool drv = driving(now);
    const bool up = drv ? drive_dir_ > 0 : vel_ > 0.0f;
    const float decel = up ? p_.decel_up : p_.decel_down;
    if (drv) {
      const float target = drive_dir_ > 0 ? p_.speed_up : -p_.speed_down;
      const bool reversing = (vel_ > 0.0f && target < 0.0f) || (vel_ < 0.0f && target > 0.0f);
      const float rate = reversing ? decel : (drive_dir_ > 0 ? p_.accel_up : p_.accel_down);
      vel_ = approach(vel_, target, rate * dt);
    } else {
      drive_dir_ = 0;
      vel_ = approach(vel_, 0.0f, decel * dt);
    }
    pos_ += vel_ * dt;
    if (pos_ >= p_.max_height) { pos_ = p_.max_height; vel_ = std::fmin(vel_, 0.0f); }
    if (pos_ <= p_.min_height) { pos_ = p_.min_height; vel_ = std::fmax(vel_, 0.0f); }
  }

  static float approach(float v, float target, float max_step) {
    if (v < target) return std::fmin(v + max_step, target);
    return std::fmax(v - max_step, target);
  }

  // Tenths below the fine limit, whole centimetres from it up.
  float quantize(float h) const {
    const float tenths = std::round(h * 10.0f) / 10.0f;
    return tenths < p_.fine_height_limit ? tenths : std::round(h);
  }

  void emitReport(const ByteSink& sink) {
    if (!sink) return;
    static constexpr uint8_t kSeg[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66,
                                       0x6d, 0x7d, 0x07, 0x7f, 0x6f};
    constexpr uint8_t kDp = 0x80;
    const float shown = quantize(pos_);
    uint8_t d[3];
    if (shown < p_.fine_height_limit) {
      const long t = std::lround(shown * 10.0f);  // e.g. 725 -> "7" "2." "5"
      d[0] = kSeg[t / 100];
      d[1] = static_cast<uint8_t>(kSeg[(t / 10) % 10] | kDp);
      d[2] = kSeg[t % 10];
    } else {
      const long n = std::lround(shown);
      d[0] = kSeg[n / 100];
      d[1] = kSeg[(n / 10) % 10];
      d[2] = kSeg[n % 10];
    }
    uint8_t f[9] = {0x9b, 0x07, 0x12, d[0], d[1], d[2], 0, 0, 0x9d};
    const uint16_t crc = desk_proto::crc16(f + 1, 5);
    f[6] = static_cast<uint8_t>(crc >> 8);
    f[7] = static_cast<uint8_t>(crc & 0xff);
    for (uint8_t b : f) sink(b);
  }

  static constexpr uint32_t kNever = UINT32_MAX;

  Params p_;
  float pos_;
  float vel_ = 0.0f;
  uint32_t last_t_;
  int drive_dir_ = 0;  // +1 up, -1 down, 0 none
  uint32_t last_drive_at_ = 0;
  uint32_t awake_since_ = kNever;
  uint32_t last_activity_ = 0;
  uint32_t last_report_ = 0;
  bool rx_cut_ = false;
};
