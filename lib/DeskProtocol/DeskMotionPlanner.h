#pragma once

#include <cstdint>
#include <functional>

#include "DeskCommands.h"
#include "MotionModel.h"
#include "VelocityEstimator.h"

// DeskMotionPlanner is the desk's movement state machine: continuous up/down
// driving with safety limits, a travel timeout and a height-staleness abort,
// absolute-height seeking with closed-loop correction, held-command sequencing
// (mimicking a long handset button-press), and the wake/settle sequencing
// one-shot commands need.
//
// Seeking is physics-based: a VelocityEstimator derives the live travel speed
// from the height stream, and a learned MotionModel predicts how far the desk
// will coast once driving stops — the drive ends when the remaining distance
// drops to the predicted coast. Between height reports the position is
// dead-reckoned forward at the measured speed, so the stop point isn't
// quantized to the report cadence. When no clean speed measurement is available
// (e.g. above the display's fine-resolution range), the learned terminal
// speed is assumed instead; that overestimates the coast during ramp-up, so
// errors fall on the undershoot side and the correction tap closes the gap
// driving in the same direction as the seek.
//
// It owns no hardware — callers feed it the current time and the latest known
// height, and it reports the frames it wants sent via a callback. That keeps
// it free of any Arduino/ESP dependency so it can be exercised by host-side
// unit tests; DeskController is the thin adapter that wires it to the real
// UART link and clock.
class DeskMotionPlanner {
 public:
  using SendFn = std::function<void(const desk_cmd::Frame&)>;

  struct Timing {
    uint32_t wake_settle;           // delay after waking before the first command
    uint32_t command_interval;      // cadence of repeated movement/hold frames
    uint32_t move_timeout;          // safety net: max duration of one continuous move
    uint32_t height_stale_timeout;  // abort an active move if height reports stop
    uint32_t target_timeout;        // give up awaiting a height report for moveToHeight
    uint32_t wake_retry_interval;   // re-wake cadence while awaiting that report
    float target_deadband;          // "close enough" to a moveToHeight target
    float coarse_target_deadband;   // deadband at/above fine_height_limit, where
                                    // readings are whole centimetres
    uint32_t hold_duration;         // how long to hold a held-style command
    uint32_t seek_settle_delay;     // max wait for stability; fires early once stable
    uint32_t stable_duration;       // height unchanged this long before sampling
    uint32_t correction_tap_max;    // ceiling on a single correction tap drive (ms)
    uint32_t terminal_speed_drive_min;  // drive must run this long for its stop speed
                                        // to count as a terminal-speed observation
    float fine_height_limit;        // display resolution drops to 1 cm at/above this;
                                    // speed estimation and learning are gated below it
    uint32_t speed_window;          // estimator slope window (ms)
    uint32_t speed_min_span;        // min sample span for a valid slope (ms)
    uint32_t speed_max_age;         // newest sample older than this -> no estimate (ms)
  };

  // Snapshot of all learned values — used to persist and restore across power
  // cycles (see MotionStore).
  using LearnedState = MotionModel::State;

  DeskMotionPlanner(float min_height, float max_height, Timing timing,
                    MotionModel::Tunables tunables, SendFn send);

  // --- Continuous movement ---
  void moveUp(uint32_t now);
  void moveDown(uint32_t now);
  void stop();
  bool moving() const { return motor_.mode == Mode::kUp || motor_.mode == Mode::kDown; }

  // --- Seek an absolute height (cm), clamped to the configured range ---
  void moveToHeight(float target_cm, uint32_t now, bool has_height, float height);

  // True while a seek (including post-drive settling and correction taps) is
  // in progress — use this to detect when a seek fully completes.
  bool seeking() const { return seek_.active; }

  // --- One-shot command (wake, settle, then send once) ---
  void issue(const desk_cmd::Frame& frame, uint32_t now);

  // --- Held command (wake, settle, then repeat at cadence for hold_duration) ---
  void holdCommand(const desk_cmd::Frame& frame, uint32_t now);

  // Advance the state machine; call every loop iteration.
  void tick(uint32_t now, bool has_height, float height, uint32_t height_age);

  // Access and restore learned state for persistence across power cycles.
  // setLearnedState sanitizes implausible values (e.g. a missing key or a
  // corrupt flash file restoring as 0) back to their seeds.
  LearnedState learnedState() const { return model_.state(); }
  void setLearnedState(const LearnedState& s) { model_.setState(s); }

  // Reset all learned values back to the compile-time seeds, as if the device
  // were freshly flashed. Call together with motion_store::reset() to start
  // calibration over from scratch.
  void resetLearnedState() { model_.reset(); }

 private:
  enum class Mode { kIdle, kUp, kDown, kHold };
  enum class SeekPhase { kNone, kDriving, kSettling };

  float deadbandAt(float height) const;
  void startMove(Mode mode, uint32_t now);
  void beginSeek(float target_cm, bool up, uint32_t now);
  void beginSettling(float height, uint32_t now);
  void beginCorrectionDrive(bool up, uint32_t duration, float start_height, uint32_t now);
  void servicePending(uint32_t now, bool has_height, float height);
  void serviceMovement(uint32_t now, bool has_height, float height, uint32_t height_age);
  void serviceSeekSettling(uint32_t now, bool has_height, float height);
  void serviceHold(uint32_t now);

  float min_height_;
  float max_height_;
  Timing timing_;
  SendFn send_;

  // Learned kinematics (coast prediction, tap sizing) and the live speed
  // estimate feeding it.
  MotionModel model_;
  VelocityEstimator estimator_;
  float last_fed_height_ = -1.0f;  // dedupe: feed each decoded value once
  uint32_t last_height_change_at_ = 0;  // when the reported height last changed —
                                        // dead-reckoning anchor between reports

  // Current drive mode and per-move timing counters. Shared by all move types
  // (continuous, seek drive, correction tap, hold).
  struct Motor {
    Mode mode = Mode::kIdle;
    uint32_t started_at = 0;    // when the current drive started (for move_timeout)
    uint32_t next_frame_at = 0; // earliest time the next command frame may be sent
    bool height_seen_fresh = false; // arms the staleness abort once the stream is live
  } motor_;

  // Absolute-height seek: active from beginSeek until stop().
  struct Seek {
    bool active = false;
    float target = 0.0f;
    SeekPhase phase = SeekPhase::kNone;
    bool up = false;
    uint32_t correction_attempts = 0;
    uint32_t correction_drive_until = 0;  // non-zero only during a correction tap
    uint32_t drive_started_at = 0;        // when the initial drive began
    // Where and how fast the initial drive stopped — the coast measurement's
    // starting point. stop_speed_measured is false when the speed was the
    // terminal-speed fallback, which must not be learned from.
    float stop_height = 0.0f;
    float stop_speed = 0.0f;
    bool stop_speed_measured = false;
  } seek_;

  // Post-drive settling: active while seek_.phase == kSettling.
  struct Settle {
    uint32_t started_at = 0;
    float last_height = 0.0f;   // last height seen — changes reset stable_since
    uint32_t stable_since = 0;  // when height last changed
  } settle_;

  // Metadata for the most recent correction tap, used to learn tap gain from
  // the observed movement once the next settle phase fires.
  struct Tap {
    uint32_t duration = 0;
    float start_height = 0.0f;
  } last_tap_;

  // Deferred moveToHeight: queued while no height reading is available yet.
  struct Deferred {
    bool active = false;
    float target = 0.0f;
    uint32_t set_at = 0;
    uint32_t last_wake = 0;
  } deferred_;

  // One-shot command awaiting its post-wake settle delay.
  struct OneShot {
    bool active = false;
    desk_cmd::Frame frame{nullptr, 0};
    uint32_t fire_at = 0;
  } one_shot_;

  // Held command: repeats at cadence until hold_.until.
  struct Hold {
    desk_cmd::Frame frame{nullptr, 0};
    uint32_t until = 0;
  } hold_;
};
