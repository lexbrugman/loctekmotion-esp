#pragma once

#include <cstdint>
#include <functional>

#include "DeskCommands.h"
#include "MotionModel.h"
#include "VelocityEstimator.h"

// DeskMotionPlanner is the desk's movement state machine: absolute-height
// seeking with closed-loop correction — driven within safety limits, a travel
// timeout and a height-staleness abort — held-command sequencing
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

  struct Settings {
    // Not an aggregate: fill the fields in by name, never positionally.
    constexpr Settings() {}

    uint32_t wake_settle = 0;               // delay after waking before the first command
    uint32_t command_interval = 0;          // cadence of repeated movement/hold frames
    uint32_t move_timeout = 0;              // safety net: max duration of one drive
    uint32_t height_stale_timeout = 0;      // abort an active move if height reports stop
    uint32_t first_report_timeout = 0;      // abort a drive no fresh report has confirmed
                                            // within this long of its start
    uint32_t motion_window = 0;             // a height change this recent means the desk
                                            // is moving (decides whether halt() cancels)
    uint32_t target_timeout = 0;            // give up awaiting a height report for moveToHeight
    uint32_t wake_retry_interval = 0;       // re-wake cadence while awaiting that report
    float target_deadband = 0.0f;           // "close enough" to a moveToHeight target
    float coarse_target_deadband = 0.0f;    // deadband at/above fine_height_limit, where
                                            // readings are whole centimetres
    uint32_t seek_settle_delay = 0;         // max wait for stability; fires early once stable
    uint32_t stable_duration = 0;           // height unchanged this long before sampling
    uint32_t correction_tap_max = 0;        // ceiling on a single correction tap drive (ms)
    uint32_t terminal_speed_drive_min = 0;  // drive must run this long for its stop speed
                                            // to count as a terminal-speed observation
    float fine_height_limit = 0.0f;         // display resolution drops to 1 cm at/above this;
                                            // speed estimation and learning are gated below it
    uint32_t speed_window = 0;              // estimator slope window (ms)
    uint32_t speed_min_span = 0;            // min sample span for a valid slope (ms)
    uint32_t speed_max_age = 0;             // newest sample older than this -> no estimate (ms)
  };

  // Snapshot of all learned values — used to persist and restore across power
  // cycles (see MotionStore).
  using LearnedState = MotionModel::State;

  // Summary of a *completed* seek — settled within the deadband or exhausted
  // its correction attempts — for telemetry. Aborted seeks (STOP, timeouts,
  // travel limits) produce none.
  struct SeekReport {
    float target = 0.0f;
    float start_height = 0.0f;  // height when the seek began
    float stop_height = 0.0f;   // (dead-reckoned) position at drive cut-off
    float stop_speed = 0.0f;    // speed the coast prediction used (cm/s)
    bool stop_speed_measured = false;  // false -> terminal-speed fallback
    float predicted_coast = 0.0f;      // coast expected at cut-off (cm)
    float observed_coast = 0.0f;       // movement seen by the first settle (cm)
    float settled_height = 0.0f;       // final position
    uint32_t correction_taps = 0;
    uint32_t duration_ms = 0;  // drive start -> completion
  };

  DeskMotionPlanner(float min_height, float max_height, Settings settings,
                    MotionModel::Tunables tunables, SendFn send);

  // Stop everything this planner is doing or has queued: drives, seeks, held
  // commands and pending one-shots. It sends nothing — the desk halts a drive
  // by itself once the frames stop — so it is also what every internal abort
  // uses.
  void stop();
  // What a user's STOP should call: stop(), and also cancel any travel the
  // desk is making on its own (a preset, sit/stand, Memory recall), which
  // stop() alone leaves running. A preset key cancels such travel, but on an
  // idle desk it starts one, so the cancel (desk_cmd::Cancel) is only sent
  //   - when the planner wasn't driving the desk itself,
  //   - while the desk is seen moving (within motion_window), both now and
  //     after the wake_settle it waits behind a wake frame — the box ignores
  //     a bare cancel during a preset started on the handset,
  //   - once per travel: after any key (a cancel, or a preset pressed from
  //     HA) the desk is already coming to rest.
  // A new command or stop() before it fires drops it. A key held on the
  // handset can't be cancelled: the box ignores the cancel then (verified on
  // hardware — it doesn't start a move either).
  void halt(uint32_t now);
  // True while a drive frame is being repeated (a seek's drive or a
  // correction tap).
  bool moving() const { return state_ == State::kDriving || state_ == State::kTapping; }
  // True while anything is in flight that needs tick() to keep its timing: a
  // drive, a seek (including its deferred and settling phases), a held
  // command or a queued one-shot. Callers use it to hold off work that would
  // stall the loop.
  bool busy() const { return state_ != State::kIdle || one_shot_.active; }

  // --- Seek an absolute height (cm), clamped to the configured range ---
  void moveToHeight(float target_cm, uint32_t now, bool has_height, float height);

  // True while a seek (including post-drive settling and correction taps) is
  // in progress — use this to detect when a seek fully completes.
  bool seeking() const {
    return state_ == State::kDriving || state_ == State::kTapping || state_ == State::kSettling;
  }

  // True while pursuing an absolute-height target — the queued/deferred wake
  // phase plus the active seek and its settling. Distinguishes a slider-
  // commanded move from handset / preset motion.
  bool pursuingTarget() const { return seeking() || state_ == State::kAwaitingHeight; }
  // The target being pursued, clamped to the travel range. Only meaningful
  // while pursuingTarget().
  float target() const {
    return state_ == State::kAwaitingHeight ? deferred_.target : seek_.target;
  }

  // Fetch the report for the most recently completed seek. Returns each
  // report at most once (one-shot), false when nothing completed since the
  // last call.
  bool takeSeekReport(SeekReport& out);

  // --- One-shot command (wake, settle, then send once) ---
  void issue(const desk_cmd::Frame& frame, uint32_t now);

  // --- Held command (wake, settle, then repeat at cadence for |duration|) ---
  void holdCommand(const desk_cmd::Frame& frame, uint32_t now, uint32_t duration);

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
  // What the planner is doing. Exactly one at a time; a new command replaces
  // whatever was in progress. A queued one-shot (one_shot_) is the only thing
  // that runs alongside, and only beside kIdle or kAwaitingHeight.
  enum class State {
    kIdle,
    kAwaitingHeight,  // moveToHeight queued until a first height report arrives
    kDriving,         // seek: the initial drive toward the target
    kTapping,         // seek: a short correction tap
    kSettling,        // seek: waiting for the desk to come to rest
    kHolding,         // held command, repeated at cadence until hold_.until
  };

  static VelocityEstimator::Config estimatorConfig(const Settings& s);
  float deadbandAt(float height) const;
  void startMove(State state, uint32_t now);
  void beginSeek(float target_cm, bool up, uint32_t now, float height);
  void beginSettling(float height, uint32_t now);
  void beginCorrectionDrive(bool up, uint32_t duration, float start_height, uint32_t now);
  void servicePending(uint32_t now, bool has_height, float height);
  void serviceMovement(uint32_t now, bool has_height, float height, uint32_t height_age);
  void serviceSeekSettling(uint32_t now, bool has_height, float height);
  void serviceHold(uint32_t now);

  float min_height_;
  float max_height_;
  Settings settings_;
  SendFn send_;

  // Learned kinematics (coast prediction, tap sizing) and the live speed
  // estimate feeding it.
  MotionModel model_;
  VelocityEstimator estimator_;
  // What the desk itself is doing, observed from the height stream and the
  // keys we send. Independent of state_: the desk also moves on its own
  // (presets, sit/stand, the handset).
  struct Travel {
    float last_height = -1.0f;    // dedupe: each decoded value counts once
    // The last real height change: the dead-reckoning anchor between
    // reports. The first report after boot isn't a change — nothing was
    // known before it — so changed stays false until a second value.
    uint32_t last_change_at = 0;
    bool changed = false;
    uint32_t started_at = 0;      // first change after resting for the window
    // When a key the box acts on (a one-shot other than a wake) last went
    // out — for halt()'s once-per-travel rule.
    uint32_t key_sent_at = 0;
    bool key_sent = false;

    // A decoded height; true when it's a new value.
    bool observe(uint32_t now, float height, uint32_t window) {
      if (height == last_height) return false;
      if (last_height >= 0.0f) {
        if (!moving(now, window)) started_at = now;  // a new travel begins
        last_change_at = now;
        changed = true;
      }
      last_height = height;
      return true;
    }
    // A real height change within |window|.
    bool moving(uint32_t now, uint32_t window) const {
      return changed && now - last_change_at <= window;
    }
    void keySent(uint32_t now) {
      key_sent = true;
      key_sent_at = now;
    }
    bool keySentThisTravel() const { return key_sent && key_sent_at >= started_at; }
  } travel_;

  State state_ = State::kIdle;

  // Per-move timing counters, shared by every state that sends frames (seek
  // drive, correction tap, hold).
  struct Motor {
    uint32_t started_at = 0;    // when the current drive started (for move_timeout)
    uint32_t next_frame_at = 0; // earliest time the next command frame may be sent
    bool height_seen_fresh = false; // arms the staleness abort once the stream is live;
                                    // until then first_report_timeout applies
  } motor_;

  // Absolute-height seek, valid while seeking() (from beginSeek until stop()).
  struct Seek {
    float target = 0.0f;
    bool up = false;  // direction of the current drive or tap
    uint32_t correction_attempts = 0;
    uint32_t tap_until = 0;         // end of the current correction tap (kTapping)
    uint32_t drive_started_at = 0;  // when the initial drive began
    // Where and how fast the initial drive stopped — the coast measurement's
    // starting point. stop_speed_measured is false when the speed was the
    // terminal-speed fallback, which must not be learned from.
    float stop_height = 0.0f;
    float stop_speed = 0.0f;
    bool stop_speed_measured = false;
    // Telemetry breadcrumbs for the SeekReport.
    float start_height = 0.0f;
    float predicted_coast = 0.0f;
    float observed_coast = 0.0f;
  } seek_;

  // Most recently completed seek; report_ready_ makes takeSeekReport one-shot.
  SeekReport report_{};
  bool report_ready_ = false;

  // Post-drive settling, valid in kSettling.
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

  // Deferred moveToHeight, valid in kAwaitingHeight.
  struct Deferred {
    float target = 0.0f;
    uint32_t set_at = 0;
    uint32_t last_wake = 0;
  } deferred_;

  // One-shot command awaiting its post-wake settle delay. only_if_moving
  // (halt()'s cancel) drops it unless the desk is still seen moving then.
  struct OneShot {
    bool active = false;
    desk_cmd::Frame frame{nullptr, 0};
    uint32_t fire_at = 0;
    bool only_if_moving = false;
  } one_shot_;

  // Held command, valid in kHolding.
  struct Hold {
    desk_cmd::Frame frame{nullptr, 0};
    uint32_t until = 0;
  } hold_;
};
