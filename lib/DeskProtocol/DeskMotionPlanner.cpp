#include "DeskMotionPlanner.h"

#include <utility>

namespace {
constexpr uint32_t kMaxCorrectionAttempts = 10;
// Movement smaller than this between two readings is display noise, not a
// measurable coast/tap result worth learning from.
constexpr float kMinLearnableMove = 0.05f;
}  // namespace

VelocityEstimator::Config DeskMotionPlanner::estimatorConfig(const Settings& s) {
  VelocityEstimator::Config c;
  c.fine_resolution_max = s.fine_height_limit;
  c.window = s.speed_window;
  c.min_span = s.speed_min_span;
  c.max_sample_age = s.speed_max_age;
  return c;
}

DeskMotionPlanner::DeskMotionPlanner(float min_height, float max_height, Settings settings,
                                     MotionModel::Tunables tunables, SendFn send)
    : min_height_(min_height),
      max_height_(max_height),
      settings_(settings),
      send_(std::move(send)),
      model_(tunables),
      estimator_(estimatorConfig(settings)) {}

void DeskMotionPlanner::startMove(State state, uint32_t now) {
  one_shot_.active = false;  // cancel any queued one-shot so it can't fire mid-move
  state_ = state;
  motor_.started_at = now;
  motor_.height_seen_fresh = false;
  motor_.next_frame_at = now + settings_.wake_settle;
  send_(desk_cmd::Wake);
}

void DeskMotionPlanner::beginSeek(float target_cm, bool up, uint32_t now, float height) {
  estimator_.reset();  // stale samples from a previous move must not leak in
  seek_ = {};
  seek_.target = target_cm;
  seek_.up = up;
  seek_.drive_started_at = now;
  seek_.start_height = height;
  startMove(State::kDriving, now);
}

bool DeskMotionPlanner::takeSeekReport(SeekReport& out) {
  if (!report_ready_) return false;
  report_ready_ = false;
  out = report_;
  return true;
}

void DeskMotionPlanner::beginSettling(float height, uint32_t now) {
  state_ = State::kSettling;
  settle_.started_at = now;
  settle_.last_height = height;
  settle_.stable_since = now;
}

void DeskMotionPlanner::beginCorrectionDrive(bool up, uint32_t duration,
                                             float start_height, uint32_t now) {
  // Skips wake/settle — the desk is already awake from the initial drive.
  state_ = State::kTapping;
  seek_.up = up;
  seek_.tap_until = now + duration;
  motor_.started_at = now;
  motor_.height_seen_fresh = false;
  motor_.next_frame_at = now;  // first frame fires on the very next tick
  last_tap_.duration = duration;
  last_tap_.start_height = start_height;
}

void DeskMotionPlanner::stop() {
  state_ = State::kIdle;
  seek_ = {};
  one_shot_.active = false;  // a pending preset (or cancel) must not fire after a STOP
}

void DeskMotionPlanner::halt(uint32_t now) {
  // Our own drive halts once its frames stop; it needs no cancel.
  const bool driving = seeking() || state_ == State::kHolding;
  stop();
  if (driving || !travel_.moving(now, settings_.motion_window)) return;
  if (travel_.keySentThisTravel()) return;
  send_(desk_cmd::Wake);
  one_shot_ = {true, desk_cmd::Cancel, now + settings_.wake_settle, /*only_if_moving=*/true};
}

float DeskMotionPlanner::deadbandAt(float height) const {
  // At/above the fine-resolution limit the display reports whole centimetres,
  // so a sub-resolution deadband can never be confirmed there: a target
  // between two readings would tap back and forth between them until the
  // attempt cap. Half the coarse resolution is the tightest deadband every
  // target can satisfy.
  const float fine = settings_.target_deadband;
  const float coarse = settings_.coarse_target_deadband;
  return (height >= settings_.fine_height_limit && coarse > fine) ? coarse : fine;
}

void DeskMotionPlanner::moveToHeight(float target_cm, uint32_t now, bool has_height,
                                     float height) {
  if (target_cm < min_height_) target_cm = min_height_;
  if (target_cm > max_height_) target_cm = max_height_;

  if (!has_height) {
    one_shot_.active = false;  // replaces a queued preset, like any new command
    state_ = State::kAwaitingHeight;
    deferred_ = {target_cm, now, now};
    send_(desk_cmd::Wake);
    return;
  }

  const float deadband = deadbandAt(height);
  if (target_cm > height + deadband) {
    beginSeek(target_cm, /*up=*/true, now, height);
  } else if (target_cm < height - deadband) {
    beginSeek(target_cm, /*up=*/false, now, height);
  } else {
    stop();
  }
}

void DeskMotionPlanner::issue(const desk_cmd::Frame& frame, uint32_t now) {
  stop();
  send_(desk_cmd::Wake);
  one_shot_ = {true, frame, now + settings_.wake_settle, /*only_if_moving=*/false};
}

void DeskMotionPlanner::holdCommand(const desk_cmd::Frame& frame, uint32_t now,
                                    uint32_t duration) {
  seek_ = {};  // a hold replaces any seek, like any other new command
  hold_ = {frame, now + duration};
  startMove(State::kHolding, now);
}

void DeskMotionPlanner::tick(uint32_t now, bool has_height, float height,
                             uint32_t height_age) {
  // Feed each newly decoded value to the speed estimator exactly once.
  if (has_height && travel_.observe(now, height, settings_.motion_window)) {
    estimator_.addSample(now, height);
  }

  servicePending(now, has_height, height);
  switch (state_) {
    case State::kHolding:
      serviceHold(now);
      break;
    case State::kSettling:
      serviceSeekSettling(now, has_height, height);
      break;
    case State::kDriving:
    case State::kTapping:
      serviceMovement(now, has_height, height, height_age);
      break;
    case State::kIdle:
    case State::kAwaitingHeight:
      break;
  }
}

void DeskMotionPlanner::servicePending(uint32_t now, bool has_height, float height) {
  if (one_shot_.active && static_cast<int32_t>(now - one_shot_.fire_at) >= 0) {
    one_shot_.active = false;
    // A cancel is dropped if the travel ended by itself during the settle:
    // the key would then start a preset move instead.
    if (!one_shot_.only_if_moving || travel_.moving(now, settings_.motion_window)) {
      send_(one_shot_.frame);
      // Every one-shot but a plain wake is a key the box acts on — one that
      // cancels any travel in progress (see halt()).
      if (one_shot_.frame.data != desk_cmd::Wake.data) travel_.keySent(now);
    }
  }

  if (state_ == State::kAwaitingHeight) {
    if (has_height) {
      state_ = State::kIdle;
      moveToHeight(deferred_.target, now, has_height, height);
    } else if (now - deferred_.set_at > settings_.target_timeout) {
      state_ = State::kIdle;
    } else if (now - deferred_.last_wake >= settings_.wake_retry_interval) {
      deferred_.last_wake = now;
      send_(desk_cmd::Wake);
    }
  }
}

void DeskMotionPlanner::serviceMovement(uint32_t now, bool has_height, float height,
                                        uint32_t height_age) {
  if (now - motor_.started_at > settings_.move_timeout) { stop(); return; }

  if (has_height && height_age <= settings_.height_stale_timeout) {
    motor_.height_seen_fresh = true;
  } else if (motor_.height_seen_fresh) {
    stop(); return;
  } else if (now - motor_.started_at > settings_.first_report_timeout) {
    // The stream never came alive for this drive: the RX line is dead, or the
    // display shows something other than the height (e.g. "LOC" while child-
    // locked, when the desk ignores the drive anyway). Without a live height
    // the limit and target checks are blind, so don't keep driving until
    // move_timeout.
    stop(); return;
  }

  // Stop decisions run on every tick; only frame *sending* is gated on the
  // command cadence below. Gating these checks behind next_frame_at would
  // delay each stop by up to a full command interval — biasing seeks toward
  // overshoot and stretching correction taps past their computed duration.
  if (has_height) {
    if (seek_.up && height >= max_height_) { stop(); return; }
    if (!seek_.up && height <= min_height_) { stop(); return; }
    if (state_ == State::kTapping) {
      if (static_cast<int32_t>(now - seek_.tap_until) >= 0) {
        beginSettling(height, now); return;
      }
    } else {
      // Stop driving once the remaining distance is within the predicted
      // coast. Speed comes from the live estimate when one is available;
      // otherwise assume terminal speed — an overestimate during ramp-up,
      // which errs toward stopping early (undershoot) so the correction
      // tap drives forward rather than reversing.
      float speed = model_.terminalSpeed(seek_.up);
      bool measured = false;
      if (estimator_.valid(now)) {
        speed = estimator_.speed();
        if (speed < 0.0f) speed = -speed;
        measured = true;
      }
      // Dead-reckon between reports: the display only reports every
      // ~108 ms, so waiting for the report that confirms the stop point
      // stops up to a full report late. With a measured speed, advance the
      // last report by speed·(time since it changed) and stop at the
      // predicted crossing instead; estimator validity bounds the gap to
      // speed_max_age. The fallback speed must not extrapolate — an
      // assumed speed across a stale gap could predict past the true
      // position, and fallback errors have to stay on the undershoot side.
      float position = height;
      if (measured) {
        const float gap_s =
            static_cast<float>(now - travel_.last_change_at) / 1000.0f;
        position += (seek_.up ? speed : -speed) * gap_s;
      }
      const float coast = model_.coastDistance(seek_.up, speed);
      const bool reached = seek_.up ? position + coast >= seek_.target
                                    : position - coast <= seek_.target;
      if (reached) {
        // The coast observation must measure from where the desk actually
        // was at cut-off, so the learned start point is the dead-reckoned
        // position, not the stale report.
        seek_.stop_height = position;
        seek_.stop_speed = speed;
        seek_.stop_speed_measured = measured;
        seek_.predicted_coast = coast;
        beginSettling(height, now);
        return;
      }
    }
  }

  if (static_cast<int32_t>(now - motor_.next_frame_at) < 0) return;
  send_(seek_.up ? desk_cmd::Up : desk_cmd::Down);
  motor_.next_frame_at = now + settings_.command_interval;
}

void DeskMotionPlanner::serviceSeekSettling(uint32_t now, bool has_height, float height) {
  if (has_height && height != settle_.last_height) {
    settle_.last_height = height;
    settle_.stable_since = now;
  }

  const bool stable = has_height && (now - settle_.stable_since) >= settings_.stable_duration;
  const bool timed_out = (now - settle_.started_at) >= settings_.seek_settle_delay;
  if (!stable && !timed_out) return;
  if (!has_height) { stop(); return; }

  // Update the model — but only from clean measurements: endpoints inside the
  // display's fine-resolution range, and (for the coast) a stop speed that was
  // actually measured rather than the terminal-speed fallback. Quantized or
  // modeled inputs would feed the learning its own assumptions back.
  const bool settled_fine = height < settings_.fine_height_limit;
  if (seek_.correction_attempts == 0) {
    const float coast = seek_.up ? height - seek_.stop_height
                                 : seek_.stop_height - height;
    seek_.observed_coast = coast;  // telemetry, regardless of learning gates
    const bool stop_fine = seek_.stop_height < settings_.fine_height_limit;
    if (seek_.stop_speed_measured && settled_fine && stop_fine) {
      if (coast > kMinLearnableMove) {
        model_.learnCoast(seek_.up, seek_.stop_speed, coast);
      }
      // A drive long enough to have reached cruise speed also yields a
      // terminal-speed observation.
      if (settle_.started_at - seek_.drive_started_at >= settings_.terminal_speed_drive_min) {
        model_.learnTerminalSpeed(seek_.up, seek_.stop_speed);
      }
    }
  } else {
    const float moved = height > last_tap_.start_height
                            ? height - last_tap_.start_height
                            : last_tap_.start_height - height;
    const bool tap_fine = last_tap_.start_height < settings_.fine_height_limit;
    if (settled_fine && tap_fine && moved > kMinLearnableMove) {
      model_.learnTap(seek_.up, last_tap_.duration, moved);
    }
  }

  const float abs_diff = height > seek_.target ? height - seek_.target
                                                : seek_.target - height;
  if (abs_diff <= deadbandAt(height) ||
      seek_.correction_attempts >= kMaxCorrectionAttempts) {
    report_ = {seek_.target,         seek_.start_height,
               seek_.stop_height,    seek_.stop_speed,
               seek_.stop_speed_measured, seek_.predicted_coast,
               seek_.observed_coast, height,
               seek_.correction_attempts, now - seek_.drive_started_at};
    report_ready_ = true;
    stop();
    return;
  }

  ++seek_.correction_attempts;
  const bool tap_up = height < seek_.target;
  const uint32_t tap_dur =
      model_.tapDurationMs(tap_up, abs_diff, settings_.correction_tap_max);
  beginCorrectionDrive(tap_up, tap_dur, height, now);
}

void DeskMotionPlanner::serviceHold(uint32_t now) {
  if (static_cast<int32_t>(now - hold_.until) >= 0) { stop(); return; }
  if (static_cast<int32_t>(now - motor_.next_frame_at) < 0) return;
  send_(hold_.frame);
  motor_.next_frame_at = now + settings_.command_interval;
}
