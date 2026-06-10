#include "DeskMotionPlanner.h"

#include <utility>

namespace {
constexpr uint32_t kMaxCorrectionAttempts = 10;
// Movement smaller than this between two readings is display noise, not a
// measurable coast/tap result worth learning from.
constexpr float kMinLearnableMove = 0.05f;
}  // namespace

DeskMotionPlanner::DeskMotionPlanner(float min_height, float max_height, Timing timing,
                                     MotionModel::Tunables tunables, SendFn send)
    : min_height_(min_height),
      max_height_(max_height),
      timing_(timing),
      send_(std::move(send)),
      model_(tunables),
      estimator_({timing.fine_height_limit, timing.speed_window, timing.speed_min_span,
                  timing.speed_max_age}) {}

void DeskMotionPlanner::startMove(Mode mode, uint32_t now) {
  one_shot_.active = false;  // cancel any queued one-shot so it can't fire mid-move
  motor_.mode = mode;
  motor_.started_at = now;
  motor_.height_seen_fresh = false;
  motor_.next_frame_at = now + timing_.wake_settle;
  send_(desk_cmd::Wake);
}

void DeskMotionPlanner::beginSeek(float target_cm, bool up, uint32_t now) {
  estimator_.reset();  // stale samples from a previous move must not leak in
  seek_ = {};
  seek_.active = true;
  seek_.target = target_cm;
  seek_.phase = SeekPhase::kDriving;
  seek_.up = up;
  seek_.drive_started_at = now;
  startMove(up ? Mode::kUp : Mode::kDown, now);
}

void DeskMotionPlanner::beginSettling(float height, uint32_t now) {
  motor_.mode = Mode::kIdle;
  seek_.correction_drive_until = 0;
  seek_.phase = SeekPhase::kSettling;
  settle_.started_at = now;
  settle_.last_height = height;
  settle_.stable_since = now;
}

void DeskMotionPlanner::beginCorrectionDrive(bool up, uint32_t duration,
                                             float start_height, uint32_t now) {
  // Skips wake/settle — the desk is already awake from the initial drive.
  seek_.up = up;
  seek_.phase = SeekPhase::kDriving;
  seek_.correction_drive_until = now + duration;
  motor_.mode = up ? Mode::kUp : Mode::kDown;
  motor_.started_at = now;
  motor_.height_seen_fresh = false;
  motor_.next_frame_at = now;  // first frame fires on the very next tick
  last_tap_.duration = duration;
  last_tap_.start_height = start_height;
}

void DeskMotionPlanner::moveUp(uint32_t now) {
  seek_ = {};
  startMove(Mode::kUp, now);
}

void DeskMotionPlanner::moveDown(uint32_t now) {
  seek_ = {};
  startMove(Mode::kDown, now);
}

void DeskMotionPlanner::stop() {
  motor_.mode = Mode::kIdle;
  seek_ = {};
  deferred_.active = false;
  one_shot_.active = false;  // a pending preset must not fire after a STOP
}

float DeskMotionPlanner::deadbandAt(float height) const {
  // At/above the fine-resolution limit the display reports whole centimetres,
  // so a sub-resolution deadband can never be confirmed there: a target
  // between two readings would tap back and forth between them until the
  // attempt cap. Half the coarse resolution is the tightest deadband every
  // target can satisfy.
  const float fine = timing_.target_deadband;
  const float coarse = timing_.coarse_target_deadband;
  return (height >= timing_.fine_height_limit && coarse > fine) ? coarse : fine;
}

void DeskMotionPlanner::moveToHeight(float target_cm, uint32_t now, bool has_height,
                                     float height) {
  if (target_cm < min_height_) target_cm = min_height_;
  if (target_cm > max_height_) target_cm = max_height_;

  if (!has_height) {
    deferred_ = {true, target_cm, now, now};
    send_(desk_cmd::Wake);
    return;
  }

  const float deadband = deadbandAt(height);
  if (target_cm > height + deadband) {
    beginSeek(target_cm, /*up=*/true, now);
  } else if (target_cm < height - deadband) {
    beginSeek(target_cm, /*up=*/false, now);
  } else {
    stop();
  }
}

void DeskMotionPlanner::issue(const desk_cmd::Frame& frame, uint32_t now) {
  stop();
  send_(desk_cmd::Wake);
  one_shot_ = {true, frame, now + timing_.wake_settle};
}

void DeskMotionPlanner::holdCommand(const desk_cmd::Frame& frame, uint32_t now) {
  hold_ = {frame, now + timing_.hold_duration};
  startMove(Mode::kHold, now);
}

void DeskMotionPlanner::tick(uint32_t now, bool has_height, float height,
                             uint32_t height_age) {
  // Feed each newly decoded value to the speed estimator exactly once.
  if (has_height && height != last_fed_height_) {
    last_fed_height_ = height;
    last_height_change_at_ = now;
    estimator_.addSample(now, height);
  }

  servicePending(now, has_height, height);
  if (motor_.mode == Mode::kHold) {
    serviceHold(now);
  } else if (seek_.phase == SeekPhase::kSettling) {
    serviceSeekSettling(now, has_height, height);
  } else {
    serviceMovement(now, has_height, height, height_age);
  }
}

void DeskMotionPlanner::servicePending(uint32_t now, bool has_height, float height) {
  if (one_shot_.active && static_cast<int32_t>(now - one_shot_.fire_at) >= 0) {
    send_(one_shot_.frame);
    one_shot_.active = false;
  }

  if (deferred_.active) {
    if (has_height) {
      deferred_.active = false;
      moveToHeight(deferred_.target, now, has_height, height);
    } else if (now - deferred_.set_at > timing_.target_timeout) {
      deferred_.active = false;
    } else if (now - deferred_.last_wake >= timing_.wake_retry_interval) {
      deferred_.last_wake = now;
      send_(desk_cmd::Wake);
    }
  }
}

void DeskMotionPlanner::serviceMovement(uint32_t now, bool has_height, float height,
                                        uint32_t height_age) {
  if (motor_.mode == Mode::kIdle) return;

  if (now - motor_.started_at > timing_.move_timeout) { stop(); return; }

  if (has_height) {
    if (height_age <= timing_.height_stale_timeout) {
      motor_.height_seen_fresh = true;
    } else if (motor_.height_seen_fresh) {
      stop(); return;
    }
  }

  // Stop decisions run on every tick; only frame *sending* is gated on the
  // command cadence below. Gating these checks behind next_frame_at would
  // delay each stop by up to a full command interval — biasing seeks toward
  // overshoot and stretching correction taps past their computed duration.
  if (has_height) {
    if (motor_.mode == Mode::kUp && height >= max_height_) { stop(); return; }
    if (motor_.mode == Mode::kDown && height <= min_height_) { stop(); return; }
    if (seek_.active) {
      if (seek_.correction_drive_until != 0) {
        if (static_cast<int32_t>(now - seek_.correction_drive_until) >= 0) {
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
              static_cast<float>(now - last_height_change_at_) / 1000.0f;
          position += (seek_.up ? speed : -speed) * gap_s;
        }
        const float coast = model_.coastDistance(seek_.up, speed);
        const bool reached = motor_.mode == Mode::kUp
                                 ? position + coast >= seek_.target
                                 : position - coast <= seek_.target;
        if (reached) {
          // The coast observation must measure from where the desk actually
          // was at cut-off, so the learned start point is the dead-reckoned
          // position, not the stale report.
          seek_.stop_height = position;
          seek_.stop_speed = speed;
          seek_.stop_speed_measured = measured;
          beginSettling(height, now);
          return;
        }
      }
    }
  }

  if (static_cast<int32_t>(now - motor_.next_frame_at) < 0) return;
  send_(motor_.mode == Mode::kUp ? desk_cmd::Up : desk_cmd::Down);
  motor_.next_frame_at = now + timing_.command_interval;
}

void DeskMotionPlanner::serviceSeekSettling(uint32_t now, bool has_height, float height) {
  if (has_height && height != settle_.last_height) {
    settle_.last_height = height;
    settle_.stable_since = now;
  }

  const bool stable = has_height && (now - settle_.stable_since) >= timing_.stable_duration;
  const bool timed_out = (now - settle_.started_at) >= timing_.seek_settle_delay;
  if (!stable && !timed_out) return;
  if (!has_height) { stop(); return; }

  // Update the model — but only from clean measurements: endpoints inside the
  // display's fine-resolution range, and (for the coast) a stop speed that was
  // actually measured rather than the terminal-speed fallback. Quantized or
  // modeled inputs would feed the learning its own assumptions back.
  const bool settled_fine = height < timing_.fine_height_limit;
  if (seek_.correction_attempts == 0) {
    const bool stop_fine = seek_.stop_height < timing_.fine_height_limit;
    if (seek_.stop_speed_measured && settled_fine && stop_fine) {
      const float coast = seek_.up ? height - seek_.stop_height
                                   : seek_.stop_height - height;
      if (coast > kMinLearnableMove) {
        model_.learnCoast(seek_.up, seek_.stop_speed, coast);
      }
      // A drive long enough to have reached cruise speed also yields a
      // terminal-speed observation.
      if (settle_.started_at - seek_.drive_started_at >= timing_.terminal_speed_drive_min) {
        model_.learnTerminalSpeed(seek_.up, seek_.stop_speed);
      }
    }
  } else {
    const float moved = height > last_tap_.start_height
                            ? height - last_tap_.start_height
                            : last_tap_.start_height - height;
    const bool tap_fine = last_tap_.start_height < timing_.fine_height_limit;
    if (settled_fine && tap_fine && moved > kMinLearnableMove) {
      model_.learnTap(seek_.up, last_tap_.duration, moved);
    }
  }

  const float abs_diff = height > seek_.target ? height - seek_.target
                                                : seek_.target - height;
  if (abs_diff <= deadbandAt(height)) { stop(); return; }
  if (seek_.correction_attempts >= kMaxCorrectionAttempts) { stop(); return; }

  ++seek_.correction_attempts;
  const bool tap_up = height < seek_.target;
  const uint32_t tap_dur =
      model_.tapDurationMs(tap_up, abs_diff, timing_.correction_tap_max);
  beginCorrectionDrive(tap_up, tap_dur, height, now);
}

void DeskMotionPlanner::serviceHold(uint32_t now) {
  if (static_cast<int32_t>(now - hold_.until) >= 0) { stop(); return; }
  if (static_cast<int32_t>(now - motor_.next_frame_at) < 0) return;
  send_(hold_.frame);
  motor_.next_frame_at = now + timing_.command_interval;
}
