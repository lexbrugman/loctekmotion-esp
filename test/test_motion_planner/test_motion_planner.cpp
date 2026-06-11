// Host-side unit tests for DeskMotionPlanner. Run with: pio test -e native
#include <unity.h>

#include <cstdint>
#include <functional>
#include <vector>

#include "DeskMotionPlanner.h"

namespace {

// Fixed timing for every test, independent of cfg:: so the numbers stay easy
// to reason about and don't drift if the production constants change.
constexpr DeskMotionPlanner::Timing kTiming{
    /*wake_settle=*/200,
    /*command_interval=*/100,
    /*move_timeout=*/1000,
    /*height_stale_timeout=*/300,
    /*target_timeout=*/500,
    /*wake_retry_interval=*/150,
    /*target_deadband=*/0.5f,
    /*coarse_target_deadband=*/1.0f,
    /*seek_settle_delay=*/200,
    /*stable_duration=*/0,  // 0 = stable on the first tick with a height
    /*correction_tap_max=*/300,
    /*terminal_speed_drive_min=*/2000,
    /*fine_height_limit=*/100.0f,
    /*speed_window=*/600,
    /*speed_min_span=*/250,
    /*speed_max_age=*/400,
};

// Friendly model numbers: fallback coast = 4²/(2·2) = 4.0 cm; a measured
// 3 cm/s gives 9/(2·2) = 2.25 cm; a 1 cm error taps for √(1/25)s = 200 ms.
// Learning rates default to 0 so tests are deterministic unless a test
// switches a specific rate on.
constexpr MotionModel::Tunables kTunables{
    /*seed_terminal_speed=*/4.0f,
    /*seed_decel=*/2.0f,
    /*seed_tap_gain=*/25.0f,
    /*speed_learning_rate=*/0.0f,
    /*decel_learning_rate=*/0.0f,
    /*tap_learning_rate=*/0.0f,
    /*min_terminal_speed=*/0.5f,
    /*max_terminal_speed=*/10.0f,
    /*min_decel=*/0.2f,
    /*max_decel=*/20.0f,
    /*min_tap_gain=*/5.0f,
    /*max_tap_gain=*/100.0f,
};

constexpr MotionModel::Tunables withRates(MotionModel::Tunables t, float speed_rate,
                                          float decel_rate, float tap_rate) {
  t.speed_learning_rate = speed_rate;
  t.decel_learning_rate = decel_rate;
  t.tap_learning_rate = tap_rate;
  return t;
}

// Seek tests drive for longer than kTiming's 1 s safety timeout allows.
constexpr DeskMotionPlanner::Timing withSeekTimeout(DeskMotionPlanner::Timing t) {
  t.move_timeout = 20000;
  return t;
}
constexpr DeskMotionPlanner::Timing kSeekTiming = withSeekTimeout(kTiming);

// kSeekTiming variant: non-zero stable_duration to test stability detection.
constexpr DeskMotionPlanner::Timing withStabilityDetection(DeskMotionPlanner::Timing t) {
  t.seek_settle_delay = 600;  // generous timeout; stability should fire first
  t.stable_duration = 100;
  return t;
}
constexpr DeskMotionPlanner::Timing kStabilityTiming =
    withStabilityDetection(kSeekTiming);

constexpr float kMin = 60.0f;
constexpr float kMax = 120.0f;

// "Fresh" height age for tests that aren't exercising staleness — well under
// kTiming.height_stale_timeout.
constexpr uint32_t kFresh = 0;

// Records every frame the planner asks to send, identified by its data
// pointer — each desk_cmd:: frame wraps a distinct constexpr byte array, so
// pointer identity is enough to tell them apart.
struct Recorder {
  std::vector<const uint8_t*> sent;
  void operator()(const desk_cmd::Frame& f) { sent.push_back(f.data); }
};

bool sentFrame(const Recorder& r, size_t i, const desk_cmd::Frame& expected) {
  return i < r.sent.size() && r.sent[i] == expected.data;
}

size_t countFrames(const Recorder& r, const desk_cmd::Frame& expected) {
  size_t n = 0;
  for (auto* p : r.sent) {
    if (p == expected.data) ++n;
  }
  return n;
}

}  // namespace

void test_continuous_move_wakes_then_drives_at_cadence() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveUp(1000);
  TEST_ASSERT_EQUAL(1, rec.sent.size());
  TEST_ASSERT_TRUE(sentFrame(rec, 0, desk_cmd::Wake));
  TEST_ASSERT_TRUE(p.moving());

  p.tick(1100, false, 0.0f, kFresh);  // still settling — no movement frame yet
  TEST_ASSERT_EQUAL(1, rec.sent.size());

  p.tick(1200, false, 0.0f, kFresh);  // settled — first Up frame goes out
  TEST_ASSERT_EQUAL(2, rec.sent.size());
  TEST_ASSERT_TRUE(sentFrame(rec, 1, desk_cmd::Up));

  p.tick(1250, false, 0.0f, kFresh);  // too soon for the next cadence tick
  TEST_ASSERT_EQUAL(2, rec.sent.size());

  p.tick(1300, false, 0.0f, kFresh);  // next cadence tick
  TEST_ASSERT_EQUAL(3, rec.sent.size());
  TEST_ASSERT_TRUE(sentFrame(rec, 2, desk_cmd::Up));
}

void test_stop_halts_continuous_movement() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveDown(1000);
  p.tick(1200, false, 0.0f, kFresh);
  TEST_ASSERT_TRUE(p.moving());

  p.stop();
  TEST_ASSERT_FALSE(p.moving());

  const size_t before = rec.sent.size();
  p.tick(1300, false, 0.0f, kFresh);
  TEST_ASSERT_EQUAL(before, rec.sent.size());  // no further frames once stopped
}

void test_continuous_movement_stops_at_travel_limits() {
  Recorder up_rec;
  DeskMotionPlanner up(kMin, kMax, kTiming, kTunables, std::ref(up_rec));
  up.moveUp(1000);
  up.tick(1200, /*has_height=*/true, kMax, kFresh);  // already at the top of travel
  TEST_ASSERT_FALSE(up.moving());

  Recorder down_rec;
  DeskMotionPlanner down(kMin, kMax, kTiming, kTunables, std::ref(down_rec));
  down.moveDown(1000);
  down.tick(1200, /*has_height=*/true, kMin, kFresh);  // already at the bottom
  TEST_ASSERT_FALSE(down.moving());
}

void test_runaway_movement_stops_after_travel_timeout() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveUp(1000);
  p.tick(1000 + kTiming.move_timeout + 1, /*has_height=*/false, 0.0f, kFresh);
  TEST_ASSERT_FALSE(p.moving());  // the safety net cut the move short
}

void test_continuous_movement_survives_a_stale_report_at_wakeup() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  // The display was asleep when the move was requested: the only height
  // report on record is long since stale (older than height_stale_timeout),
  // simply because nothing has streamed since. That must not be mistaken for
  // a mid-move stall — the desk needs a moment to wake and resume streaming.
  p.moveUp(1000);
  p.tick(1200, /*has_height=*/true, /*height=*/80.0f,
         /*height_age=*/kTiming.height_stale_timeout + 50);
  TEST_ASSERT_TRUE(p.moving());
  TEST_ASSERT_EQUAL(2, rec.sent.size());  // wake, then the first Up frame
  TEST_ASSERT_TRUE(sentFrame(rec, 1, desk_cmd::Up));

  // Once a fresh report confirms the stream is live, staleness is tracked
  // exactly as before — see test_continuous_movement_aborts_when_height_...
  p.tick(1300, /*has_height=*/true, /*height=*/80.5f, /*height_age=*/0);
  TEST_ASSERT_TRUE(p.moving());
}

void test_continuous_movement_aborts_when_height_reports_go_stale() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveUp(1000);
  p.tick(1200, /*has_height=*/true, /*height=*/80.0f, /*height_age=*/0);
  TEST_ASSERT_TRUE(p.moving());  // under way, reports still fresh

  // Reports stop arriving: height is frozen and its age keeps growing. The
  // limit checks would go blind on a frozen value, so the planner should bail
  // out via the staleness net long before move_timeout (1000 ms) elapses.
  p.tick(1300, /*has_height=*/true, /*height=*/80.0f,
         /*height_age=*/kTiming.height_stale_timeout + 50);
  TEST_ASSERT_FALSE(p.moving());
}

void test_move_to_height_seeks_toward_target_and_stops_on_arrival() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveToHeight(/*target_cm=*/99.0f, /*now=*/1000, /*has_height=*/true,
                 /*height=*/80.0f);
  TEST_ASSERT_TRUE(p.moving());  // target is above current height -> seeking up

  p.tick(1200, /*has_height=*/true, /*height=*/99.0f, kFresh);  // reached the target
  TEST_ASSERT_FALSE(p.moving());
}

void test_move_to_height_within_deadband_does_not_move() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, /*has_height=*/true, /*height=*/98.8f);  // within 0.5 cm
  TEST_ASSERT_FALSE(p.moving());
  TEST_ASSERT_TRUE(rec.sent.empty());  // already there — nothing sent at all
}

void test_move_to_height_defers_until_a_height_is_known() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, /*has_height=*/false, 0.0f);
  TEST_ASSERT_FALSE(p.moving());                        // nothing to seek from yet
  TEST_ASSERT_TRUE(sentFrame(rec, 0, desk_cmd::Wake));  // woke to elicit a report

  p.tick(1100, /*has_height=*/true, /*height=*/80.0f, kFresh);  // a report arrives
  TEST_ASSERT_TRUE(p.moving());                         // deferred target now resumed
}

void test_move_to_height_rewakes_periodically_while_awaiting_report() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, /*has_height=*/false, 0.0f);
  TEST_ASSERT_EQUAL(1, countFrames(rec, desk_cmd::Wake));  // initial wake

  p.tick(1100, false, 0.0f, kFresh);  // under the 150 ms retry cadence — not yet
  TEST_ASSERT_EQUAL(1, countFrames(rec, desk_cmd::Wake));

  p.tick(1160, false, 0.0f, kFresh);  // 160 ms since the first wake — nudge it again
  TEST_ASSERT_EQUAL(2, countFrames(rec, desk_cmd::Wake));

  p.tick(1300, false, 0.0f, kFresh);  // only 140 ms since the last nudge — not yet
  TEST_ASSERT_EQUAL(2, countFrames(rec, desk_cmd::Wake));

  p.tick(1320, false, 0.0f, kFresh);  // 160 ms since the last nudge — retry again
  TEST_ASSERT_EQUAL(3, countFrames(rec, desk_cmd::Wake));

  // Still well inside target_timeout (500 ms) — a late report still resumes.
  p.tick(1400, /*has_height=*/true, /*height=*/80.0f, kFresh);
  TEST_ASSERT_TRUE(p.moving());
}

void test_move_to_height_gives_up_if_no_height_report_arrives() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, false, 0.0f);
  p.tick(1000 + kTiming.target_timeout + 1, false, 0.0f, kFresh);
  TEST_ASSERT_FALSE(p.moving());  // gave up waiting

  // A late height report doesn't resurrect the abandoned target.
  p.tick(1000 + kTiming.target_timeout + 100, true, 80.0f, kFresh);
  TEST_ASSERT_FALSE(p.moving());
}

void test_one_shot_command_wakes_then_sends_after_settle() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.issue(desk_cmd::Preset1, 1000);
  TEST_ASSERT_TRUE(sentFrame(rec, 0, desk_cmd::Wake));

  p.tick(1100, false, 0.0f, kFresh);  // still settling
  TEST_ASSERT_EQUAL(1, rec.sent.size());

  p.tick(1200, false, 0.0f, kFresh);  // settled — the command itself goes out
  TEST_ASSERT_EQUAL(2, rec.sent.size());
  TEST_ASSERT_TRUE(sentFrame(rec, 1, desk_cmd::Preset1));
}

void test_one_shot_command_cancels_an_in_progress_move() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.moveUp(1000);
  p.tick(1200, false, 0.0f, kFresh);
  TEST_ASSERT_TRUE(p.moving());

  p.issue(desk_cmd::Sit, 1300);
  TEST_ASSERT_FALSE(p.moving());  // presets move the desk autonomously
}

void test_stop_cancels_a_pending_one_shot() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  p.issue(desk_cmd::Preset1, 1000);
  p.stop();  // STOP arrives within the wake-settle window

  p.tick(1300, false, 0.0f, kFresh);  // past fire_at — must stay silent
  TEST_ASSERT_EQUAL(0, countFrames(rec, desk_cmd::Preset1));
}

void test_held_command_repeats_at_cadence_then_stops() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kTiming, kTunables, std::ref(rec));

  // Mirrors holding a handset button: wake, settle, then resend the frame at
  // the movement cadence for the requested duration before releasing.
  p.holdCommand(desk_cmd::ChildLock, 1000, /*duration=*/320);
  TEST_ASSERT_TRUE(sentFrame(rec, 0, desk_cmd::Wake));
  TEST_ASSERT_FALSE(p.moving());  // a held command isn't "moving" the desk

  p.tick(1100, false, 0.0f, kFresh);  // still settling
  TEST_ASSERT_EQUAL(1, rec.sent.size());

  p.tick(1200, false, 0.0f, kFresh);  // settled — first hold frame goes out
  TEST_ASSERT_EQUAL(2, rec.sent.size());
  TEST_ASSERT_TRUE(sentFrame(rec, 1, desk_cmd::ChildLock));

  p.tick(1300, false, 0.0f, kFresh);  // next cadence tick — repeats
  TEST_ASSERT_EQUAL(3, rec.sent.size());
  TEST_ASSERT_TRUE(sentFrame(rec, 2, desk_cmd::ChildLock));

  p.tick(1400, false, 0.0f, kFresh);  // hold_duration elapsed — releases
  TEST_ASSERT_EQUAL(3, rec.sent.size());
  TEST_ASSERT_EQUAL(2, countFrames(rec, desk_cmd::ChildLock));
}

// ---------------------------------------------------------------------------
// Physics-based seek tests. Model numbers (see kTunables): fallback coast at
// terminal speed 4 cm/s with decel 2 cm/s² = 4.0 cm; a measured 3 cm/s gives
// 2.25 cm; a 1 cm error taps √(1/25) s = 200 ms (gain 25).
// ---------------------------------------------------------------------------

void test_seek_stops_early_by_fallback_coast_without_speed_estimate() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, true, 90.0f);

  // Two samples 100 ms apart are under the estimator's 250 ms min span, so
  // the coast prediction falls back to terminal speed: 4 cm.
  p.tick(1200, true, 90.0f, kFresh);  // 90 + 4 = 94 < 99 -> keep driving
  TEST_ASSERT_TRUE(p.moving());

  p.tick(1300, true, 95.1f, kFresh);  // 95.1 + 4 >= 99 -> stop and settle
  TEST_ASSERT_FALSE(p.moving());
  TEST_ASSERT_TRUE(p.seeking());

  // Settles within the deadband (|98.8 - 99| <= 0.5) — seek complete.
  p.tick(1500, true, 98.8f, kFresh);
  TEST_ASSERT_FALSE(p.seeking());
}

void test_seek_uses_measured_speed_to_shrink_the_coast_prediction() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  // Heights rise 0.3 cm per 100 ms tick = a measured 3 cm/s, predicting a
  // 2.25 cm coast — tighter than the 4 cm terminal-speed fallback.
  p.moveToHeight(99.0f, 1000, true, 94.0f);
  uint32_t t = 1100;
  float h = 94.3f;
  // The fallback would stop at 95+ (95 + 4 >= 99); with the measured speed
  // the drive must continue until 96.75 (h + 2.25 >= 99).
  while (t <= 1900) {
    p.tick(t, true, h, kFresh);
    TEST_ASSERT_TRUE(p.moving());  // still short of the measured-coast stop
    t += 100;
    h += 0.3f;
  }

  p.tick(2000, true, 97.0f, kFresh);  // 97.0 + 2.25 >= 99 -> stop
  TEST_ASSERT_FALSE(p.moving());
  TEST_ASSERT_TRUE(p.seeking());  // settling now
}

void test_seek_sizes_correction_taps_quadratically() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, true, 90.0f);
  p.tick(1300, true, 95.1f, kFresh);  // fallback stop (95.1 + 4 >= 99)

  // Settle reads 97.0: error 2.0 cm -> tap = √(2/25) s = 282 ms (not the
  // 400 ms a linear 200 ms/cm rule would give).
  p.tick(1500, true, 97.0f, kFresh);
  TEST_ASSERT_TRUE(p.moving());  // correction tap drive running
  TEST_ASSERT_TRUE(p.seeking());

  p.tick(1700, true, 97.5f, kFresh);  // 1700 < 1500+282 -> tap still driving
  TEST_ASSERT_TRUE(p.moving());

  p.tick(1800, true, 97.9f, kFresh);  // past 1782 -> tap over, settling again
  TEST_ASSERT_FALSE(p.moving());

  p.tick(2000, true, 98.7f, kFresh);  // |98.7 - 99| <= 0.5 -> done
  TEST_ASSERT_FALSE(p.seeking());
}

void test_seek_learns_decel_from_the_observed_coast() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming,
                      withRates(kTunables, 0.0f, /*decel_rate=*/1.0f, 0.0f),
                      std::ref(rec));

  // Same drive profile as the measured-speed test: stops at 97.0 doing 3 cm/s.
  p.moveToHeight(99.0f, 1000, true, 94.0f);
  uint32_t t = 1100;
  float h = 94.3f;
  while (t <= 1900) {
    p.tick(t, true, h, kFresh);
    t += 100;
    h += 0.3f;
  }
  p.tick(2000, true, 97.0f, kFresh);
  TEST_ASSERT_FALSE(p.moving());

  // Observed coast 97.0 -> 98.5 = 1.5 cm at 3 cm/s: decel = 9/(2·1.5) = 3.0.
  // With a learning rate of 1.0 the new value replaces the 2.0 seed outright.
  p.tick(2200, true, 98.5f, kFresh);
  TEST_ASSERT_FALSE(p.seeking());  // 0.5 error is within the deadband
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, p.learnedState().decel_up);
  // The 1 s drive is under terminal_speed_drive_min — terminal speed untouched.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, p.learnedState().terminal_speed_up);
}

void test_seek_learns_terminal_speed_after_a_long_drive() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming,
                      withRates(kTunables, /*speed_rate=*/1.0f, 0.0f, 0.0f),
                      std::ref(rec));

  // A long climb at a steady measured 3 cm/s: the drive runs ~5.6 s, well
  // past terminal_speed_drive_min (2 s), so the stop speed counts as a
  // terminal-speed observation.
  p.moveToHeight(99.0f, 1000, true, 80.0f);
  uint32_t t = 1100;
  float h = 80.3f;
  while (p.moving() && t < 10000) {
    p.tick(t, true, h, kFresh);
    t += 100;
    h += 0.3f;
  }
  TEST_ASSERT_FALSE(p.moving());  // stopped by the predicted coast, not timeout
  TEST_ASSERT_TRUE(p.seeking());

  p.tick(t + 200, true, 98.9f, kFresh);  // settle within deadband
  TEST_ASSERT_FALSE(p.seeking());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, p.learnedState().terminal_speed_up);
}

void test_seek_learning_is_gated_above_the_fine_resolution_limit() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming,
                      withRates(kTunables, 1.0f, 1.0f, 1.0f), std::ref(rec));

  // Everything above 100 cm reports whole centimetres only: the estimator
  // refuses those samples (fallback speed applies) and the settle-phase
  // learning must skip its update entirely.
  p.moveToHeight(110.0f, 1000, true, 95.0f);
  p.tick(1200, true, 95.0f, kFresh);
  p.tick(1300, true, 99.5f, kFresh);
  p.tick(1400, true, 101.0f, kFresh);  // coarse sample resets the estimator
  p.tick(1500, true, 103.0f, kFresh);
  p.tick(1600, true, 106.5f, kFresh);  // 106.5 + 4 >= 110 -> settle
  TEST_ASSERT_FALSE(p.moving());

  p.tick(1800, true, 108.0f, kFresh);  // settle fires; error 2.0 -> tap
  TEST_ASSERT_TRUE(p.moving());
  // No decel/terminal learning happened: the stop speed was the fallback and
  // the endpoints sit in coarse territory.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, p.learnedState().decel_up);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, p.learnedState().terminal_speed_up);

  p.tick(2100, true, 108.5f, kFresh);  // tap over (282 ms) -> settling
  TEST_ASSERT_FALSE(p.moving());
  p.tick(2300, true, 109.0f, kFresh);  // tap moved 1.0 cm — but above 100 cm
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, p.learnedState().tap_gain_up);
}

void test_seek_stop_decision_runs_between_command_frames() {
  // The reached check must fire on the tick a satisfying height report
  // arrives, not be held until the next command-frame slot — that delay is a
  // late stop of up to a full command interval, i.e. a built-in overshoot.
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, true, 90.0f);
  p.tick(1200, true, 94.0f, kFresh);  // frame slot: 94 + 4 < 99 -> keep driving
  TEST_ASSERT_TRUE(p.moving());       // next frame not due until 1300

  p.tick(1250, true, 95.5f, kFresh);  // mid-interval report: 95.5 + 4 >= 99
  TEST_ASSERT_FALSE(p.moving());      // stops now, not at the 1300 frame slot
  TEST_ASSERT_TRUE(p.seeking());      // settling
}

void test_correction_tap_ends_at_its_computed_duration() {
  // A tap's end must be honoured to tick resolution, not rounded up to the
  // next command-frame slot — otherwise actual tap durations quantize to the
  // frame cadence and the learned quadratic gain is fitted to noise.
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, true, 90.0f);
  p.tick(1300, true, 95.1f, kFresh);  // fallback stop (95.1 + 4 >= 99)
  p.tick(1500, true, 97.0f, kFresh);  // settle: error 2.0 -> 282 ms tap (ends 1782)
  TEST_ASSERT_TRUE(p.moving());

  p.tick(1700, true, 97.5f, kFresh);  // tap frame sent; next slot would be 1800
  TEST_ASSERT_TRUE(p.moving());

  p.tick(1785, true, 97.8f, kFresh);  // past 1782 -> tap ends now, not at 1800
  TEST_ASSERT_FALSE(p.moving());
  TEST_ASSERT_TRUE(p.seeking());  // settling after the tap

  p.tick(2000, true, 98.7f, kFresh);  // |98.7 - 99| <= 0.5 -> done
  TEST_ASSERT_FALSE(p.seeking());
}

void test_seek_dead_reckons_between_height_reports() {
  // The display only reports every ~100 ms; the stop must fire at the
  // *predicted* crossing between reports, not wait for the report that
  // confirms it. Learning then measures the coast from the dead-reckoned
  // cut-off position, not the stale report.
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming,
                      withRates(kTunables, 0.0f, /*decel_rate=*/1.0f, 0.0f),
                      std::ref(rec));

  // Steady measured 3 cm/s climb; reports every 100 ms. The drive starts low
  // enough that the early fallback-coast phase (estimator not yet valid)
  // can't trip the stop.
  p.moveToHeight(98.6f, 1000, true, 90.0f);
  uint32_t t = 1100;
  float h = 90.3f;
  float last_h = h;
  while (t <= 3000) {
    p.tick(t, true, h, kFresh);
    last_h = h;
    t += 100;
    h += 0.3f;
  }
  // Last report 96.0 at 3000: 96.0 + 2.25 < 98.6 -> still driving.
  TEST_ASSERT_TRUE(p.moving());

  // No new report, but 150 ms later dead reckoning puts the desk at
  // 96.0 + 3·0.15 = 96.45: 96.45 + 2.25 >= 98.6 -> stop mid-interval.
  p.tick(3150, true, last_h, kFresh);
  TEST_ASSERT_FALSE(p.moving());
  TEST_ASSERT_TRUE(p.seeking());

  // Settles at 98.2: coast measured from the dead-reckoned 96.45, not the
  // 96.0 report = 1.75 cm at 3 cm/s -> decel 9/(2·1.75) = 2.57 (rate 1.0
  // replaces the 2.0 seed outright). |98.2 - 98.6| <= 0.5 -> done.
  p.tick(3350, true, 98.2f, kFresh);
  TEST_ASSERT_FALSE(p.seeking());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.57f, p.learnedState().decel_up);
}

void test_coarse_zone_widens_the_deadband() {
  // At/above fine_height_limit the display reports whole centimetres, so the
  // fine deadband (0.5 here) can be unsatisfiable: a target of 110.9 reads as
  // 110 (diff 0.9) or 111 (diff 0.1... but after a tap lands at 110.6 it reads
  // 111 only past 110.5) and the planner would ping-pong between the two
  // readings. The coarse deadband (1.0 here) accepts the nearest reading.
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  // Already as close as a whole-cm reading can get: no movement at all.
  p.moveToHeight(110.9f, 1000, true, 110.0f);
  TEST_ASSERT_FALSE(p.moving());
  TEST_ASSERT_TRUE(rec.sent.empty());

  // A seek settling in the coarse zone accepts within the coarse deadband
  // instead of starting a correction tap.
  p.moveToHeight(110.9f, 2000, true, 105.0f);
  TEST_ASSERT_TRUE(p.moving());
  p.tick(2200, true, 107.0f, kFresh);  // 107 + 4 >= 110.9 -> settling
  TEST_ASSERT_FALSE(p.moving());
  p.tick(2400, true, 110.0f, kFresh);  // |110 - 110.9| = 0.9 <= 1.0 -> done
  TEST_ASSERT_FALSE(p.seeking());
}

void test_seek_waits_for_height_stability_before_sampling() {
  // Verifies that the settle phase waits until height stops changing for
  // stable_duration ms before firing, rather than sampling a mid-coast reading.
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kStabilityTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, true, 80.0f);
  p.tick(1300, true, 95.1f, kFresh);  // fallback stop -> settling

  // Height is still coasting — keeps changing, so stability resets each tick.
  p.tick(1400, true, 96.5f, kFresh);
  TEST_ASSERT_FALSE(p.moving());  // settling, not tapping

  p.tick(1500, true, 97.2f, kFresh);
  TEST_ASSERT_FALSE(p.moving());  // still coasting

  // Height stable for 100 ms: settle fires, 1.8 cm outside deadband -> tap.
  p.tick(1600, true, 97.2f, kFresh);
  TEST_ASSERT_TRUE(p.moving());  // correction tap now running
}

void test_completed_seek_yields_a_report_once() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  DeskMotionPlanner::SeekReport r{};
  TEST_ASSERT_FALSE(p.takeSeekReport(r));  // nothing completed yet

  p.moveToHeight(99.0f, 1000, true, 90.0f);
  p.tick(1300, true, 95.1f, kFresh);       // fallback stop: predicted coast 4.0
  TEST_ASSERT_FALSE(p.takeSeekReport(r));  // still settling — not complete
  p.tick(1500, true, 97.0f, kFresh);       // settle: obs coast 1.9; err 2.0 -> tap
  p.tick(1800, true, 97.9f, kFresh);       // tap over -> settling again
  p.tick(2000, true, 98.7f, kFresh);       // |98.7 - 99| <= 0.5 -> complete
  TEST_ASSERT_FALSE(p.seeking());

  TEST_ASSERT_TRUE(p.takeSeekReport(r));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 99.0f, r.target);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, r.start_height);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 95.1f, r.stop_height);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, r.stop_speed);  // terminal fallback
  TEST_ASSERT_FALSE(r.stop_speed_measured);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, r.predicted_coast);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.9f, r.observed_coast);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 98.7f, r.settled_height);
  TEST_ASSERT_EQUAL(1, r.correction_taps);
  TEST_ASSERT_EQUAL(1000, r.duration_ms);

  TEST_ASSERT_FALSE(p.takeSeekReport(r));  // one-shot: consumed above
}

void test_aborted_seek_yields_no_report() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  p.moveToHeight(99.0f, 1000, true, 90.0f);
  p.tick(1200, true, 91.0f, kFresh);  // driving
  p.stop();                           // user STOP mid-seek

  DeskMotionPlanner::SeekReport r{};
  TEST_ASSERT_FALSE(p.takeSeekReport(r));
}

void test_set_learned_state_sanitizes_implausible_values() {
  Recorder rec;
  DeskMotionPlanner p(kMin, kMax, kSeekTiming, kTunables, std::ref(rec));

  // A restore from a missing/corrupt flash file arrives as zeros.
  DeskMotionPlanner::LearnedState bad{};
  p.setLearnedState(bad);

  const auto s = p.learnedState();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, s.terminal_speed_up);  // back to seed
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, s.decel_down);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, s.tap_gain_up);

  // Plausible values pass through untouched.
  DeskMotionPlanner::LearnedState good{};
  good.terminal_speed_up = 3.1f;
  good.terminal_speed_down = 3.4f;
  good.decel_up = 1.5f;
  good.decel_down = 2.2f;
  good.tap_gain_up = 18.0f;
  good.tap_gain_down = 30.0f;
  p.setLearnedState(good);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.1f, p.learnedState().terminal_speed_up);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, p.learnedState().decel_up);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, p.learnedState().tap_gain_down);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_continuous_move_wakes_then_drives_at_cadence);
  RUN_TEST(test_stop_halts_continuous_movement);
  RUN_TEST(test_continuous_movement_stops_at_travel_limits);
  RUN_TEST(test_runaway_movement_stops_after_travel_timeout);
  RUN_TEST(test_continuous_movement_survives_a_stale_report_at_wakeup);
  RUN_TEST(test_continuous_movement_aborts_when_height_reports_go_stale);
  RUN_TEST(test_move_to_height_seeks_toward_target_and_stops_on_arrival);
  RUN_TEST(test_move_to_height_within_deadband_does_not_move);
  RUN_TEST(test_move_to_height_defers_until_a_height_is_known);
  RUN_TEST(test_move_to_height_rewakes_periodically_while_awaiting_report);
  RUN_TEST(test_move_to_height_gives_up_if_no_height_report_arrives);
  RUN_TEST(test_one_shot_command_wakes_then_sends_after_settle);
  RUN_TEST(test_one_shot_command_cancels_an_in_progress_move);
  RUN_TEST(test_stop_cancels_a_pending_one_shot);
  RUN_TEST(test_held_command_repeats_at_cadence_then_stops);
  RUN_TEST(test_seek_stops_early_by_fallback_coast_without_speed_estimate);
  RUN_TEST(test_seek_uses_measured_speed_to_shrink_the_coast_prediction);
  RUN_TEST(test_seek_sizes_correction_taps_quadratically);
  RUN_TEST(test_seek_learns_decel_from_the_observed_coast);
  RUN_TEST(test_seek_learns_terminal_speed_after_a_long_drive);
  RUN_TEST(test_seek_learning_is_gated_above_the_fine_resolution_limit);
  RUN_TEST(test_seek_stop_decision_runs_between_command_frames);
  RUN_TEST(test_correction_tap_ends_at_its_computed_duration);
  RUN_TEST(test_seek_dead_reckons_between_height_reports);
  RUN_TEST(test_coarse_zone_widens_the_deadband);
  RUN_TEST(test_seek_waits_for_height_stability_before_sampling);
  RUN_TEST(test_completed_seek_yields_a_report_once);
  RUN_TEST(test_aborted_seek_yields_no_report);
  RUN_TEST(test_set_learned_state_sanitizes_implausible_values);
  return UNITY_END();
}
