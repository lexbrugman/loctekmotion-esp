// End-to-end seek tests against a simulated desk (see SimDesk.h): the real
// DeskHeightDecoder and DeskMotionPlanner, wired up the way DeskController
// does, driving simulated motor physics and reading back its display stream.
// Where test_motion_planner checks individual decisions with hand-fed
// heights, these check outcomes — that seeks land, and that learning
// converges. Run with: pio test -e native
#include <unity.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "DeskHeightDecoder.h"
#include "DeskMotionPlanner.h"
#include "DisplayState.h"
#include "SimDesk.h"

namespace {

// The production settings and seeds from src/config.h, which can't be
// included here (it needs Arduino.h). Keep in step with it.
constexpr float kMinHeight = 65.5f;
constexpr float kMaxHeight = 131.0f;
constexpr uint32_t kAlarmClearTimeout = 3000;

constexpr DeskMotionPlanner::Settings kSettings = [] {
  DeskMotionPlanner::Settings s;
  s.wake_settle = 200;
  s.command_interval = 108;
  s.move_timeout = 40000;
  s.height_stale_timeout = 1500;
  s.first_report_timeout = 2500;
  s.motion_window = 600;
  s.target_timeout = 8000;
  s.wake_retry_interval = 2000;
  s.target_deadband = 0.3f;
  s.coarse_target_deadband = 0.5f;
  s.seek_settle_delay = 2000;
  s.stable_duration = 350;
  s.correction_tap_max = 300;
  s.terminal_speed_drive_min = 2000;
  s.fine_height_limit = 100.0f;
  s.speed_window = 600;
  s.speed_min_span = 250;
  s.speed_max_age = 400;
  return s;
}();

constexpr MotionModel::Tunables kTunables = [] {
  MotionModel::Tunables t;
  t.seed_terminal_speed = 3.0f;
  t.seed_decel = 1.8f;
  t.seed_tap_gain = 25.0f;
  t.speed_learning_rate = 0.3f;
  t.decel_learning_rate = 0.4f;
  t.tap_learning_rate = 0.3f;
  t.min_terminal_speed = 0.5f;
  t.max_terminal_speed = 10.0f;
  t.min_decel = 0.2f;
  t.max_decel = 20.0f;
  t.min_tap_gain = 5.0f;
  t.max_tap_gain = 100.0f;
  return t;
}();

// Fitted to a real FlexiSpot desk: with these numbers the planner learns
// within ~10% of what that desk's firmware learned (cruise 3.85 / 3.79 cm/s,
// coast deceleration 3.25 / 3.04 cm/s², tap gain 6.33 / 10.25 cm/s², up /
// down).
constexpr SimDesk::Params kStudyDesk = [] {
  SimDesk::Params p;
  p.speed_up = 3.85f;
  p.speed_down = 3.79f;
  p.accel_up = 3.0f;
  p.accel_down = 4.0f;
  p.decel_up = 4.0f;
  p.decel_down = 3.5f;
  p.drive_hold = 150;
  return p;
}();

// The same desk braking 40% softer, so it coasts further: above the fine
// limit its coast ends on long-held whole-centimetre readings.
constexpr SimDesk::Params kSoftBrakingDesk = [] {
  SimDesk::Params p = kStudyDesk;
  p.decel_up *= 0.6f;
  p.decel_down *= 0.6f;
  return p;
}();

constexpr SimDesk::Params kDesks[] = {kStudyDesk, kSoftBrakingDesk};

// The deadband a seek settling at |height| must land within.
float deadbandAt(float height) {
  return height >= kSettings.fine_height_limit ? kSettings.coarse_target_deadband
                                               : kSettings.target_deadband;
}

// The firmware's side of the link, as DeskController wires it: bytes from the
// desk go through the decoder into DisplayState, and the planner ticks every
// loop with the latest height and its age. The loop runs every simulated
// millisecond.
class Rig {
 public:
  struct Seek {
    bool completed = false;  // the planner produced a SeekReport
    DeskMotionPlanner::SeekReport report;
    float final_position = 0.0f;  // true position once the desk came to rest
  };

  explicit Rig(SimDesk::Params desk, MotionModel::Tunables tunables = kTunables)
      : desk_(desk, now_),
        display_(kAlarmClearTimeout),
        planner_(kMinHeight, kMaxHeight, kSettings, tunables,
                 [this](const desk_cmd::Frame& f) { desk_.receive(now_, f.data, f.len); }) {
    // main.cpp's boot wake, so a first height is known.
    planner_.issue(desk_cmd::Wake, now_);
    run(1000);
  }

  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) tick();
  }

  Seek seek(float target) {
    Seek s;
    planner_.moveToHeight(target, now_, decoder_.hasHeight(), decoder_.height());
    for (uint32_t i = 0; i < 60000 && planner_.pursuingTarget(); i++) tick();
    for (uint32_t i = 0; i < 5000 && !desk_.atRest(now_); i++) tick();
    s.completed = planner_.takeSeekReport(s.report);
    s.final_position = desk_.position();
    return s;
  }

  SimDesk& desk() { return desk_; }
  DeskMotionPlanner& planner() { return planner_; }
  uint32_t now() const { return now_; }

 private:
  void tick() {
    ++now_;
    desk_.advance(now_, [this](uint8_t b) {
      if (decoder_.feed(b) == DeskHeightDecoder::Reading::kHeight) {
        display_.onHeight(now_, decoder_.height());
      }
    });
    display_.tick(now_);
    planner_.tick(now_, decoder_.hasHeight(), decoder_.height(),
                  now_ - display_.lastHeightAt());
  }

  uint32_t now_ = 1000;
  SimDesk desk_;
  DeskHeightDecoder decoder_;
  DisplayState display_;
  DeskMotionPlanner planner_;
};

// Alternating targets across the fine-resolution range, of varied lengths.
const std::vector<float> kFineTargets = {95.0f, 72.0f, 88.5f, 70.0f, 98.0f,
                                         80.0f, 91.3f, 68.0f, 85.0f, 76.4f};

void assertLanded(const Rig::Seek& s, float target) {
  TEST_ASSERT_TRUE_MESSAGE(s.completed, "seek produced no report");
  const float tolerance = deadbandAt(s.report.settled_height);
  TEST_ASSERT_FLOAT_WITHIN(tolerance, target, s.report.settled_height);
  // The display rounds to 0.1 cm, so the desk itself may be off by half that
  // more.
  TEST_ASSERT_FLOAT_WITHIN(tolerance + 0.05f, target, s.final_position);
}

}  // namespace

void test_fresh_desk_lands_every_seek() {
  for (const SimDesk::Params& desk : kDesks) {
    Rig rig(desk);
    for (float target : kFineTargets) assertLanded(rig.seek(target), target);
  }
}

void test_fresh_desk_drive_stops_short() {
  // The seeds overestimate the coast, so the first drive in each direction
  // stops short of the target, and the correction taps close the rest from
  // the same side (see config.h).
  for (const SimDesk::Params& desk : kDesks) {
    Rig rig(desk);
    const Rig::Seek up = rig.seek(95.0f);
    assertLanded(up, 95.0f);
    TEST_ASSERT_TRUE(up.report.stop_height + up.report.observed_coast < 95.0f);
    TEST_ASSERT_TRUE(up.report.correction_taps > 0);

    const Rig::Seek down = rig.seek(72.0f);
    assertLanded(down, 72.0f);
    TEST_ASSERT_TRUE(down.report.stop_height - down.report.observed_coast > 72.0f);
    TEST_ASSERT_TRUE(down.report.correction_taps > 0);
  }
}

void test_learning_cuts_correction_taps() {
  // Same targets twice: the second round runs on the learned model and
  // needs fewer taps than the first did from the seeds.
  for (const SimDesk::Params& desk : kDesks) {
    Rig rig(desk);
    const size_t n = kFineTargets.size();
    std::vector<float> twice = kFineTargets;
    twice.insert(twice.end(), kFineTargets.begin(), kFineTargets.end());

    uint32_t first_round = 0, second_round = 0;
    for (size_t i = 0; i < twice.size(); i++) {
      const Rig::Seek s = rig.seek(twice[i]);
      assertLanded(s, twice[i]);
      (i < n ? first_round : second_round) += s.report.correction_taps;
    }
    TEST_ASSERT_TRUE(second_round < first_round);
    TEST_ASSERT_TRUE(second_round <= n / 2);
  }
}

void test_learned_coast_prediction_converges() {
  // After a round of seeks, the coast the planner predicts at cut-off
  // matches what the desk actually does.
  for (const SimDesk::Params& desk : kDesks) {
    Rig rig(desk);
    for (float target : kFineTargets) rig.seek(target);
    for (float target : kFineTargets) {
      const Rig::Seek s = rig.seek(target);
      assertLanded(s, target);
      TEST_ASSERT_FLOAT_WITHIN(0.4f, s.report.observed_coast, s.report.predicted_coast);
    }
  }
}

void test_learned_state_carries_over_a_restart() {
  // What MotionStore persists is enough: a fresh planner restored from it
  // seeks as well as the one that learned it.
  Rig trained(kStudyDesk);
  for (float target : kFineTargets) trained.seek(target);

  Rig restored(kStudyDesk);
  restored.planner().setLearnedState(trained.planner().learnedState());
  Rig fresh(kStudyDesk);

  uint32_t restored_taps = 0, fresh_taps = 0;
  for (size_t i = 0; i < 4; i++) {
    restored_taps += restored.seek(kFineTargets[i]).report.correction_taps;
    fresh_taps += fresh.seek(kFineTargets[i]).report.correction_taps;
  }
  TEST_ASSERT_TRUE(restored_taps < fresh_taps);
}

void test_coarse_zone_seeks_land_within_the_coarse_deadband() {
  // Above the fine limit the display shows whole centimetres and there's no
  // live speed estimate, yet seeks there still land — including on the
  // softer-braking desk, whose coast ends on readings that hold for well
  // over stable_duration.
  for (const SimDesk::Params& desk : kDesks) {
    Rig rig(desk);
    for (float target : kFineTargets) rig.seek(target);  // learn first
    for (int round = 0; round < 2; round++) {
      for (float target : {110.0f, 125.0f, 104.0f, 118.0f, 101.0f, 129.0f}) {
        assertLanded(rig.seek(target), target);
      }
    }
  }
}

void test_seek_to_the_top_stops_at_the_travel_limit() {
  Rig rig(kStudyDesk);
  rig.seek(kMaxHeight);
  TEST_ASSERT_FALSE(rig.planner().busy());
  TEST_ASSERT_TRUE(rig.desk().atRest(rig.now()));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, kMaxHeight, rig.desk().position());
}

void test_a_dead_rx_line_stops_the_desk() {
  // The height stream stops mid-drive: the planner must stop sending drive
  // frames, so the desk halts well short of a distant target.
  Rig rig(kStudyDesk);
  rig.planner().moveToHeight(98.0f, rig.now(), true, rig.desk().displayed());
  rig.run(3000);
  TEST_ASSERT_TRUE(rig.planner().moving());
  const float cut_at = rig.desk().position();
  rig.desk().cutRx();
  rig.run(kSettings.height_stale_timeout + 3000);
  TEST_ASSERT_FALSE(rig.planner().busy());
  TEST_ASSERT_TRUE(rig.desk().atRest(rig.now()));
  // Stale timeout at cruise speed, plus the coast.
  TEST_ASSERT_TRUE(rig.desk().position() < cut_at + 8.5f);
  TEST_ASSERT_TRUE(rig.desk().position() < 98.0f - 5.0f);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_fresh_desk_lands_every_seek);
  RUN_TEST(test_fresh_desk_drive_stops_short);
  RUN_TEST(test_learning_cuts_correction_taps);
  RUN_TEST(test_learned_coast_prediction_converges);
  RUN_TEST(test_learned_state_carries_over_a_restart);
  RUN_TEST(test_coarse_zone_seeks_land_within_the_coarse_deadband);
  RUN_TEST(test_seek_to_the_top_stops_at_the_travel_limit);
  RUN_TEST(test_a_dead_rx_line_stops_the_desk);
  return UNITY_END();
}
