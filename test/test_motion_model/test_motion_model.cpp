// Host-side unit tests for MotionModel and VelocityEstimator.
// Run with: pio test -e native
#include <unity.h>

#include <cstdint>

#include "MotionModel.h"
#include "VelocityEstimator.h"

namespace {

constexpr MotionModel::Tunables kTunables{
    /*seed_terminal_speed=*/4.0f,
    /*seed_decel=*/2.0f,
    /*seed_tap_gain=*/25.0f,
    /*speed_learning_rate=*/0.5f,
    /*decel_learning_rate=*/0.5f,
    /*tap_learning_rate=*/0.5f,
    /*min_terminal_speed=*/0.5f,
    /*max_terminal_speed=*/10.0f,
    /*min_decel=*/0.2f,
    /*max_decel=*/20.0f,
    /*min_tap_gain=*/5.0f,
    /*max_tap_gain=*/100.0f,
};

constexpr VelocityEstimator::Config kEstimatorConfig{
    /*fine_resolution_max=*/100.0f,
    /*window=*/600,
    /*min_span=*/250,
    /*max_sample_age=*/400,
};

}  // namespace

// --- MotionModel -------------------------------------------------------------

void test_model_predicts_coast_from_speed_and_decel() {
  MotionModel m(kTunables);
  // v²/(2·decel): 4²/(2·2) = 4 cm; 3²/(2·2) = 2.25 cm; standstill coasts 0.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, m.coastDistance(true, 4.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.25f, m.coastDistance(true, 3.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.coastDistance(false, 0.0f));
}

void test_model_sizes_taps_quadratically_with_clamps() {
  MotionModel m(kTunables);
  // distance = gain·t² -> t = √(distance/gain): 1 cm at gain 25 -> 200 ms.
  TEST_ASSERT_EQUAL(200, m.tapDurationMs(true, 1.0f, 300));
  // √(2/25) s = 282 ms.
  TEST_ASSERT_EQUAL(282, m.tapDurationMs(true, 2.0f, 300));
  // 3 cm would need 346 ms — clamped to the 300 ms ceiling.
  TEST_ASSERT_EQUAL(300, m.tapDurationMs(true, 3.0f, 300));
  // A vanishing error never produces a zero-length tap.
  TEST_ASSERT_EQUAL(1, m.tapDurationMs(true, 0.00001f, 300));
}

void test_model_learns_decel_from_coast_observations() {
  MotionModel m(kTunables);
  // Observed: 3 cm/s coasted 1.5 cm -> decel 3.0. EMA at 0.5 from the 2.0
  // seed lands halfway: 2.5.
  m.learnCoast(true, 3.0f, 1.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, m.state().decel_up);
  // The other direction is untouched.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, m.state().decel_down);
  // Degenerate observations are ignored.
  m.learnCoast(true, 0.0f, 1.0f);
  m.learnCoast(true, 3.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, m.state().decel_up);
}

void test_model_learning_respects_clamps() {
  MotionModel m(kTunables);
  // A tiny observed coast implies an absurd decel — pinned to the max.
  m.learnCoast(true, 9.0f, 0.01f);  // observed: 81/0.02 = 4050 cm/s²
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, m.state().decel_up);
}

void test_model_learns_tap_gain_from_observed_movement() {
  MotionModel m(kTunables);
  // A 200 ms tap moved 0.6 cm: observed gain = 0.6/0.04 = 15. EMA at 0.5
  // from the 25 seed: 20.
  m.learnTap(false, 200, 0.6f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, m.state().tap_gain_down);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, m.state().tap_gain_up);
}

void test_model_learns_terminal_speed() {
  MotionModel m(kTunables);
  m.learnTerminalSpeed(true, 3.0f);  // EMA at 0.5 from the 4.0 seed: 3.5
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.5f, m.state().terminal_speed_up);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, m.state().terminal_speed_down);
}

void test_model_set_state_sanitizes_out_of_range_values() {
  MotionModel m(kTunables);
  MotionModel::State s{};  // all zeros — below every minimum
  s.decel_up = 1.5f;       // except one plausible value
  m.setState(s);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, m.state().terminal_speed_up);  // seed
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, m.state().decel_up);  // kept
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, m.state().decel_down);  // seed
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, m.state().tap_gain_up);  // seed
}

void test_model_reset_restores_seeds() {
  MotionModel m(kTunables);
  m.learnTerminalSpeed(true, 2.0f);
  m.learnCoast(true, 3.0f, 1.5f);
  m.reset();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, m.state().terminal_speed_up);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, m.state().decel_up);
}

// --- VelocityEstimator ---------------------------------------------------------

void test_estimator_measures_slope_over_window() {
  VelocityEstimator e(kEstimatorConfig);
  e.addSample(0, 90.0f);
  e.addSample(100, 90.3f);
  e.addSample(200, 90.6f);
  TEST_ASSERT_FALSE(e.valid(200));  // span 200 ms < 250 ms minimum
  e.addSample(300, 90.9f);
  TEST_ASSERT_TRUE(e.valid(300));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, e.speed());
}

void test_estimator_reports_negative_speed_moving_down() {
  VelocityEstimator e(kEstimatorConfig);
  e.addSample(0, 90.9f);
  e.addSample(150, 90.45f);
  e.addSample(300, 90.0f);
  TEST_ASSERT_TRUE(e.valid(300));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -3.0f, e.speed());
}

void test_estimator_goes_stale_without_fresh_samples() {
  VelocityEstimator e(kEstimatorConfig);
  e.addSample(0, 90.0f);
  e.addSample(300, 90.9f);
  TEST_ASSERT_TRUE(e.valid(300));
  TEST_ASSERT_FALSE(e.valid(800));  // newest sample is 500 ms old (> 400 max)
}

void test_estimator_rejects_coarse_samples_and_resets() {
  VelocityEstimator e(kEstimatorConfig);
  e.addSample(0, 99.0f);
  e.addSample(150, 99.5f);
  e.addSample(300, 99.9f);
  TEST_ASSERT_TRUE(e.valid(300));
  // Crossing into whole-centimetre territory wipes the estimate entirely —
  // a slope across mixed resolutions lies.
  e.addSample(400, 100.0f);
  TEST_ASSERT_FALSE(e.valid(400));
  // It stays dark until enough fine samples accumulate again.
  e.addSample(500, 99.9f);
  e.addSample(600, 99.6f);
  TEST_ASSERT_FALSE(e.valid(600));  // span 100 ms — still building up
  e.addSample(800, 99.0f);
  TEST_ASSERT_TRUE(e.valid(800));
}

void test_estimator_drops_samples_outside_the_window() {
  VelocityEstimator e(kEstimatorConfig);
  // 0..1200 at 3 cm/s, sampled every 100 ms: only the trailing 600 ms counts.
  for (uint32_t t = 0; t <= 1200; t += 100) {
    e.addSample(t, 90.0f + 0.003f * static_cast<float>(t));
  }
  TEST_ASSERT_TRUE(e.valid(1200));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, e.speed());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_model_predicts_coast_from_speed_and_decel);
  RUN_TEST(test_model_sizes_taps_quadratically_with_clamps);
  RUN_TEST(test_model_learns_decel_from_coast_observations);
  RUN_TEST(test_model_learning_respects_clamps);
  RUN_TEST(test_model_learns_tap_gain_from_observed_movement);
  RUN_TEST(test_model_learns_terminal_speed);
  RUN_TEST(test_model_set_state_sanitizes_out_of_range_values);
  RUN_TEST(test_model_reset_restores_seeds);
  RUN_TEST(test_estimator_measures_slope_over_window);
  RUN_TEST(test_estimator_reports_negative_speed_moving_down);
  RUN_TEST(test_estimator_goes_stale_without_fresh_samples);
  RUN_TEST(test_estimator_rejects_coarse_samples_and_resets);
  RUN_TEST(test_estimator_drops_samples_outside_the_window);
  return UNITY_END();
}
