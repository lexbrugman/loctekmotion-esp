// Host-side unit tests for DisplayState. Run with: pio test -e native
#include <unity.h>

#include <cstdint>

#include "DisplayState.h"

namespace {
constexpr uint32_t kClear = 3000;
}  // namespace

void test_lock_display_locks_and_a_height_unlocks() {
  DisplayState d(kClear);
  TEST_ASSERT_FALSE(d.childLocked());
  d.onLockDisplay();
  TEST_ASSERT_TRUE(d.childLocked());
  d.onHeight(1000, 80.0f);
  TEST_ASSERT_FALSE(d.childLocked());
}

void test_alarm_display_arms_the_alarm() {
  DisplayState d(kClear);
  d.onAlarmDisplay(1000);
  TEST_ASSERT_TRUE(d.alarmOn());
}

void test_alarm_survives_movement() {
  // Armed, then the desk moves: the display shows the live height, which
  // keeps changing — that is no evidence the alarm went off.
  DisplayState d(kClear);
  d.onHeight(500, 80.0f);
  d.onAlarmDisplay(1000);
  for (uint32_t t = 1100; t <= 1000 + 2 * kClear; t += 100) {
    d.onHeight(t, 80.0f + static_cast<float>(t - 1000) / 1000.0f);
    d.tick(t);
  }
  TEST_ASSERT_TRUE(d.alarmOn());
}

void test_alarm_clears_on_a_static_height_without_alarm_frames() {
  // The alarm was disarmed on the handset: the display goes back to showing
  // a static height, with no "=" frames any more.
  DisplayState d(kClear);
  d.onAlarmDisplay(1000);
  d.onHeight(1100, 80.0f);
  d.onHeight(1000 + kClear, 80.0f);
  TEST_ASSERT_TRUE(d.alarmOn());  // not yet past the window
  d.onHeight(1100 + kClear + 1, 80.0f);
  TEST_ASSERT_FALSE(d.alarmOn());
}

void test_alarm_frames_keep_the_alarm_on() {
  // Height shown briefly after a move, while "=" frames keep arriving.
  DisplayState d(kClear);
  d.onHeight(1000, 80.0f);
  for (uint32_t t = 1000; t <= 1000 + 2 * kClear; t += 500) {
    d.onAlarmDisplay(t);
    d.onHeight(t + 100, 80.0f);
    d.tick(t + 100);
  }
  TEST_ASSERT_TRUE(d.alarmOn());
}

void test_alarm_clears_on_a_dark_display() {
  // While armed the display never sleeps, so no frames of any kind for the
  // window means the alarm is off.
  DisplayState d(kClear);
  d.onHeight(900, 80.0f);
  d.onAlarmDisplay(1000);
  d.tick(1000 + kClear);
  TEST_ASSERT_TRUE(d.alarmOn());
  d.tick(1000 + kClear + 1);
  TEST_ASSERT_FALSE(d.alarmOn());
}

void test_last_height_at_tracks_height_frames_only() {
  DisplayState d(kClear);
  TEST_ASSERT_EQUAL(0, d.lastHeightAt());
  d.onHeight(1000, 80.0f);
  d.onAlarmDisplay(1500);
  d.onLockDisplay();
  TEST_ASSERT_EQUAL(1000, d.lastHeightAt());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_lock_display_locks_and_a_height_unlocks);
  RUN_TEST(test_alarm_display_arms_the_alarm);
  RUN_TEST(test_alarm_survives_movement);
  RUN_TEST(test_alarm_clears_on_a_static_height_without_alarm_frames);
  RUN_TEST(test_alarm_frames_keep_the_alarm_on);
  RUN_TEST(test_alarm_clears_on_a_dark_display);
  RUN_TEST(test_last_height_at_tracks_height_frames_only);
  return UNITY_END();
}
