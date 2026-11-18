// Host-side unit tests for DeferredActions / Countdown. Run with: pio test -e native
#include <unity.h>

#include <cstdint>

#include "DeferredActions.h"

namespace {
enum Action : uint8_t { kFirst, kSecond, kLast = 31 };
}  // namespace

void test_take_reports_a_request_exactly_once() {
  DeferredActions a;
  TEST_ASSERT_FALSE(a.take(kFirst));  // nothing requested yet

  a.request(kFirst);
  TEST_ASSERT_TRUE(a.pending(kFirst));
  TEST_ASSERT_TRUE(a.take(kFirst));
  TEST_ASSERT_FALSE(a.pending(kFirst));
  TEST_ASSERT_FALSE(a.take(kFirst));
}

void test_repeated_requests_collapse_into_one() {
  DeferredActions a;
  a.request(kFirst);
  a.request(kFirst);
  TEST_ASSERT_TRUE(a.take(kFirst));
  TEST_ASSERT_FALSE(a.take(kFirst));
}

void test_actions_are_independent() {
  DeferredActions a;
  a.request(kSecond);
  a.request(kLast);
  TEST_ASSERT_FALSE(a.take(kFirst));
  TEST_ASSERT_TRUE(a.take(kLast));
  TEST_ASSERT_TRUE(a.pending(kSecond));  // untouched by taking another
  TEST_ASSERT_TRUE(a.take(kSecond));
}

void test_countdown_is_due_only_once_armed_and_elapsed() {
  Countdown c;
  TEST_ASSERT_FALSE(c.armed());
  TEST_ASSERT_FALSE(c.due(5000));  // never armed

  c.arm(1000, /*delay=*/100);
  TEST_ASSERT_TRUE(c.armed());
  TEST_ASSERT_FALSE(c.due(1099));
  TEST_ASSERT_TRUE(c.due(1100));
  TEST_ASSERT_TRUE(c.due(1200));  // stays due
}

void test_countdown_disarm_and_rearm() {
  Countdown c;
  c.arm(1000, /*delay=*/100);
  c.disarm();
  TEST_ASSERT_FALSE(c.armed());
  TEST_ASSERT_FALSE(c.due(2000));

  c.arm(3000, /*delay=*/100);  // re-arming starts a fresh deadline
  TEST_ASSERT_FALSE(c.due(3050));
  TEST_ASSERT_TRUE(c.due(3100));
}

void test_countdown_survives_the_millis_rollover() {
  Countdown c;
  c.arm(0xffffffc0u, /*delay=*/100);  // deadline wraps past zero
  TEST_ASSERT_FALSE(c.due(0xfffffff0u));
  TEST_ASSERT_FALSE(c.due(10));
  TEST_ASSERT_TRUE(c.due(40));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_take_reports_a_request_exactly_once);
  RUN_TEST(test_repeated_requests_collapse_into_one);
  RUN_TEST(test_actions_are_independent);
  RUN_TEST(test_countdown_is_due_only_once_armed_and_elapsed);
  RUN_TEST(test_countdown_disarm_and_rearm);
  RUN_TEST(test_countdown_survives_the_millis_rollover);
  return UNITY_END();
}
