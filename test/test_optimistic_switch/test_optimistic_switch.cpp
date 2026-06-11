// Host-side unit tests for OptimisticSwitch. Run with: pio test -e native
#include <unity.h>

#include "OptimisticSwitch.h"

void test_first_sync_always_publishes() {
  // The first real value must reach the broker even when it equals the
  // tracker's default, so the entity is never left without a state.
  OptimisticSwitch s;
  bool out = true;
  TEST_ASSERT_TRUE(s.sync(false, 1000, out));
  TEST_ASSERT_FALSE(out);
}

void test_sync_publishes_only_on_change() {
  OptimisticSwitch s;
  bool out = false;
  TEST_ASSERT_TRUE(s.sync(false, 1000, out));
  TEST_ASSERT_FALSE(s.sync(false, 1100, out));  // unchanged — quiet
  TEST_ASSERT_TRUE(s.sync(true, 1200, out));    // changed — publish
  TEST_ASSERT_TRUE(out);
  TEST_ASSERT_FALSE(s.sync(true, 1300, out));
}

void test_request_sends_command_and_holds_optimistically() {
  OptimisticSwitch s;
  bool out = false;
  s.sync(false, 1000, out);  // broker shows OFF

  // HA asks ON while the desk still reports OFF: send the command and
  // publish the requested state right away.
  TEST_ASSERT_TRUE(s.request(true, false, 2000, /*settle=*/500, out));
  TEST_ASSERT_TRUE(out);

  // The stale real state must not flip the switch back during the hold...
  TEST_ASSERT_FALSE(s.sync(false, 2200, out));

  // ...and once the desk confirms, nothing new needs publishing.
  TEST_ASSERT_FALSE(s.sync(true, 2400, out));
  TEST_ASSERT_FALSE(s.sync(true, 2600, out));
}

void test_failed_toggle_reverts_after_the_hold_expires() {
  OptimisticSwitch s;
  bool out = false;
  s.sync(false, 1000, out);

  TEST_ASSERT_TRUE(s.request(true, false, 2000, /*settle=*/500, out));
  TEST_ASSERT_FALSE(s.sync(false, 2400, out));  // still inside the hold

  // The desk never took the command: past the hold, the real state reverts
  // the switch rather than lying forever.
  TEST_ASSERT_TRUE(s.sync(false, 2600, out));
  TEST_ASSERT_FALSE(out);
}

void test_request_for_current_state_snaps_back_without_command() {
  OptimisticSwitch s;
  bool out = false;
  s.sync(true, 1000, out);

  // HA's UI flipped but the desk is already there: no command, republish the
  // real state to snap the UI back.
  TEST_ASSERT_FALSE(s.request(true, true, 2000, 500, out));
  TEST_ASSERT_TRUE(out);
}

void test_snap_back_publish_is_recorded() {
  // Regression: toggling ON while a disarm was still in flight. The ON
  // request matched the (stale) real state, so only a snap-back publish
  // happened — and because it wasn't recorded, the later real OFF was
  // swallowed as "already published", leaving the switch stuck ON.
  OptimisticSwitch s;
  bool out = false;
  s.sync(true, 1000, out);  // alarm armed, broker shows ON

  // OFF request: disarm command sent, OFF published optimistically.
  TEST_ASSERT_TRUE(s.request(false, true, 2000, /*settle=*/8000, out));
  TEST_ASSERT_FALSE(out);

  // ON request while the desk still reports ON (disarm hold running): no
  // command, but the snap-back publishes — and records — ON.
  TEST_ASSERT_FALSE(s.request(true, true, 3000, 8000, out));
  TEST_ASSERT_TRUE(out);

  // The disarm completes and the real state goes OFF: that transition must
  // publish, not be swallowed against a stale record.
  TEST_ASSERT_TRUE(s.sync(false, 9000, out));
  TEST_ASSERT_FALSE(out);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_first_sync_always_publishes);
  RUN_TEST(test_sync_publishes_only_on_change);
  RUN_TEST(test_request_sends_command_and_holds_optimistically);
  RUN_TEST(test_failed_toggle_reverts_after_the_hold_expires);
  RUN_TEST(test_request_for_current_state_snaps_back_without_command);
  RUN_TEST(test_snap_back_publish_is_recorded);
  UNITY_END();
  return 0;
}
