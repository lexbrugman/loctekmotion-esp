// Host-side unit tests for DeskHeightDecoder. Run with: pio test -e native
#include <unity.h>

#include <cstddef>
#include <cstdint>

#include "DeskHeightDecoder.h"

namespace {

// 7-segment encodings used by the control box (decimal-point bit = 0x80).
constexpr uint8_t SEG[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66,
                           0x6d, 0x7d, 0x07, 0x7f, 0x6f};
constexpr uint8_t DP = 0x80;

// Feed a buffer; return true if any byte completed a valid height frame.
bool feedAll(DeskHeightDecoder& d, const uint8_t* b, size_t n) {
  bool got = false;
  for (size_t i = 0; i < n; i++) got = d.feed(b[i]) || got;
  return got;
}

}  // namespace

// "72.5" -> digits 7, 2(.), 5 with the decimal point on the middle digit.
void test_decodes_fractional_height() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x07, 0x12, SEG[7], (uint8_t)(SEG[2] | DP),
                       SEG[5], 0x00, 0x00, 0x9d};
  TEST_ASSERT_TRUE(feedAll(d, f, sizeof(f)));
  TEST_ASSERT_TRUE(d.has_height());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 72.5f, d.height());
}

// "131" -> three integer digits, no decimal point.
void test_decodes_integer_height() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x07, 0x12, SEG[1], SEG[3],
                       SEG[1], 0x00, 0x00, 0x9d};
  TEST_ASSERT_TRUE(feedAll(d, f, sizeof(f)));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// "65.5" -> bottom of travel.
void test_decodes_low_height() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x07, 0x12, SEG[6], (uint8_t)(SEG[5] | DP),
                       SEG[5], 0x00, 0x00, 0x9d};
  TEST_ASSERT_TRUE(feedAll(d, f, sizeof(f)));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 65.5f, d.height());
}

// A blank/zero leading digit is a transitional display and must be ignored.
void test_rejects_blank_leading_digit() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x07, 0x12, 0x00, SEG[1],
                       SEG[1], 0x00, 0x00, 0x9d};
  TEST_ASSERT_FALSE(feedAll(d, f, sizeof(f)));
  TEST_ASSERT_FALSE(d.has_height());
}

// A hyphen ("-") in a digit means the handset is mid-transition ("1-1").
void test_rejects_hyphen() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x07, 0x12, SEG[1], 0x40,
                       SEG[1], 0x00, 0x00, 0x9d};
  TEST_ASSERT_FALSE(feedAll(d, f, sizeof(f)));
}

// Non-height frames (e.g. a command echo, type 0x02) must be ignored.
void test_ignores_non_height_frames() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x06, 0x02, 0x01, 0x00, 0xfc, 0xa0, 0x9d};
  TEST_ASSERT_FALSE(feedAll(d, f, sizeof(f)));
}

// A start byte mid-stream must resynchronise the framer.
void test_resynchronises_after_garbage() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0xaa, 0x12, 0x00,  // junk
                       0x9b, 0x07, 0x12, SEG[1], SEG[3], SEG[1], 0x00, 0x00, 0x9d};
  TEST_ASSERT_TRUE(feedAll(d, f, sizeof(f)));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// Some control boxes report with a longer (len 10) frame.
void test_accepts_len10_frame() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x0a, 0x12, SEG[1], SEG[3], SEG[1],
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x9d};
  TEST_ASSERT_TRUE(feedAll(d, f, sizeof(f)));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// "LOC" -> child lock indicator, not a height. isLockDisplay() is set on the
// frame-ending byte and clears once the next frame starts.
void test_detects_lock_display() {
  DeskHeightDecoder d;
  const uint8_t f[] = {0x9b, 0x07, 0x12, 0x38, 0x5c, 0x39, 0x00, 0x00, 0x9d};
  TEST_ASSERT_FALSE(feedAll(d, f, sizeof(f)));
  TEST_ASSERT_FALSE(d.has_height());
  TEST_ASSERT_TRUE(d.isLockDisplay());

  d.feed(0x9b);  // next frame starts; the indicator is no longer "current"
  TEST_ASSERT_FALSE(d.isLockDisplay());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_decodes_fractional_height);
  RUN_TEST(test_decodes_integer_height);
  RUN_TEST(test_decodes_low_height);
  RUN_TEST(test_rejects_blank_leading_digit);
  RUN_TEST(test_rejects_hyphen);
  RUN_TEST(test_ignores_non_height_frames);
  RUN_TEST(test_resynchronises_after_garbage);
  RUN_TEST(test_accepts_len10_frame);
  RUN_TEST(test_detects_lock_display);
  return UNITY_END();
}
