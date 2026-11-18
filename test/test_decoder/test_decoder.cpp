// Host-side unit tests for DeskHeightDecoder. Run with: pio test -e native
#include <unity.h>

#include <cstddef>
#include <cstdint>

#include <vector>

#include "DeskCommands.h"
#include "DeskHeightDecoder.h"

namespace {

// 7-segment encodings used by the control box (decimal-point bit = 0x80).
constexpr uint8_t SEG[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66,
                           0x6d, 0x7d, 0x07, 0x7f, 0x6f};
constexpr uint8_t DP = 0x80;

// Wrap a body (TYPE + payload) in the desk's framing: start byte, LEN, the
// body, its CRC (high byte first) and the end byte.
std::vector<uint8_t> frame(std::vector<uint8_t> body) {
  std::vector<uint8_t> f{0x9b, static_cast<uint8_t>(body.size() + 3)};
  f.insert(f.end(), body.begin(), body.end());
  const uint16_t crc = desk_proto::crc16(f.data() + 1, f.size() - 1);
  f.push_back(static_cast<uint8_t>(crc >> 8));
  f.push_back(static_cast<uint8_t>(crc & 0xff));
  f.push_back(0x9d);
  return f;
}

// A (len 7) display report showing the three given segment bytes.
std::vector<uint8_t> display(uint8_t d1, uint8_t d2, uint8_t d3) {
  return frame({0x12, d1, d2, d3});
}

using Reading = DeskHeightDecoder::Reading;

// Feed a buffer; return the last reading any byte completed (kNone if none).
Reading feedAll(DeskHeightDecoder& d, const uint8_t* b, size_t n) {
  Reading last = Reading::kNone;
  for (size_t i = 0; i < n; i++) {
    const Reading r = d.feed(b[i]);
    if (r != Reading::kNone) last = r;
  }
  return last;
}

Reading feedAll(DeskHeightDecoder& d, const std::vector<uint8_t>& b) {
  return feedAll(d, b.data(), b.size());
}

}  // namespace

// "72.5" -> digits 7, 2(.), 5 with the decimal point on the middle digit.
void test_decodes_fractional_height() {
  DeskHeightDecoder d;
  const auto f = display(SEG[7], (uint8_t)(SEG[2] | DP), SEG[5]);
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_TRUE(d.hasHeight());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 72.5f, d.height());
}

// "131" -> three integer digits, no decimal point.
void test_decodes_integer_height() {
  DeskHeightDecoder d;
  const auto f = display(SEG[1], SEG[3], SEG[1]);
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// "65.5" -> bottom of travel.
void test_decodes_low_height() {
  DeskHeightDecoder d;
  const auto f = display(SEG[6], (uint8_t)(SEG[5] | DP), SEG[5]);
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 65.5f, d.height());
}

// A blank/zero leading digit is a transitional display and must be ignored.
void test_rejects_blank_leading_digit() {
  DeskHeightDecoder d;
  const auto f = display(0x00, SEG[1], SEG[1]);
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FALSE(d.hasHeight());
}

// A hyphen ("-") in a digit means the handset is mid-transition ("1-1").
void test_rejects_hyphen() {
  DeskHeightDecoder d;
  const auto f = display(SEG[1], 0x40, SEG[1]);
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, f));
}

// Non-height frames (e.g. a command echo, type 0x02) must be ignored.
void test_ignores_non_height_frames() {
  DeskHeightDecoder d;
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, desk_cmd::Up.data, desk_cmd::Up.len));
}

// A start byte mid-stream must resynchronise the framer.
void test_resynchronises_after_garbage() {
  DeskHeightDecoder d;
  auto f = display(SEG[1], SEG[3], SEG[1]);
  f.insert(f.begin(), {0xaa, 0x12, 0x00});  // junk
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// Some control boxes report with a longer (len 10) frame.
void test_accepts_len10_frame() {
  DeskHeightDecoder d;
  const auto f = frame({0x12, SEG[1], SEG[3], SEG[1], 0x00, 0x00, 0x00});
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// "LOC" -> child lock indicator, not a height.
void test_detects_lock_display() {
  DeskHeightDecoder d;
  TEST_ASSERT_TRUE(Reading::kLock == feedAll(d, display(0x38, 0x5c, 0x39)));
  TEST_ASSERT_FALSE(d.hasHeight());
}

// "=XX" -> sit-stand reminder alarm indicator, not a height. The '=' is
// exactly the handset's top+bottom segment pair (a+d, 0x09, both blinking in
// sync).
void test_detects_alarm_display() {
  DeskHeightDecoder d;
  TEST_ASSERT_TRUE(Reading::kAlarm == feedAll(d, display(0x09, SEG[0], SEG[5])));
  TEST_ASSERT_FALSE(d.hasHeight());
}

// Other horizontal-bar combinations are not the alarm glyph: they must be
// rejected as heights without setting the alarm indicator.
void test_other_bar_glyphs_are_not_an_alarm_display() {
  const uint8_t glyphs[] = {0x41, 0x48, 0x49};
  for (uint8_t glyph : glyphs) {
    DeskHeightDecoder d;
    TEST_ASSERT_TRUE(Reading::kNone == feedAll(d, display(glyph, SEG[0], SEG[5])));
    TEST_ASSERT_FALSE(d.hasHeight());
  }
}

// A lone horizontal bar is the transitional "-", not an '=': "-05" must be
// rejected as a height without being mistaken for the alarm indicator.
void test_lone_leading_hyphen_is_not_an_alarm_display() {
  DeskHeightDecoder d;
  TEST_ASSERT_TRUE(Reading::kNone == feedAll(d, display(0x40, SEG[0], SEG[5])));
  TEST_ASSERT_FALSE(d.hasHeight());
}

// The alarm blink's off phase shows " 00" (blank leading digit) — neither a
// height nor an alarm indicator.
void test_alarm_blink_off_phase_is_ignored() {
  DeskHeightDecoder d;
  TEST_ASSERT_TRUE(Reading::kNone == feedAll(d, display(0x00, SEG[0], SEG[0])));
  TEST_ASSERT_FALSE(d.hasHeight());
}

// The standard CRC-16/MODBUS check value pins the algorithm itself (the
// command frames' known checksums are pinned by static_asserts in
// DeskCommands.h).
void test_crc_matches_the_modbus_check_value() {
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL(0x4b37, desk_proto::crc16(check, sizeof(check)));
}

// A bit error that turns one valid digit into another ("72.5" -> "72.6")
// still looks like a height; only the checksum gives it away.
void test_rejects_a_corrupted_digit() {
  DeskHeightDecoder d;
  auto f = display(SEG[7], (uint8_t)(SEG[2] | DP), SEG[5]);
  f[5] = SEG[6];
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FALSE(d.hasHeight());
}

void test_rejects_a_corrupted_checksum() {
  DeskHeightDecoder d;
  auto f = display(SEG[7], (uint8_t)(SEG[2] | DP), SEG[5]);
  f[7] ^= 0x01;
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FALSE(d.hasHeight());
}

// Indicator frames are checksummed like any other: a corrupt "LOC" must not
// flip the child-lock state.
void test_corrupted_lock_display_is_ignored() {
  DeskHeightDecoder d;
  auto f = display(0x38, 0x5c, 0x39);
  f[6] ^= 0x01;
  TEST_ASSERT_TRUE(Reading::kNone == feedAll(d, f));
}

// A frame missing its end byte is dropped, and the next one still decodes.
void test_rejects_a_frame_without_its_end_byte() {
  DeskHeightDecoder d;
  auto f = display(SEG[7], (uint8_t)(SEG[2] | DP), SEG[5]);
  f.back() = 0x00;
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, display(SEG[1], SEG[3], SEG[1])));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// A frame cut short mid-payload must not swallow the one that follows.
void test_resynchronises_after_a_truncated_frame() {
  DeskHeightDecoder d;
  auto f = display(SEG[7], (uint8_t)(SEG[2] | DP), SEG[5]);
  f.resize(4);
  const auto next = display(SEG[1], SEG[3], SEG[1]);
  f.insert(f.end(), next.begin(), next.end());
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
}

// "32.1" has a checksum whose first byte equals the end byte (0x9d): frames
// are delimited by LEN, so it must not end the frame early.
void test_end_byte_inside_the_checksum_does_not_end_the_frame() {
  DeskHeightDecoder d;
  const auto f = display(SEG[3], (uint8_t)(SEG[2] | DP), SEG[1]);
  TEST_ASSERT_EQUAL(0x9d, f[6]);
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 32.1f, d.height());
}

// Likewise a start byte (0x9b) inside the checksum is data, not a resync.
void test_start_byte_inside_the_checksum_does_not_resynchronise() {
  // No three-digit display has such a checksum, so vary a len-10 report's
  // trailing payload byte until one does.
  for (int extra = 0; extra < 256; extra++) {
    const auto f = frame({0x12, SEG[1], SEG[3], SEG[1], static_cast<uint8_t>(extra), 0x00, 0x00});
    if (f[9] != 0x9b && f[10] != 0x9b) continue;
    DeskHeightDecoder d;
    TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, d.height());
    return;
  }
  TEST_ASSERT_TRUE(false);  // no such frame found
}

// An implausible LEN abandons the frame instead of overrunning the buffer.
void test_rejects_out_of_range_lengths() {
  DeskHeightDecoder d;
  const uint8_t too_long[] = {0x9b, 0x0b, 0x12, SEG[1], SEG[3], SEG[1], 0, 0, 0, 0, 0, 0, 0x9d};
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, too_long, sizeof(too_long)));
  const uint8_t too_short[] = {0x9b, 0x03, 0x12, 0x00, 0x9d};
  TEST_ASSERT_FALSE(Reading::kHeight == feedAll(d, too_short, sizeof(too_short)));
  TEST_ASSERT_TRUE(Reading::kHeight == feedAll(d, display(SEG[1], SEG[3], SEG[1])));
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
  RUN_TEST(test_detects_alarm_display);
  RUN_TEST(test_other_bar_glyphs_are_not_an_alarm_display);
  RUN_TEST(test_lone_leading_hyphen_is_not_an_alarm_display);
  RUN_TEST(test_alarm_blink_off_phase_is_ignored);
  RUN_TEST(test_crc_matches_the_modbus_check_value);
  RUN_TEST(test_rejects_a_corrupted_digit);
  RUN_TEST(test_rejects_a_corrupted_checksum);
  RUN_TEST(test_corrupted_lock_display_is_ignored);
  RUN_TEST(test_rejects_a_frame_without_its_end_byte);
  RUN_TEST(test_resynchronises_after_a_truncated_frame);
  RUN_TEST(test_end_byte_inside_the_checksum_does_not_end_the_frame);
  RUN_TEST(test_start_byte_inside_the_checksum_does_not_resynchronise);
  RUN_TEST(test_rejects_out_of_range_lengths);
  return UNITY_END();
}
