#include "DeskHeightDecoder.h"

int DeskHeightDecoder::decodeDigit(uint8_t seg) {
  switch (seg & ~kDecimalBit) {  // ignore the decimal-point bit
    case 0x3f: return 0;
    case 0x06: return 1;
    case 0x5b: return 2;
    case 0x4f: return 3;
    case 0x66: return 4;
    case 0x6d: return 5;
    case 0x7d: return 6;
    case 0x07: return 7;
    case 0x7f: return 8;
    case 0x6f: return 9;
    case 0x40: return kHyphen;  // a lone middle segment ("-")
    default: return kInvalid;   // blank (0x00) or noise
  }
}

bool DeskHeightDecoder::feed(uint8_t byte) {
  if (byte == kFrameStart) {
    // A start byte always (re)synchronises the framer, even mid-frame.
    in_frame_ = true;
    len_ = 0;
    buf_[len_++] = byte;
    lock_display_ = false;
    alarm_display_ = false;
    return false;
  }

  if (!in_frame_) {
    return false;  // waiting for a frame to begin
  }

  if (len_ < kMaxFrame) {
    buf_[len_++] = byte;
  } else {
    // Overlong frame: give up and wait for the next start byte.
    in_frame_ = false;
    return false;
  }

  if (byte == kFrameEnd) {
    in_frame_ = false;
    return parseFrame();
  }

  return false;
}

bool DeskHeightDecoder::parseFrame() {
  // Need at least: start, len, type, d1, d2, d3.
  if (len_ < 6) return false;

  const uint8_t msg_len = buf_[1];
  const uint8_t msg_type = buf_[2];
  if (msg_type != kMsgTypeHeight) return false;
  if (msg_len != 7 && msg_len != 10) return false;

  // The desk shows "LOC" — digit bytes for L, o, C with the decimal-point bit
  // masked off — in place of the height while child lock is engaged.
  if ((buf_[3] & ~kDecimalBit) == 0x38 && (buf_[4] & ~kDecimalBit) == 0x5c &&
      (buf_[5] & ~kDecimalBit) == 0x39) {
    lock_display_ = true;
    return false;
  }

  // The sit-stand reminder alarm replaces the height with "=XX" (blinking):
  // the '=' is exactly the top and bottom segments (a+d). The blink's off
  // phase (blank leading digit) falls through to the zero/blank rejection
  // below, and anything else unrecognised is rejected as a height anyway.
  if ((buf_[3] & ~kDecimalBit) == 0x09) {
    alarm_display_ = true;
    return false;
  }

  const int d1 = decodeDigit(buf_[3]);
  const int d2 = decodeDigit(buf_[4]);
  const int d3 = decodeDigit(buf_[5]);

  // The leading digit is never zero on these desks (travel is ~65-131 cm), so a
  // zero/blank there means a transitional or empty display - reject it.
  if (d1 < 1 || d1 > 9) return false;
  // A hyphen or unrecognised pattern in the lower digits means the handset is
  // mid-transition (e.g. showing "---"); skip until it settles.
  if (d2 < 0 || d2 > 9) return false;
  if (d3 < 0 || d3 > 9) return false;

  float value = d1 * 100.0f + d2 * 10.0f + d3;
  if (buf_[4] & kDecimalBit) {  // decimal point rides on the middle digit
    value /= 10.0f;
  }

  if (value < kMinPlausible || value > kMaxPlausible) return false;

  height_ = value;
  has_height_ = true;
  return true;
}
