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

DeskHeightDecoder::Reading DeskHeightDecoder::feed(uint8_t byte) {
  // Total frame size once LEN (buf_[1]) is in: start + LEN + LEN more bytes.
  const size_t total = len_ >= 2 ? static_cast<size_t>(buf_[1]) + 2 : 0;
  // The two checksum bytes may take any value, the start byte's included.
  const bool in_checksum = total != 0 && len_ >= total - 3 && len_ < total - 1;

  if (byte == kFrameStart && !(in_frame_ && in_checksum)) {
    // Anywhere else a start byte (re)synchronises the framer, even mid-frame.
    in_frame_ = true;
    len_ = 0;
    buf_[len_++] = byte;
    return Reading::kNone;
  }

  if (!in_frame_) {
    return Reading::kNone;  // waiting for a frame to begin
  }

  buf_[len_++] = byte;

  if (len_ == 2) {
    // LEN just arrived: a frame that can't be valid (or wouldn't fit) is
    // abandoned here, and the framer waits for the next start byte.
    if (byte < kMinLen || static_cast<size_t>(byte) + 2 > kMaxFrame) in_frame_ = false;
    return Reading::kNone;
  }

  if (len_ < static_cast<size_t>(buf_[1]) + 2) return Reading::kNone;  // frame still filling

  in_frame_ = false;
  return parseFrame();
}

DeskHeightDecoder::Reading DeskHeightDecoder::parseFrame() {
  if (buf_[len_ - 1] != kFrameEnd) return Reading::kNone;
  // Checksum covers LEN through the last payload byte.
  const uint16_t crc = desk_proto::crc16(buf_ + 1, len_ - 4);
  if (buf_[len_ - 3] != (crc >> 8) || buf_[len_ - 2] != (crc & 0xff)) return Reading::kNone;

  const uint8_t msg_len = buf_[1];
  const uint8_t msg_type = buf_[2];
  if (msg_type != kMsgTypeHeight) return Reading::kNone;
  if (msg_len != 7 && msg_len != 10) return Reading::kNone;

  // The desk shows "LOC" — digit bytes for L, o, C with the decimal-point bit
  // masked off — in place of the height while child lock is engaged.
  if ((buf_[3] & ~kDecimalBit) == 0x38 && (buf_[4] & ~kDecimalBit) == 0x5c &&
      (buf_[5] & ~kDecimalBit) == 0x39) {
    return Reading::kLock;
  }

  // The sit-stand reminder alarm replaces the height with "=XX" (blinking):
  // the '=' is exactly the top and bottom segments (a+d). The blink's off
  // phase (blank leading digit) falls through to the zero/blank rejection
  // below, and anything else unrecognised is rejected as a height anyway.
  if ((buf_[3] & ~kDecimalBit) == 0x09) {
    return Reading::kAlarm;
  }

  const int d1 = decodeDigit(buf_[3]);
  const int d2 = decodeDigit(buf_[4]);
  const int d3 = decodeDigit(buf_[5]);

  // The leading digit is never zero on these desks (travel is ~65-131 cm), so a
  // zero/blank there means a transitional or empty display - reject it.
  if (d1 < 1 || d1 > 9) return Reading::kNone;
  // A hyphen or unrecognised pattern in the lower digits means the handset is
  // mid-transition (e.g. showing "---"); skip until it settles.
  if (d2 < 0 || d2 > 9) return Reading::kNone;
  if (d3 < 0 || d3 > 9) return Reading::kNone;

  float value = d1 * 100.0f + d2 * 10.0f + d3;
  if (buf_[4] & kDecimalBit) {  // decimal point rides on the middle digit
    value /= 10.0f;
  }

  if (value < kMinPlausible || value > kMaxPlausible) return Reading::kNone;

  height_ = value;
  has_height_ = true;
  return Reading::kHeight;
}
