#pragma once

#include <cstddef>
#include <cstdint>

// DeskHeightDecoder parses the UART byte stream coming *from* a LoctekMotion /
// FlexiSpot control box and reconstructs the desk height shown on the handset's
// 7-segment display.
//
// The control box frames its messages as:
//
//     0x9b  LEN  TYPE  <payload...>  0x9d
//
// For a height report TYPE is 0x12 and LEN is 7 (some control boxes use 10).
// The three display digits are payload bytes 1..3 (i.e. frame bytes 3, 4, 5),
// each encoded as a standard 7-segment bitmask (bit0 = segment a ... bit6 =
// segment g, bit7 = decimal point). The decimal point on the *middle* digit
// indicates the value should be divided by ten (e.g. "7" "2." "5" -> 72.5).
//
// This class is intentionally free of any Arduino/ESP dependency so it can be
// exercised by host-side unit tests.
class DeskHeightDecoder {
 public:
  // Feed a single received byte. Returns true exactly when a new, valid height
  // has been decoded; read it with height().
  bool feed(uint8_t byte);

  float height() const { return height_; }
  bool has_height() const { return has_height_; }

  // True for exactly the feed() call that completes a frame showing "LOC" on
  // the display — what the desk shows in place of the height while child lock
  // is engaged.
  bool isLockDisplay() const { return lock_display_; }

  // Plausibility window (cm). Frames decoding outside this band are rejected as
  // line noise. Deliberately wider than any real desk's travel.
  static constexpr float kMinPlausible = 30.0f;
  static constexpr float kMaxPlausible = 250.0f;

 private:
  static constexpr uint8_t kFrameStart = 0x9b;
  static constexpr uint8_t kFrameEnd = 0x9d;
  static constexpr uint8_t kMsgTypeHeight = 0x12;
  static constexpr uint8_t kDecimalBit = 0x80;

  static constexpr int kHyphen = 10;
  static constexpr int kInvalid = -1;

  // Decode a 7-segment byte (decimal-point bit masked off) into 0-9, kHyphen,
  // or kInvalid for an unrecognised / blank pattern.
  static int decodeDigit(uint8_t seg);

  bool parseFrame();

  static constexpr size_t kMaxFrame = 12;
  uint8_t buf_[kMaxFrame] = {0};
  size_t len_ = 0;
  bool in_frame_ = false;

  float height_ = 0.0f;
  bool has_height_ = false;
  bool lock_display_ = false;
};
