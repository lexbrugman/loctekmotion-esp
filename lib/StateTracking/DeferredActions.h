#pragma once

#include <cstdint>

// Bookkeeping for work that is requested in one place and carried out later
// from the main loop: an MQTT command handler must not start anything heavy
// or terminal (an OTA download, a restart) from inside the client's callback,
// so it only records the request.
//
// Pure logic (callers pass the time in), free of any Arduino dependency, so
// it can be exercised by host-side unit tests.

// A set of one-shot request flags, indexed by a small caller-defined enum
// (values 0..31). Requesting an action that is already pending is a no-op.
class DeferredActions {
 public:
  void request(uint8_t action) { pending_ |= mask(action); }
  bool pending(uint8_t action) const { return (pending_ & mask(action)) != 0; }

  // True exactly once per request: reports the action as due and clears it.
  bool take(uint8_t action) {
    if (!pending(action)) return false;
    pending_ &= ~mask(action);
    return true;
  }

 private:
  static uint32_t mask(uint8_t action) { return uint32_t{1} << action; }

  uint32_t pending_ = 0;
};

// A single deadline some way off — e.g. the short grace period between
// announcing a restart and performing it. Safe across the millis() rollover.
class Countdown {
 public:
  void arm(uint32_t now, uint32_t delay) {
    armed_ = true;
    at_ = now + delay;
  }
  void disarm() { armed_ = false; }
  bool armed() const { return armed_; }
  // True from the deadline onwards (it does not disarm itself).
  bool due(uint32_t now) const {
    return armed_ && static_cast<int32_t>(now - at_) >= 0;
  }

 private:
  bool armed_ = false;
  uint32_t at_ = 0;
};
