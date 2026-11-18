#pragma once

#include <cstdint>

// State tracker for a retained ON/OFF MQTT entity whose *real* state is read
// back from the desk with a delay (e.g. derived from its display): child
// lock, the sit-stand alarm, movement availability.
//
// It solves two problems:
//
//   change tracking - retained state should be (re)published only when it
//   actually changes, plus once initially (the first real value must publish
//   even when it equals the default, so the broker is never left without a
//   state).
//
//   optimistic holds - after a toggle command the desk can take seconds to
//   reflect the new state (held button-presses, display detection windows).
//   request() publishes the *requested* state immediately and holds it
//   against the stale real state until the desk catches up or the settle
//   window expires - after which sync() reverts to whatever is real, so a
//   toggle that didn't take visibly snaps back rather than lying forever.
//
// Every publish - including request()'s snap-back when the desk is already in
// the requested state - must be recorded as the last published value: one the
// tracker doesn't know about makes it swallow the next real transition (it
// once left the alarm switch stuck ON).
//
// Pure logic (callers pass the time in), free of any Arduino/MQTT dependency,
// so it can be exercised by host-side unit tests.
class OptimisticSwitch {
 public:
  // Loop-side: reconcile the real state |actual| with the broker. Returns
  // true when a publish is due and sets |out| to the value to publish.
  // While an optimistic hold is active, a stale |actual| is suppressed; the
  // hold ends when |actual| confirms the published value or the hold expires.
  bool sync(bool actual, uint32_t now, bool& out) {
    if (optimistic_pending_) {
      if (actual == last_published_ ||
          static_cast<int32_t>(now - optimistic_until_) >= 0) {
        optimistic_pending_ = false;
      } else {
        return false;  // keep showing the requested state for now
      }
    }
    if (published_once_ && actual == last_published_) return false;
    published_once_ = true;
    last_published_ = actual;
    out = actual;
    return true;
  }

  // The broker may have lost its retained state (e.g. it restarted without
  // persistence): after a reconnect, publish again what it should be showing.
  // Returns true and sets |out| to that value — the requested state while an
  // optimistic hold is running — or false when nothing has been published
  // yet, in which case the next sync() publishes as usual.
  bool republish(bool& out) const {
    if (!published_once_) return false;
    out = last_published_;
    return true;
  }

  // Command-side: HA requested |want| while the desk reports |actual|.
  // Always sets |out| (publish it): the requested state when a command is
  // actually due, or the real state to snap HA's optimistic UI back when the
  // desk is already there. Returns true when the caller should send the
  // command to the desk; the optimistic hold then runs for |settle| ms.
  bool request(bool want, bool actual, uint32_t now, uint32_t settle, bool& out) {
    published_once_ = true;
    if (want != actual) {
      last_published_ = want;
      optimistic_pending_ = true;
      optimistic_until_ = now + settle;
      out = want;
      return true;
    }
    last_published_ = actual;
    optimistic_pending_ = false;
    out = actual;
    return false;
  }

 private:
  bool last_published_ = false;   // what the broker is showing
  bool published_once_ = false;   // guards the very first publish
  bool optimistic_pending_ = false;
  uint32_t optimistic_until_ = 0;
};
