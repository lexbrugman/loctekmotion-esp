#pragma once

#include <cstdint>

// Estimates the desk's travel speed (cm/s, signed: positive = up) from the
// decoded height stream by fitting a slope over a short trailing window of
// samples.
//
// The handset display only reports 0.1 cm resolution below 100 cm — above
// that it switches to whole centimetres, far too coarse for speed estimation.
// Any sample at or above fine_resolution_max therefore invalidates the
// estimator until clean samples accumulate again; callers must be prepared
// for valid() to be false and fall back to a model-based speed.
class VelocityEstimator {
 public:
  struct Config {
    float fine_resolution_max;  // samples at/above this height (cm) are rejected
    uint32_t window;            // trailing slope window (ms)
    uint32_t min_span;          // minimum oldest..newest span for a valid slope (ms)
    uint32_t max_sample_age;    // newest sample older than this -> invalid (ms)
  };

  explicit VelocityEstimator(Config config) : config_(config) {}

  // Drop all samples; the estimator reports invalid until enough fresh,
  // fine-resolution samples accumulate again.
  void reset() { count_ = 0; }

  // Feed a freshly decoded height report. Callers should feed each *new*
  // value once (repeats are harmless but add nothing).
  void addSample(uint32_t now, float height);

  // True when a slope is available: enough span, all samples fine-resolution,
  // and the newest sample is recent.
  bool valid(uint32_t now) const;

  // Slope over the window, cm/s, signed (positive = moving up). Only
  // meaningful while valid().
  float speed() const;

 private:
  struct Sample {
    uint32_t at = 0;
    float height = 0.0f;
  };

  // 8 samples at the desk's ~108 ms report cadence spans ~750 ms — comfortably
  // more than any sensible window config.
  static constexpr int kCapacity = 8;

  void dropOlderThan(uint32_t cutoff);

  Config config_;
  Sample samples_[kCapacity];
  int head_ = 0;  // index of the oldest sample
  int count_ = 0;
};
