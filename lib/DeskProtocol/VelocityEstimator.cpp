#include "VelocityEstimator.h"

void VelocityEstimator::addSample(uint32_t now, float height) {
  if (height >= config_.fine_resolution_max) {
    // Whole-centimetre territory: a slope across coarse samples lies, so the
    // estimator stays dark until the desk is back in the fine range.
    reset();
    return;
  }

  dropOlderThan(now - config_.window);

  if (count_ == kCapacity) {  // full: overwrite the oldest
    head_ = (head_ + 1) % kCapacity;
    --count_;
  }
  samples_[(head_ + count_) % kCapacity] = {now, height};
  ++count_;
}

bool VelocityEstimator::valid(uint32_t now) const {
  if (count_ < 2) return false;
  const Sample& oldest = samples_[head_];
  const Sample& newest = samples_[(head_ + count_ - 1) % kCapacity];
  if (now - newest.at > config_.max_sample_age) return false;
  return newest.at - oldest.at >= config_.min_span;
}

float VelocityEstimator::speed() const {
  if (count_ < 2) return 0.0f;
  const Sample& oldest = samples_[head_];
  const Sample& newest = samples_[(head_ + count_ - 1) % kCapacity];
  const uint32_t dt = newest.at - oldest.at;
  if (dt == 0) return 0.0f;
  return (newest.height - oldest.height) * 1000.0f / static_cast<float>(dt);
}

void VelocityEstimator::dropOlderThan(uint32_t cutoff) {
  while (count_ > 0 && static_cast<int32_t>(samples_[head_].at - cutoff) < 0) {
    head_ = (head_ + 1) % kCapacity;
    --count_;
  }
}
