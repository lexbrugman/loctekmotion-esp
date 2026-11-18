#pragma once

#include "DeskMotionPlanner.h"

// Persists DeskMotionPlanner's learned state (the per-direction terminal
// speed, coast deceleration and tap gain — see MotionModel) to LittleFS so
// convergence survives power cycles. Stateless free functions over a JSON
// file, like ConfigStore.
namespace motion_store {

// Populate |out| from the stored state. Returns false if the file is absent
// or unreadable (first boot, or after a filesystem format); in that case |out|
// is left unchanged and the planner's compile-time seeds should be used.
bool load(DeskMotionPlanner::LearnedState& out);

// Write the learned state to flash. DeskController calls this when a seek
// ends and the learned values changed.
bool save(const DeskMotionPlanner::LearnedState& state);

// Delete the stored state so the planner starts fresh from its compile-time
// seeds on the next boot. Returns false only if the file exists but the
// remove fails; a missing file is treated as success.
bool reset();

}  // namespace motion_store
