#pragma once

#include "DeskMotionPlanner.h"

// Persists DeskMotionPlanner's learned state (coast margins and correction tap
// rate) to LittleFS so convergence survives power cycles. Follows the same
// pattern as ConfigStore: stateless free functions, JSON-backed flat file.
namespace motion_store {

// Populate |out| from the stored state. Returns false if the file is absent
// or unreadable (first boot, or after a filesystem format); in that case |out|
// is left unchanged and the planner's compile-time seeds should be used.
bool load(DeskMotionPlanner::LearnedState& out);

// Write the current learned state to flash. Called by DeskController after
// each completed seek so the learned values are always up to date.
bool save(const DeskMotionPlanner::LearnedState& state);

// Delete the stored state so the planner starts fresh from its compile-time
// seeds on the next boot. Returns false only if the file exists but the
// remove fails; a missing file is treated as success.
bool reset();

}  // namespace motion_store
