#include "MotionStore.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "JsonFile.h"

namespace {
constexpr char kPath[] = "/motion.json";
}  // namespace

namespace motion_store {

bool load(DeskMotionPlanner::LearnedState& out) {
  JsonDocument doc;
  if (!json_file::load(kPath, doc)) return false;

  // Missing keys (e.g. a file written by an older firmware with a different
  // model) deserialize as 0, which setLearnedState sanitizes back to seeds.
  out.terminal_speed_up = doc["vu"] | 0.0f;
  out.terminal_speed_down = doc["vd"] | 0.0f;
  out.decel_up = doc["du"] | 0.0f;
  out.decel_down = doc["dd"] | 0.0f;
  out.tap_gain_up = doc["tu"] | 0.0f;
  out.tap_gain_down = doc["td"] | 0.0f;
  return true;
}

bool save(const DeskMotionPlanner::LearnedState& state) {
  JsonDocument doc;
  doc["vu"] = state.terminal_speed_up;
  doc["vd"] = state.terminal_speed_down;
  doc["du"] = state.decel_up;
  doc["dd"] = state.decel_down;
  doc["tu"] = state.tap_gain_up;
  doc["td"] = state.tap_gain_down;
  return json_file::save(kPath, doc);
}

bool reset() {
  if (!LittleFS.exists(kPath)) return true;
  return LittleFS.remove(kPath);
}

}  // namespace motion_store
