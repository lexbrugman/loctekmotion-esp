#include "JsonFile.h"

#include <LittleFS.h>

namespace json_file {

bool load(const char* path, JsonDocument& doc) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  const DeserializationError err = deserializeJson(doc, f);
  f.close();
  return !err;
}

bool save(const char* path, const JsonDocument& doc) {
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  const bool ok = serializeJson(doc, f) > 0;
  f.close();
  return ok;
}

}  // namespace json_file
