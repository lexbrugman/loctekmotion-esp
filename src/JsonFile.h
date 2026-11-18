#pragma once

#include <ArduinoJson.h>

// Small JSON files on LittleFS (the config and the learned motion state):
// the open / parse / serialise boilerplate their stores share. LittleFS must
// already be mounted (see config_store::begin).
namespace json_file {

// Parse |path| into |doc|. Returns false if the file is absent or invalid.
bool load(const char* path, JsonDocument& doc);

// Write |doc| to |path|, replacing it. Returns false on write failure.
bool save(const char* path, const JsonDocument& doc);

}  // namespace json_file
