#pragma once

#include <cstddef>
#include <cstdint>

// Fixed command frames sent *to* the LoctekMotion control box.
//
// A command is only acted upon while the handset "screen" is awake; callers are
// expected to assert the screen-wake GPIO and/or send kWake first.
namespace desk_cmd {

struct Frame {
  const uint8_t* data;
  size_t len;
};

inline constexpr uint8_t kUp[] = {0x9b, 0x06, 0x02, 0x01, 0x00, 0xfc, 0xa0, 0x9d};
inline constexpr uint8_t kDown[] = {0x9b, 0x06, 0x02, 0x02, 0x00, 0x0c, 0xa0, 0x9d};
inline constexpr uint8_t kWake[] = {0x9b, 0x06, 0x02, 0x00, 0x00, 0x6c, 0xa1, 0x9d};
inline constexpr uint8_t kPreset1[] = {0x9b, 0x06, 0x02, 0x04, 0x00, 0xac, 0xa3, 0x9d};
inline constexpr uint8_t kPreset2[] = {0x9b, 0x06, 0x02, 0x08, 0x00, 0xac, 0xa6, 0x9d};
inline constexpr uint8_t kSit[] = {0x9b, 0x06, 0x02, 0x00, 0x01, 0xac, 0x60, 0x9d};
inline constexpr uint8_t kStand[] = {0x9b, 0x06, 0x02, 0x10, 0x00, 0xac, 0xac, 0x9d};
inline constexpr uint8_t kMemory[] = {0x9b, 0x06, 0x02, 0x20, 0x00, 0xac, 0xb8, 0x9d};
inline constexpr uint8_t kAlarm[] = {0x9b, 0x06, 0x02, 0x40, 0x00, 0xac, 0x90, 0x9d};
inline constexpr uint8_t kChildLock[] = {0x9b, 0x06, 0x02, 0x20, 0x00, 0xac, 0xb8, 0x9d};

inline constexpr Frame Up{kUp, sizeof(kUp)};
inline constexpr Frame Down{kDown, sizeof(kDown)};
inline constexpr Frame Wake{kWake, sizeof(kWake)};
inline constexpr Frame Preset1{kPreset1, sizeof(kPreset1)};
inline constexpr Frame Preset2{kPreset2, sizeof(kPreset2)};
inline constexpr Frame Sit{kSit, sizeof(kSit)};
inline constexpr Frame Stand{kStand, sizeof(kStand)};
inline constexpr Frame Memory{kMemory, sizeof(kMemory)};
inline constexpr Frame Alarm{kAlarm, sizeof(kAlarm)};
inline constexpr Frame ChildLock{kChildLock, sizeof(kChildLock)};

}  // namespace desk_cmd
