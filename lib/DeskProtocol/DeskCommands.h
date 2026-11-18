#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "DeskCrc.h"

// Fixed command frames sent *to* the LoctekMotion control box.
//
// A command is only acted upon while the handset "screen" is awake; callers are
// expected to assert the screen-wake GPIO and/or send kWake first.
namespace desk_cmd {

struct Frame {
  const uint8_t* data;
  size_t len;
};

// A key frame: the handset's "button" message — TYPE 0x02 with a two-byte
// key code — checksummed at compile time (see desk_proto::crc16).
constexpr std::array<uint8_t, 8> keyFrame(uint8_t code_hi, uint8_t code_lo) {
  std::array<uint8_t, 8> f{0x9b, 0x06, 0x02, code_hi, code_lo, 0x00, 0x00, 0x9d};
  const uint16_t crc = desk_proto::crc16(f.data() + 1, 4);
  f[5] = static_cast<uint8_t>(crc >> 8);
  f[6] = static_cast<uint8_t>(crc & 0xff);
  return f;
}

inline constexpr auto kUp = keyFrame(0x01, 0x00);
inline constexpr auto kDown = keyFrame(0x02, 0x00);
inline constexpr auto kWake = keyFrame(0x00, 0x00);
inline constexpr auto kPreset1 = keyFrame(0x04, 0x00);
inline constexpr auto kPreset2 = keyFrame(0x08, 0x00);
inline constexpr auto kSit = keyFrame(0x00, 0x01);
inline constexpr auto kStand = keyFrame(0x10, 0x00);
inline constexpr auto kMemory = keyFrame(0x20, 0x00);
inline constexpr auto kAlarm = keyFrame(0x40, 0x00);
inline constexpr auto kChildLock = keyFrame(0x20, 0x00);  // Memory's key, held

// The checksums the reverse-engineered frames carry, pinning keyFrame() (and
// the CRC's byte order) to what the desk is known to accept.
static_assert(kUp[5] == 0xfc && kUp[6] == 0xa0, "Up");
static_assert(kDown[5] == 0x0c && kDown[6] == 0xa0, "Down");
static_assert(kWake[5] == 0x6c && kWake[6] == 0xa1, "Wake");
static_assert(kPreset1[5] == 0xac && kPreset1[6] == 0xa3, "Preset1");
static_assert(kPreset2[5] == 0xac && kPreset2[6] == 0xa6, "Preset2");
static_assert(kSit[5] == 0xac && kSit[6] == 0x60, "Sit");
static_assert(kStand[5] == 0xac && kStand[6] == 0xac, "Stand");
static_assert(kMemory[5] == 0xac && kMemory[6] == 0xb8, "Memory");
static_assert(kAlarm[5] == 0xac && kAlarm[6] == 0x90, "Alarm");

inline constexpr Frame Up{kUp.data(), kUp.size()};
inline constexpr Frame Down{kDown.data(), kDown.size()};
inline constexpr Frame Wake{kWake.data(), kWake.size()};
inline constexpr Frame Preset1{kPreset1.data(), kPreset1.size()};
inline constexpr Frame Preset2{kPreset2.data(), kPreset2.size()};
inline constexpr Frame Sit{kSit.data(), kSit.size()};
inline constexpr Frame Stand{kStand.data(), kStand.size()};
inline constexpr Frame Memory{kMemory.data(), kMemory.size()};
inline constexpr Frame Alarm{kAlarm.data(), kAlarm.size()};
inline constexpr Frame ChildLock{kChildLock.data(), kChildLock.size()};

// Any preset key pressed while the desk travels on its own cancels the travel;
// on an idle desk it starts a move instead. See DeskMotionPlanner::halt.
inline constexpr Frame Cancel = Preset1;

}  // namespace desk_cmd
