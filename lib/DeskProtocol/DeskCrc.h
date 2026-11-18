#pragma once

#include <cstddef>
#include <cstdint>

namespace desk_proto {

// CRC-16/MODBUS (poly 0xa001 reflected, init 0xffff), the checksum every
// frame on the desk link carries over LEN, TYPE and the payload; on the wire
// the high byte goes first. constexpr, so the fixed command frames in
// DeskCommands.h are checksummed at compile time.
constexpr uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xa001)
                      : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

}  // namespace desk_proto
