#pragma once

#include <Stream.h>

#include <cstdint>

#if defined(ESP8266)
#include <SoftwareSerial.h>
#elif defined(ESP32)
#include <HardwareSerial.h>
#endif

// Presents the UART link to the desk's control box as a plain Stream,
// regardless of how the platform implements it: the ESP8266 has only one
// hardware UART (kept free here for USB debug logging), so the link runs
// over SoftwareSerial; the ESP32 has several spare hardware UARTs, so it
// uses one of those directly. DeskController only ever talks to stream().
class DeskUart {
 public:
  DeskUart(uint8_t rx_pin, uint8_t tx_pin);

  void begin(uint32_t baud);
  Stream& stream() { return impl_; }

 private:
#if defined(ESP8266)
  SoftwareSerial impl_;
#elif defined(ESP32)
  HardwareSerial& impl_;
  uint8_t rx_pin_;
  uint8_t tx_pin_;
#else
#error "Unsupported platform: back DeskUart with a Stream for it"
#endif
};
