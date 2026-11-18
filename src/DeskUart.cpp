#include "DeskUart.h"

#include <Arduino.h>

#if defined(ESP8266)

DeskUart::DeskUart(uint8_t rx_pin, uint8_t tx_pin) : impl_(rx_pin, tx_pin) {}

void DeskUart::begin(uint32_t baud) { impl_.begin(baud); }

#elif defined(ESP32)

DeskUart::DeskUart(uint8_t rx_pin, uint8_t tx_pin)
    : impl_(Serial2), rx_pin_(rx_pin), tx_pin_(tx_pin) {}

void DeskUart::begin(uint32_t baud) {
  impl_.begin(baud, SERIAL_8N1, rx_pin_, tx_pin_);
}

#endif
