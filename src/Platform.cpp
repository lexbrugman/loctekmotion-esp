#include "Platform.h"

#if defined(ESP8266)
#include <user_interface.h>
#elif defined(ESP32)
#include <esp_system.h>
#include <esp_timer.h>
#endif

namespace {

// Arbitrary non-zero marker: RTC memory powers up with unpredictable content
// on a cold boot, so a plain 0/1 flag could false-positive there — requiring
// a specific value makes "unset" unambiguous.
constexpr uint32_t kResetFlagMagic = 0x4c4d5246;  // "LMRF"

#if defined(ESP8266)
// First user-memory block (in 4-byte words); nothing else in this firmware
// touches RTC memory, so there's no risk of collision.
constexpr uint32_t kRtcSlot = 0;
#elif defined(ESP32)
// Survives any reset/restart (and deep sleep) without being zero- or
// static-initialized first — exactly the "leave it alone unless we
// explicitly write it" semantics this flag needs. Cleared only by a power
// loss, same as the ESP8266 RTC memory used above.
RTC_NOINIT_ATTR uint32_t g_reset_flag;
#endif

}  // namespace

namespace platform {

bool readResetFlag() {
#if defined(ESP8266)
  uint32_t value = 0;
  ESP.rtcUserMemoryRead(kRtcSlot, &value, sizeof(value));
  return value == kResetFlagMagic;
#else
  return g_reset_flag == kResetFlagMagic;
#endif
}

void writeResetFlag(bool set) {
  uint32_t value = set ? kResetFlagMagic : 0;  // non-const: rtcUserMemoryWrite takes uint32_t*
#if defined(ESP8266)
  ESP.rtcUserMemoryWrite(kRtcSlot, &value, sizeof(value));
#else
  g_reset_flag = value;
#endif
}

bool resetByButton() {
#if defined(ESP8266)
  return ESP.getResetInfoPtr()->reason == REASON_EXT_SYS_RST;
#else
  const esp_reset_reason_t reason = esp_reset_reason();
  return reason == ESP_RST_POWERON || reason == ESP_RST_EXT;
#endif
}

uint64_t uptimeSeconds() {
#if defined(ESP8266)
  return micros64() / 1000000;
#else
  return esp_timer_get_time() / 1000000;
#endif
}

std::unique_ptr<SecureClient> makeSecureClient() {
  auto client = std::unique_ptr<SecureClient>(new SecureClient());
  client->setInsecure();
#if defined(ESP8266)
  // Trim only the transmit buffer (we send just HTTP request headers) to
  // ease the ESP8266's tight heap; ESP32's mbedTLS client doesn't expose
  // this. The receive buffer must stay at BearSSL's full 16 KiB: the
  // release-asset CDN sends full-size TLS records, and a smaller buffer
  // breaks the firmware download with HTTPC_ERROR_CONNECTION_LOST (the small
  // version.txt replies are unaffected, which hides the problem).
  client->setBufferSizes(16384, 512);
#endif
  return client;
}

}  // namespace platform
