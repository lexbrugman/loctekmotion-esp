#include "Platform.h"

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

std::unique_ptr<SecureClient> makeSecureClient() {
  auto client = std::unique_ptr<SecureClient>(new SecureClient());
  client->setInsecure();
#if defined(ESP8266)
  // Trim only the transmit buffer (our outbound traffic is a few hundred
  // bytes of HTTP request headers either way) to ease the ESP8266's tighter
  // heap budget; ESP32's mbedTLS-based client doesn't need or expose this.
  // The receive side must stay at BearSSL's full 16 KiB default: a smaller
  // one can't hold a full-size incoming TLS record, and the release-asset
  // CDN serving the multi-hundred-KB firmware binary (unlike the GitHub
  // Pages host this used to fetch from) sends some that large — shrinking
  // it here reproducibly broke the firmware download with "connection lost"
  // (HTTPC_ERROR_CONNECTION_LOST) while leaving the tiny version.txt check,
  // whose replies always fit in a small record, unaffected.
  client->setBufferSizes(16384, 512);
#endif
  return client;
}

}  // namespace platform
