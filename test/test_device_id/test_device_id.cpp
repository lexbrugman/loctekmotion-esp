// Host-side unit tests for the device-id / hostname helpers.
// Run with: pio test -e native
#include <unity.h>

#include <cstring>
#include <string>

#include "DeviceId.h"

// Sanitize a copy of |in| in place and assert it equals |want|.
static void expectSanitized(const char* in, const char* want) {
  char buf[64];
  strncpy(buf, in, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  sanitizeDeviceId(buf);
  TEST_ASSERT_EQUAL_STRING(want, buf);
}

void test_sanitize_passes_valid_id_unchanged() {
  // A plain lower-case token (the factory default) must survive untouched.
  expectSanitized("desk", "desk");
  expectSanitized("study_desk", "study_desk");  // '_' is allowed here
  expectSanitized("desk2", "desk2");
}

void test_sanitize_lowercases() {
  expectSanitized("StudyDesk", "studydesk");
  expectSanitized("DESK", "desk");
}

void test_sanitize_replaces_illegal_chars_with_underscore() {
  expectSanitized("study desk", "study_desk");  // space
  expectSanitized("desk-1", "desk_1");          // hyphen isn't a topic char
  expectSanitized("a/b#c", "a_b_c");            // MQTT wildcards/separators
}

void test_sanitize_empty_stays_empty() {
  expectSanitized("", "");
}

void test_hostname_maps_underscore_to_hyphen() {
  // The bug this fixes: '_' is illegal in an ESP8266 Wi-Fi hostname.
  TEST_ASSERT_EQUAL_STRING("study-desk", deviceHostname("study_desk").c_str());
  TEST_ASSERT_EQUAL_STRING("a-b-c", deviceHostname("a_b_c").c_str());
}

void test_hostname_leaves_legal_ids_unchanged() {
  TEST_ASSERT_EQUAL_STRING("desk", deviceHostname("desk").c_str());
  TEST_ASSERT_EQUAL_STRING("desk2", deviceHostname("desk2").c_str());
  TEST_ASSERT_EQUAL_STRING("", deviceHostname("").c_str());
}

void test_sanitize_then_hostname_is_a_valid_hostname() {
  // The real pipeline: portal input -> sanitize -> hostname must contain only
  // [a-z0-9-].
  char buf[64];
  strncpy(buf, "Study Desk!", sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  sanitizeDeviceId(buf);                              // -> "study_desk_"
  const std::string host = deviceHostname(buf);       // -> "study-desk-"
  for (char c : host) {
    bool legal = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-';
    TEST_ASSERT_TRUE(legal);
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_sanitize_passes_valid_id_unchanged);
  RUN_TEST(test_sanitize_lowercases);
  RUN_TEST(test_sanitize_replaces_illegal_chars_with_underscore);
  RUN_TEST(test_sanitize_empty_stays_empty);
  RUN_TEST(test_hostname_maps_underscore_to_hyphen);
  RUN_TEST(test_hostname_leaves_legal_ids_unchanged);
  RUN_TEST(test_sanitize_then_hostname_is_a_valid_hostname);
  UNITY_END();
  return 0;
}
