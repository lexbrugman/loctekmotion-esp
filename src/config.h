#pragma once

// ---------------------------------------------------------------------------
// Compile-time configuration. No credentials live here: Wi-Fi and MQTT settings
// are provisioned on-device via a captive portal (see ConfigStore / WiFiManager)
// because the published firmware is public.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <cstdint>

namespace cfg {

// --- Identity ---------------------------------------------------------------
// Factory defaults for the device id/name, used until the captive portal's
// "Device ID" / "Device name" fields are filled in (see DeviceConfig /
// ConfigStore::load). The id keys Home Assistant discovery, MQTT topics, the
// MQTT client id and the Wi-Fi/mDNS hostname — running more than one of these
// on the same broker requires giving each a unique id (only [a-z0-9_]), which
// is exactly what those provisioned fields are for.
inline constexpr char kDeviceId[] = "desk";
inline constexpr char kDeviceName[] = "Desk";
inline constexpr char kManufacturer[] = "LoctekMotion";
inline constexpr char kModel[] = "FlexiSpot (Wemos D1 mini)";

// Build version, injected by CI as -DFW_VERSION=<short-sha> (see firmware.yml).
// The OTA check compares this against the published version.txt, so it must
// equal the commit the binary was built from.
#ifndef FW_VERSION
#define FW_VERSION dev
#endif
#define CFG_STR2(x) #x
#define CFG_STR(x) CFG_STR2(x)
inline constexpr char kSwVersion[] = CFG_STR(FW_VERSION);

// --- Topics -----------------------------------------------------------------
inline constexpr char kDiscoveryPrefix[] = "homeassistant";
// State/command topics live under "<kTopicNamespace>/<device id>" — see
// MqttManager::begin, which builds that prefix from the provisioned id so
// each device gets its own namespace without a separate provisioned field.
inline constexpr char kTopicNamespace[] = "loctekdesk";

// --- Captive-portal provisioning --------------------------------------------
// AP raised on first boot (or after a Wi-Fi reset) to enter Wi-Fi + MQTT config.
// The SSID is this prefix plus a short MAC-derived tag (see main.cpp's
// portalSsid()) so each physical device raises a uniquely identifiable
// network — handy for telling multiple desks' setup APs apart.
inline constexpr char kPortalSsidPrefix[] = "loctekmotion-";
inline constexpr char kPortalPassword[] = "loctekdesk";  // >= 8 chars
inline constexpr uint16_t kPortalTimeout = 180;          // seconds

// --- Pins --------------------------------------------------------------
// Board-specific (see DeskUart for how the link itself is implemented per
// platform); everything else in the firmware is written against these names.
#if defined(ESP8266)
// Wemos D1 mini: D5 (GPIO14, TX -> desk RX), D6 (GPIO12, RX <- desk TX),
// D2 (GPIO4, screen/PIN20 wake line, held HIGH to keep the handset awake).
// D5/D6 are the board's silkscreen labels (see <Arduino.h>'s pins_arduino.h);
// they aren't the hardware-UART pins, so DeskUart runs the link over
// SoftwareSerial, leaving the hardware Serial free for USB debug logging.
inline constexpr uint8_t kPinUartTx = D5;
inline constexpr uint8_t kPinUartRx = D6;
inline constexpr uint8_t kPinScreen = D2;
#elif defined(ESP32)
// ESP32 dev board: GPIO17 (TX -> desk RX), GPIO16 (RX <- desk TX),
// GPIO23 (screen/PIN20 wake line). 16/17 are UART2's default pins, so
// DeskUart runs the link over a real hardware UART, leaving Serial (UART0)
// free for USB debug logging — the same role SoftwareSerial plays above.
inline constexpr uint8_t kPinUartTx = 17;
inline constexpr uint8_t kPinUartRx = 16;
inline constexpr uint8_t kPinScreen = 23;
#else
#error "Unsupported board: add pin definitions for it here"
#endif
inline constexpr uint32_t kUartBaud = 9600;

// --- Desk geometry (cm) -----------------------------------------------------
inline constexpr float kMinHeight = 65.5f;
inline constexpr float kMaxHeight = 131.0f;
inline constexpr char kHeightUnit[] = "cm";

// --- Timing (ms) ------------------------------------------------------------
inline constexpr uint32_t kCommandInterval = 108;  // movement frame cadence
inline constexpr uint32_t kWakeSettle = 200;       // wait after wake before cmd
inline constexpr uint32_t kMoveTimeout = 40000;    // safety: max single travel
// Safety: abort an active continuous move if no fresh height report arrives
// within this window — the desk streams its display continuously while
// moving, so a gap this long means the link (not just the display) has
// stalled and the limit/target checks can no longer be trusted.
inline constexpr uint32_t kHeightStaleTimeout = 1500;
inline constexpr uint32_t kTargetTimeout = 8000;   // give up awaiting height
inline constexpr uint32_t kWakeRetryInterval = 2000;  // re-wake cadence while waiting
inline constexpr float kTargetDeadband = 0.3f;     // "close enough" to target
// "Close enough" when the settled reading is in the coarse zone (at/above
// kFineHeightLimit the display reports whole centimetres). Half the 1 cm
// resolution is the tightest deadband every target can satisfy there; anything
// smaller leaves the planner tapping back and forth between the two readings
// bracketing the target until the attempt cap runs out.
inline constexpr float kCoarseTargetDeadband = 0.5f;
// Maximum wait for stability after stopping; fires early once height is stable.
// Must comfortably exceed the longest real coast (~1.5 s from cruise speed):
// if this fires mid-coast, the "settled" height truncates every coast
// observation, the learned deceleration converges above its true value, and
// seeks systematically overshoot instead of creeping up from the undershoot
// side.
inline constexpr uint32_t kSeekSettleDelay = 2000;
// How long height must remain unchanged before the post-drive sample is taken.
// Ensures we measure the desk's truly final position, not a mid-coast reading.
// At 0.1 cm display resolution, a creep slower than 0.5 cm/s produces steps
// more than 200 ms apart, so this must be long enough not to mistake the slow
// tail of a coast for stability; 350 ms tolerates creep down to ~0.3 cm/s.
inline constexpr uint32_t kStableDuration = 350;

// --- Motion model (learned kinematics; see MotionModel) -----------------------
// Seeking predicts the coast distance from the live travel speed and a learned
// per-direction deceleration rate, and sizes correction taps from a learned
// quadratic tap-gain. The constants below are the seeds, EMA learning rates,
// and plausibility clamps; the learned values converge per desk and persist
// across power cycles (see MotionStore).
//
// Seeds err toward overestimating speed / coast, so early seeks stop slightly
// short and corrections tap forward (same direction as the drive) rather than
// reversing past the target.
inline constexpr float kTerminalSpeedSeed = 3.0f;  // cm/s; typical cruise speed
// Coast deceleration: 1.8 cm/s² gives ~2.5 cm of coast from 3 cm/s — the
// empirically observed ballpark for these desks.
inline constexpr float kDecelSeed = 1.8f;
// Tap gain k in distance = k·t² (t in seconds): a short tap accelerates from
// standstill, so distance grows quadratically with duration. k=25 means a
// 200 ms tap covers ~1 cm.
inline constexpr float kTapGainSeed = 25.0f;
inline constexpr float kSpeedLearningRate = 0.3f;
inline constexpr float kDecelLearningRate = 0.4f;
inline constexpr float kTapLearningRate = 0.3f;  // tap dynamics are noisier — slower
// Plausibility clamps: learned values outside these ranges are treated as
// corrupt on restore (reset to seed) and learning updates are pinned to them.
inline constexpr float kTerminalSpeedMin = 0.5f, kTerminalSpeedMax = 10.0f;  // cm/s
inline constexpr float kDecelMin = 0.2f, kDecelMax = 20.0f;                  // cm/s²
inline constexpr float kTapGainMin = 5.0f, kTapGainMax = 100.0f;             // cm/s²
// The handset display reports 0.1 cm resolution below 100 cm but whole
// centimetres at/above it — speed estimation and model learning only trust
// readings below this; predictions still apply everywhere.
inline constexpr float kFineHeightLimit = 100.0f;
// Velocity estimator: slope fitted over a trailing window of height reports
// (~108 ms cadence), needing a minimum span to be meaningful and going stale
// quickly once reports stop.
inline constexpr uint32_t kSpeedWindow = 600;    // slope window (ms)
inline constexpr uint32_t kSpeedMinSpan = 250;   // min oldest..newest span (ms)
inline constexpr uint32_t kSpeedMaxAge = 400;    // newest-sample freshness (ms)
// A drive must run at least this long for its measured stop speed to count as
// a terminal-speed observation (shorter drives may still be ramping up).
inline constexpr uint32_t kTerminalSpeedDriveMin = 2000;
// Ceiling on a single correction tap drive, so a large miss can't produce an
// overshooting tap; bigger errors are closed across multiple taps.
inline constexpr uint32_t kCorrectionTapMax = 300;
// How long to hold a held-style command (e.g. Child Lock): the desk only
// registers some toggles when the signal mimics a sustained handset
// button-press, not a single tap — matches the upstream ESPHome config's
// hold-then-auto-release duration for the same command.
inline constexpr uint32_t kChildLockHold = 5000;
// After issuing a child-lock toggle, how long to keep showing the requested
// state optimistically before trusting the desk's own display again: covers
// kChildLockHold (the desk doesn't even start responding until the held
// command is released) plus a margin for the display to update.
inline constexpr uint32_t kChildLockToggleSettle = kChildLockHold + 2000;

// --- Double-reset detection --------------------------------------------------
// Pressing the board's RST button twice within this window of booting
// triggers the same Wi-Fi reset as the "Wi-Fi setup" HA button — a way back
// into the captive portal that doesn't depend on MQTT/HA being reachable
// (e.g. after a config mistake locks the device out of its broker). Detected
// via a flag in RTC memory (see Platform::read/writeResetFlag): it survives
// a reset but is cleared by a power cycle, so a double press can't be
// confused with someone simply power-cycling the desk. Every board has a
// reset button, unlike the FLASH/BOOT buttons some (but not all — notably the
// Wemos D1 mini) carry onboard, so this works everywhere without relying on
// extra hardware.
inline constexpr uint32_t kDoubleResetWindow = 4000;

inline constexpr uint32_t kTelemetryInterval = 30000;  // wifi/uptime publish
inline constexpr uint32_t kMqttRetryInterval = 5000;
inline constexpr uint32_t kWifiRetryInterval = 10000;
// Grace period before a restart/Wi-Fi-reset takes effect, so the "restarting"
// log message has time to reach the broker before the link is torn down.
inline constexpr uint32_t kRestartFlushDelay = 100;
// Delay before the first OTA check, so it doesn't compete with Wi-Fi/MQTT for
// bandwidth while they're still settling right after boot.
inline constexpr uint32_t kOtaStartupDelay = 15000;

// --- Over-the-air updates (pull from a GitHub release) -----------------------
// CI builds and tests every branch push, and keeps a rolling release per branch
// tagged "<branch>-latest" with the build output attached as assets (see
// .github/workflows/firmware.yml). Release assets on a public repo are served
// from stable, anonymous URLs:
//   https://github.com/<owner>/<repo>/releases/download/<branch>-latest/firmware.bin
//   https://github.com/<owner>/<repo>/releases/download/<branch>-latest/version.txt
// CI injects -DFW_OTA_REPO="<owner>/<repo>" (from GITHUB_REPOSITORY) and
// -DFW_OTA_CHANNEL="<branch>-latest" for the branch it's building (see
// firmware.yml), so a binary always tracks the repo and channel it was built
// from — flash a feature branch's build and the device follows that branch
// from then on, no manual repointing needed. Plain local/dev builds (no
// flags) fall back to this upstream repo's master-latest channel. The device
// reflashes only when version.txt differs from kSwVersion.
#ifndef FW_OTA_REPO
#define FW_OTA_REPO "lexbrugman/loctekmotion-esp"
#endif
#ifndef FW_OTA_CHANNEL
#define FW_OTA_CHANNEL "master-latest"
#endif
inline constexpr char kOtaBaseUrl[] =
    "https://github.com/" FW_OTA_REPO "/releases/download/" FW_OTA_CHANNEL;
// Exposed to Home Assistant as a diagnostic sensor (see MqttManager::announce)
// so it's visible at a glance which channel — and therefore which branch — a
// given device is following; handy when a feature-branch build has been
// flashed onto a test device alongside production ones.
inline constexpr char kOtaChannel[] = FW_OTA_CHANNEL;
// Each board gets its own asset name (CI publishes one binary per board to
// every channel) so a device can never fetch and flash another board's image
// — the flash layouts differ enough that doing so would brick it.
#if defined(ESP8266)
inline constexpr char kFirmwareAsset[] = "firmware-d1_mini.bin";
#elif defined(ESP32)
inline constexpr char kFirmwareAsset[] = "firmware-esp32dev.bin";
#endif
inline constexpr char kVersionAsset[] = "version.txt";
inline constexpr uint32_t kUpdateCheckInterval = 6UL * 60UL * 60UL * 1000UL;  // 6h

}  // namespace cfg
