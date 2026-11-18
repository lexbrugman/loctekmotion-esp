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
// fields are filled in (see DeviceConfig; README "Running more than one").
inline constexpr char kDeviceId[] = "desk";
inline constexpr char kDeviceName[] = "Desk";
inline constexpr char kManufacturer[] = "LoctekMotion";
// The board is part of the model string (shown on the HA device page), so
// it's set per platform next to the pins below.

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
// State/command topics live under "<kTopicNamespace>/<device id>".
inline constexpr char kTopicNamespace[] = "loctekdesk";

// --- Captive-portal provisioning --------------------------------------------
// AP raised on first boot (or after a Wi-Fi reset); its SSID is this prefix
// plus a MAC-derived tag (see Provisioning.cpp's portalSsid()).
inline constexpr char kPortalSsidPrefix[] = "loctekmotion-";
inline constexpr char kPortalPassword[] = "loctekdesk";  // >= 8 chars
inline constexpr uint16_t kPortalTimeout = 180;          // seconds

// --- Board (pins, model name) ------------------------------------------
// TX -> desk RX, RX <- desk TX, and the screen/PIN20 wake line (held HIGH to
// keep the handset awake). DeskUart explains how each board runs the link.
#if defined(ESP8266)
// Wemos D1 mini; D5/D6/D2 are its silkscreen labels (GPIO14/12/4).
inline constexpr uint8_t kPinUartTx = D5;
inline constexpr uint8_t kPinUartRx = D6;
inline constexpr uint8_t kPinScreen = D2;
inline constexpr char kModel[] = "FlexiSpot (Wemos D1 mini)";
#elif defined(ESP32)
// ESP32 dev board; 16/17 are UART2's default pins.
inline constexpr uint8_t kPinUartTx = 17;
inline constexpr uint8_t kPinUartRx = 16;
inline constexpr uint8_t kPinScreen = 23;
inline constexpr char kModel[] = "FlexiSpot (ESP32)";
#else
#error "Unsupported board: add pin definitions and a model name for it here"
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
// Safety: abort a drive when height reports stop (the desk streams them
// continuously while moving, so a gap this long means the link stalled).
inline constexpr uint32_t kHeightStaleTimeout = 1500;
// Safety: abort a drive no fresh report has confirmed by now — kWakeSettle
// plus the display waking up and streaming again, with margin.
inline constexpr uint32_t kFirstReportTimeout = 2500;
// "Desk is moving" window (see DeskMotionPlanner::halt). At ~3 cm/s the
// display changes every ~30 ms below kFineHeightLimit and every ~330 ms in its
// whole-centimetre range above; this covers both while keeping the
// false-positive window just after the desk stops short.
inline constexpr uint32_t kMotionWindow = 600;
inline constexpr uint32_t kTargetTimeout = 8000;   // give up awaiting height
inline constexpr uint32_t kWakeRetryInterval = 2000;  // re-wake cadence while waiting
inline constexpr float kTargetDeadband = 0.3f;     // "close enough" to target
// "Close enough" at/above kFineHeightLimit, where the display shows whole
// centimetres: half that resolution (see DeskMotionPlanner::deadbandAt).
inline constexpr float kCoarseTargetDeadband = 0.5f;
// Maximum wait for stability after stopping; fires early once height is stable
// below kFineHeightLimit, and is the only exit at/above it (see
// kStableDuration). Must comfortably exceed the longest real coast (~1.5 s from cruise speed):
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
// Not used at/above kFineHeightLimit: whole-centimetre steps can sit still
// for over a second mid-coast, so settling there waits out kSeekSettleDelay.
inline constexpr uint32_t kStableDuration = 350;

// --- Motion model (learned kinematics; see MotionModel) -----------------------
// Seeds, EMA learning rates and plausibility clamps. Seeds err toward
// overestimating speed / coast, so early seeks stop slightly short and
// corrections tap forward (same direction as the drive) rather than reversing
// past the target.
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
// Plausibility clamps (see MotionModel::setState).
inline constexpr float kTerminalSpeedMin = 0.5f, kTerminalSpeedMax = 10.0f;  // cm/s
inline constexpr float kDecelMin = 0.2f, kDecelMax = 20.0f;                  // cm/s²
inline constexpr float kTapGainMin = 5.0f, kTapGainMax = 100.0f;             // cm/s²
// The display shows 0.1 cm below this height and whole centimetres from it
// up; speed estimation and learning only trust readings below it.
inline constexpr float kFineHeightLimit = 100.0f;
// Velocity estimator (see VelocityEstimator), at the ~108 ms report cadence.
inline constexpr uint32_t kSpeedWindow = 600;    // slope window (ms)
inline constexpr uint32_t kSpeedMinSpan = 250;   // min oldest..newest span (ms)
inline constexpr uint32_t kSpeedMaxAge = 400;    // newest-sample freshness (ms)
// A drive must run at least this long for its measured stop speed to count as
// a terminal-speed observation (shorter drives may still be ramping up).
inline constexpr uint32_t kTerminalSpeedDriveMin = 2000;
// Ceiling on a single correction tap drive, so a large miss can't produce an
// overshooting tap; bigger errors are closed across multiple taps.
inline constexpr uint32_t kCorrectionTapMax = 300;
// Child lock only registers as a sustained handset press, not a tap.
inline constexpr uint32_t kChildLockHold = 5000;
// After issuing a child-lock toggle, how long to keep showing the requested
// state optimistically before trusting the desk's own display again: covers
// kChildLockHold (the desk doesn't even start responding until the held
// command is released) plus a margin for the display to update.
inline constexpr uint32_t kChildLockToggleSettle = kChildLockHold + 2000;
// How long the display must look "alarm off" before the alarm counts as off
// (see DisplayState for the rules).
inline constexpr uint32_t kAlarmClearTimeout = 3000;
// A tap of the alarm button arms the reminder / cycles its interval;
// *disarming* requires holding it, so the switch's OFF is sent as a held
// command (like child lock).
inline constexpr uint32_t kAlarmOffHold = 3500;
// After toggling the alarm via HA, how long to keep showing the requested
// state optimistically: disarming takes kAlarmOffHold itself plus the
// kAlarmClearTimeout static-height window before the real state can confirm.
inline constexpr uint32_t kAlarmToggleSettle =
    kAlarmOffHold + kAlarmClearTimeout + 2000;

// --- Double-reset detection --------------------------------------------------
// A second RST press within this long of booting resets Wi-Fi into the setup
// portal (see provisioning::checkDoubleReset).
inline constexpr uint32_t kDoubleResetWindow = 4000;

inline constexpr uint32_t kTelemetryInterval = 30000;  // wifi signal publish
// Time Home Assistant gets to set up the entities after discovery, or after
// it (re)starts, before the device publishes state HA has to receive.
inline constexpr uint32_t kHaSettleDelay = 5000;
inline constexpr uint32_t kMqttRetryInterval = 5000;
// Grace period before a restart/Wi-Fi-reset takes effect, so the "restarting"
// log message has time to reach the broker before the link is torn down.
inline constexpr uint32_t kRestartFlushDelay = 100;
// Delay from UpdateManager::begin() to the first OTA check, so it doesn't
// compete with Wi-Fi/MQTT while they settle.
inline constexpr uint32_t kOtaStartupDelay = 15000;

// --- Over-the-air updates (pull from a GitHub release) -----------------------
// The channel is the rolling "<branch>-latest" release CI publishes; CI
// injects the repo and channel of the build, and local builds fall back to
// upstream's master-latest (see README "Over-the-air updates").
#ifndef FW_OTA_REPO
#define FW_OTA_REPO "lexbrugman/loctekmotion-esp"
#endif
#ifndef FW_OTA_CHANNEL
#define FW_OTA_CHANNEL "master-latest"
#endif
inline constexpr char kOtaBaseUrl[] =
    "https://github.com/" FW_OTA_REPO "/releases/download/" FW_OTA_CHANNEL;
// The channel's release page, linked from the HA Firmware entity.
inline constexpr char kOtaReleaseUrl[] =
    "https://github.com/" FW_OTA_REPO "/releases/tag/" FW_OTA_CHANNEL;
// Shown as the HA Firmware entity's title: which branch a device follows.
inline constexpr char kOtaChannel[] = FW_OTA_CHANNEL;
// Per board, so a device can never flash another board's image (it'd brick).
#if defined(ESP8266)
inline constexpr char kFirmwareAsset[] = "firmware-d1_mini.bin";
#elif defined(ESP32)
inline constexpr char kFirmwareAsset[] = "firmware-esp32dev.bin";
#endif
inline constexpr char kVersionAsset[] = "version.txt";
inline constexpr uint32_t kUpdateCheckInterval = 6UL * 60UL * 60UL * 1000UL;  // 6h

}  // namespace cfg
