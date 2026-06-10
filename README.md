# LoctekMotion / FlexiSpot desk - MQTT firmware

Lightweight, native firmware for controlling a LoctekMotion / FlexiSpot standing
desk from a **Wemos D1 mini (ESP8266)** or an **ESP32 dev board**. It talks to
Home Assistant over **MQTT with auto-discovery**, is provisioned on-device via
a **captive portal**, and **updates itself over-the-air** from a GitHub release.
Absolute-height positioning is closed-loop and **self-calibrating**: the
firmware learns each desk's motion characteristics and remembers them across
power cycles (see [Height seeking & self-calibration](#height-seeking--self-calibration)).

The desk's serial protocol was reverse-engineered by the
[iMicknl/LoctekMotion_IoT](https://github.com/iMicknl/LoctekMotion_IoT)
project - this firmware builds on that research.

## First-boot provisioning

No credentials are compiled in. On first boot (or after a Wi-Fi reset) the
device raises a Wi-Fi access point:

1. Connect to **`loctekmotion-<id>`** (password `loctekdesk`) - `<id>` is a
   short tag derived from the device's MAC address, so each desk's setup
   network is distinguishable from the others.
2. Pick your Wi-Fi network and enter its password.
3. Fill in the **Device ID / Device name** and **MQTT host / port / user /
   password** fields.
4. Save - the device stores everything and reconnects automatically.

To re-run setup later: press the **Wi-Fi setup** button in Home Assistant
(clears Wi-Fi config and reboots into the portal), or - if a config mistake
has made the device unreachable - double-press the board's **reset (RST)**
button within a few seconds, which triggers the same reset without needing
MQTT. Either way the portal pre-fills the MQTT fields with the current
values, so you can fix a bad host/port/credential in place.

### Running more than one

Device ID keys everything that must stay distinct per device: the MQTT client
id, the topic namespace (`loctekdesk/<device id>/...`), the HA device
identifier and entity `unique_id`s, and the Wi-Fi/mDNS hostname. Give each desk
its own Device ID (lower-case `[a-z0-9_]`, e.g. `study_desk` / `standing_desk`)
via the portal and they coexist cleanly on the same broker - one firmware build
serves any number of them. Device name is just the friendly label shown in HA.

## Entities exposed to Home Assistant

| Entity | Type | Notes |
| --- | --- | --- |
| Desk | cover | open/close/stop + position (0–100 %) |
| Target height | number | absolute height in cm |
| Height | sensor | decoded desk height |
| WiFi signal / Uptime / OTA channel | sensor | diagnostics |
| Preset 1/2, Sit, Stand | button | stored positions |
| Memory, Alarm, Wake screen | button | |
| Child lock | switch | tracks the desk's real state (the handset's "LOC" display) |
| Reset calibration | button | wipe the learned motion model (see [below](#height-seeking--self-calibration)) |
| Firmware update | button | force an OTA check now |
| Wi-Fi setup | button | reboot into the captive portal |
| Restart | button | reboot the ESP |

The desk ignores movement commands while child-locked, so every
movement-related entity (cover, target height, the preset/sit/stand/memory/
alarm buttons) is marked unavailable - greyed out in HA - while the lock is
on, rather than silently doing nothing.

## Height seeking & self-calibration

The desk's protocol has no native "go to height" command - it only streams the
handset display while moving. Absolute-height moves (the cover position and
the Target height number) are therefore closed-loop: the firmware drives the
desk toward the target, predicts how far it will coast after the drive stops,
and cuts the drive early so the coast lands on the target; any remaining error
is closed with short correction taps.

The predictions come from a small per-desk motion model
([`lib/DeskProtocol/MotionModel.*`](lib/DeskProtocol)) that learns the desk's
terminal speed, coast deceleration and tap gain - separately per direction -
from every seek it performs. Learned values persist on flash
(`src/MotionStore.*`) so calibration survives power cycles, and the **Reset
calibration** button wipes them back to the compile-time seeds in
[`src/config.h`](src/config.h). Fresh devices err on the undershoot side by
design, so early seeks finish with a forward tap instead of overshooting and
reversing.

## Hardware / pins

Two boards are supported, each with its own PlatformIO environment and pin set
(`src/config.h`):

| Function | Wemos D1 mini (`d1_mini`) | ESP32 dev board (`esp32dev`) |
| --- | --- | --- |
| UART TX → desk RX | D5 / GPIO14 | GPIO17 |
| UART RX ← desk TX | D6 / GPIO12 | GPIO16 |
| Screen wake (PIN20) | D2 / GPIO4 | GPIO23 |

Both run the desk link at 9600 baud over different transports: the ESP8266 has
only one hardware UART (kept free for USB debug logging), so it runs the link
over `SoftwareSerial`; the ESP32 has spare hardware UARTs and runs it on UART2
directly. `src/DeskUart.*` and `src/Platform.*` hide these (and the WiFi/OTA/TLS
API) differences behind small board-agnostic shims.

## Build & flash

[PlatformIO Core](https://docs.platformio.org/en/latest/core/) is a command-line
build tool (not an IDE - the IDE is an optional VS Code extension we don't use):

```bash
pip install platformio
pio run -e d1_mini  -t upload    # flash a Wemos D1 mini (ESP8266) over USB
pio run -e esp32dev -t upload    # flash an ESP32 dev board over USB
pio device monitor               # watch debug logs (115200)
```

Run the unit tests - protocol decoder, motion model and motion planner - on
your machine (no hardware needed):

```bash
pio test -e native
```

## Over-the-air updates

CI builds and tests both boards on every push and pull request (see
[`firmware.yml`](.github/workflows/firmware.yml)), and publishes a **rolling
release** tagged `<branch>-latest` - overwritten on each build, with one binary
per board attached - that's the OTA channel devices poll:

```
https://github.com/lexbrugman/loctekmotion-esp/releases/download/master-latest/firmware-d1_mini.bin
https://github.com/lexbrugman/loctekmotion-esp/releases/download/master-latest/firmware-esp32dev.bin
https://github.com/lexbrugman/loctekmotion-esp/releases/download/master-latest/version.txt           # short commit SHA, shared by both boards
```

- On boot and every 6 hours the device fetches `version.txt` and, if it differs
  from the running build, downloads and flashes its own board's
  `firmware-*.bin` (`cfg::kFirmwareAsset` - never the other board's, since that
  would brick it). The **Firmware update** button forces a check immediately.
- The build version (`-DFW_VERSION`) is the short commit SHA, so the running
  firmware's version always matches the commit it was built from.
- `kOtaBaseUrl` is built from `FW_OTA_REPO` and `FW_OTA_CHANNEL`, which CI
  injects as the repo it's running in (`<owner>/<repo>`) and `<branch>-latest`
  for the branch being built - so a binary always tracks the repo and channel
  it came from. Flash a feature-branch build onto a test device and it keeps
  following that branch from then on, without touching production devices.
  Local/dev builds and forks fall back to this upstream repo's
  `master-latest`.

These are CI-managed rolling tags, not hand-rolled releases. Pushes to the
default branch also archive an untouched `master-<timestamp>-<commit-sha>`
release with the same assets, so every build a production device may have run
stays browsable in chronological order; feature branches only get their
rolling channel, and a branch's releases are pruned automatically when the
branch is deleted (see [`cleanup.yml`](.github/workflows/cleanup.yml)).
