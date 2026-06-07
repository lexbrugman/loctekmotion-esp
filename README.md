# LoctekMotion / FlexiSpot desk — ESPHome firmware

ESPHome firmware for controlling a LoctekMotion / FlexiSpot standing desk over a
Wemos D1 mini (ESP8266), with a GitHub Actions workflow that compiles the
firmware on every push.

The device config is adapted from
[`iMicknl/LoctekMotion_IoT`](https://github.com/iMicknl/LoctekMotion_IoT)
(`tests/office-desk-esp8266.yaml`). Two changes were made so it builds as a
standalone repo:

1. The `loctekmotion_desk_height` component is pulled from the upstream repo via
   `external_components: github://...` instead of a local `../components` path.
2. Wi-Fi credentials and the API key now come from `!secret` lookups (a
   `secrets.yaml` file) instead of inline substitutions, so nothing sensitive
   lives in the committed config.

## Layout

```
.
├── office-desk.yaml              # the device configuration
├── secrets.yaml.example          # template for your credentials
├── .github/workflows/build.yml   # CI: compile + release
└── .gitignore                    # keeps secrets.yaml out of git
```

## Hardware / pins

| Function    | Substitution  | Pin         |
| ----------- | ------------- | ----------- |
| UART TX     | `tx_pin`      | D5 (GPIO14) |
| UART RX     | `rx_pin`      | D6 (GPIO12) |
| Screen wake | `screen_pin`  | D2 (GPIO4)  |

Adjust `min_height` / `max_height` (cm) and the pins at the top of
`office-desk.yaml` to match your desk and wiring.

## Build it in CI (GitHub Actions)

The workflow runs on every push to `main`, on pull requests, and on demand
(**Actions → Build firmware → Run workflow**). Each run compiles the config and
uploads the result as a build artifact you can download from the run summary.

To bake in your real credentials, add these **repository secrets**
(*Settings → Secrets and variables → Actions → New repository secret*):

| Secret name            | Value                                   |
| ---------------------- | --------------------------------------- |
| `WIFI_SSID`            | your Wi-Fi network name                 |
| `WIFI_PASSWORD`        | your Wi-Fi password                     |
| `AP_FALLBACK_PASSWORD` | password for the fallback hotspot       |
| `API_ENCRYPTION_KEY`   | Home Assistant API key (base64, 32 byte)|

If a secret is missing the build still succeeds using a placeholder value — handy
for verifying that the config compiles, but the resulting binary won't join your
Wi-Fi until the secrets are set and the workflow re-run.

## Build it locally

```bash
pip install esphome
cp secrets.yaml.example secrets.yaml   # then edit with your values
esphome compile office-desk.yaml
# flash over USB the first time:
esphome run office-desk.yaml
```
