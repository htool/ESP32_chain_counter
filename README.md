# ESP32 chain counter (SensESP)

Bidirectional windlass chain counter for **Sailor Hat ESP32**. Counts gypsy pulses with up/down direction, publishes rode length to Signal K, and optionally sends B&G proprietary PGN 130824 on NMEA 2000. Firmware is **OTA-flashable** over Wi-Fi.

SensESP 3 is ESP32-only. This is not an ESP8266 project.

## Hardware

Same pinout as the original Arduino sketch:

| Function | GPIO | Notes |
| --- | --- | --- |
| Chain sensor | 35 | Optocoupler, input-only |
| Down (out) | 36 | Optocoupler, active low |
| Up (in) | 39 | Optocoupler, active low |
| CAN TX | 32 | SH-ESP32 transceiver |
| CAN RX | 34 | SH-ESP32 transceiver |

Direction comes from the up/down inputs. A sensor edge adds or subtracts one pulse. Rode (m) = pulse count × distance per pulse (default 0.1675 m). Pulse count cannot go below 0. On boot, a stored count below 10 is treated as fully retrieved (0).

## Signal K paths

| Path | Direction |
| --- | --- |
| `winches.windlass.rode` | published (m) |
| `winches.windlass.pulseCount` | published; PUT / delta sets the count |
| `winches.windlass.distanceperpulse` | published; PUT / delta sets metres per pulse |

Compatible with `signalk-chain-plugin` (`winches.windlass.rode`).

NMEA 2000 TX of PGN 130824 starts after the first received N2K frame (same as the old sketch).

## Build and first flash (USB)

Install [PlatformIO](https://platformio.org/) (VS Code or `pio` CLI).

```bash
cp src/wifi_secrets.h.example src/wifi_secrets.h   # edit SSID/password
pio run -e shesp32 -t upload
pio device monitor
```

`src/wifi_secrets.h` is gitignored. Without it the device starts a Wi-Fi AP named `chain-counter` (password `thisisfine`) so you can set Wi-Fi in the SensESP web UI.

Default Signal K server is `192.168.3.1:3000` (boatnet). Change it in the web UI if needed.

If Signal K is HTTPS/TLS, flash `shesp32_espidf` instead of `shesp32`.

## OTA (remote flash)

OTA is enabled with password `thisisfine` (override `OTA_PASSWORD` in `wifi_secrets.h`). First image must be uploaded over USB so the OTA partition table is on the device.

Then in `platformio.ini` uncomment:

```ini
upload_protocol = espota
upload_port = DEVICE_IP
upload_flags =
    --auth=thisisfine
```

and run `pio run -e shesp32 -t upload`. Web UI is `http://chain-counter.local/` (or the device IP). Pulse count and distance per pulse are editable there and survive reboot.

## Legacy sketch

The Arduino IDE sketch is in `legacy/ESP32_chain_counter.ino`.
