# 🌱 ESP32 1–16 Zone Irrigation Controller

![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S3-blue)
![Zones](https://img.shields.io/badge/Zones-1%E2%80%9316-green)
![Web UI](https://img.shields.io/badge/Web%20UI-Local-orange)
![Weather](https://img.shields.io/badge/Weather-Aware-success)
![OTA](https://img.shields.io/badge/Updates-OTA-informational)

A flexible **ESP32-based irrigation controller** for **1–16 zones**, built for real gardens and real installations.

It combines **local scheduling**, **weather-aware delay logic**, optional **tank/mains source switching**, and a clean **web interface** for daily control and setup.

---

## Why This Project

This controller is designed to be simple to use, flexible to configure, and reliable in day-to-day operation.

It can run anything from a small home garden through to a larger multi-zone irrigation setup, with support for optional displays, KC868 hardware, and standalone local control even when Wi-Fi drops out.

---

## Highlights

- **1–16 irrigation zones**
- **Two start times per zone**
- **7-day scheduling**
- **Minute and second runtime control**
- Optional **tank ↔ mains switching**
- **Rain and wind delay logic**
- **Live weather display**
- **Modern local web dashboard**
- **Manual zone control**
- **OTA firmware updates**
- **Event logging with CSV export**
- Optional **TFT, OLED, or LCD display support**
- Compatible with **ESP32, ESP32-S3, and KC868-A6/A8**

---

## Built for Real Use

The dashboard gives you the information that matters at a glance:

- Current zone activity
- Tank level
- Water source mode
- Live weather conditions
- Rain or wind delay status
- Next scheduled watering event
- Manual On / Off control for each zone

All scheduling and control runs locally on the controller, so normal operation continues even if internet or Wi-Fi becomes unavailable.

---

## Smart Watering

Weather-aware control helps avoid unnecessary watering and adds an extra layer of protection to scheduled runs.

Supported logic includes:

- Rain delay from weather data or rain sensor
- Wind delay with configurable threshold
- Rain cooldown timer
- 24-hour rainfall limit handling

Watering can be cancelled and logged automatically when blocked by rain, wind, cooldown rules, or system lockout conditions.

---

## Hardware Options

The project supports several hardware styles depending on the build:

| Variant | Description |
|---|---|
| **ESP32 + SPI TFT** | Full-colour local display with system status and zone info |
| **ESP32 + I²C OLED** | Compact display option for smaller enclosures |
| **ESP8266 + I²C LCD** | Lightweight LCD version |
| **KC868-A6/A8** | Relay-expander based setup with built-in I/O convenience |

For KC868 hardware, PCF8574 expanders are supported at:

- `0x24` for relays
- `0x22` for inputs

If relay expanders are not detected, the controller can fall back to standard GPIO mode.

---

## Feature Overview

| Feature | Support |
|---|---|
| 1–16 zones | ✅ |
| Two start times per zone | ✅ |
| 7-day scheduling | ✅ |
| Sequential or concurrent operation | ✅ |
| Editable zone names | ✅ |
| Tank / mains switching | ✅ Optional |
| Weather integration | ✅ |
| Rain delay | ✅ |
| Wind delay | ✅ |
| Local web UI | ✅ |
| OTA updates | ✅ |
| Event logging | ✅ |
| TFT / OLED / LCD support | ✅ Optional |
| KC868-A6/A8 support | ✅ |

---

## Quick Setup

1. Flash the controller firmware
2. Connect to the setup access point: **ESPIrrigationAP**
3. Open **http://192.168.4.1**
4. Enter Wi-Fi credentials
5. Open **espirrigation.local** or the assigned IP address
6. Configure:
   - timezone
   - weather location
   - zone count
   - pin assignments
   - optional tank / mains settings

---

## Requirements

- **ESP32**, **ESP32-S3**, or **KC868-A6/A8**
- Relay outputs for your zones if not using KC868 hardware
- Tank sensor with **0–3.3 V analog output**
- Solenoid supply, typically **12 V DC** or **12/24 V AC**
- Optional **TFT, OLED, or LCD display**
- Wi-Fi for setup, web UI, OTA, and weather data

---

## Web Interface

Main endpoints:

| Path | Description |
|---|---|
| `/` | Dashboard |
| `/setup` | Configuration |
| `/status` | JSON status |
| `/events` | Event log |
| `/tank` | Tank calibration |
| `/download/events.csv` | Download event log |
| `/stopall` | Stop all zones |
| `/reboot` | Reboot controller |

---

## Screenshots

### Main Dashboard
![Main Dashboard](docs/images/main-dashboard.png)

### KC868-A6 Wiring
![KC868-A6 Wiring](docs/images/kc868-wiring.png)

### ESP32 + TFT + 8 Relay + 12V DC
![ESP32 TFT 8 Relay 12V DC](docs/images/esp32-tft-8relay-12vdc.png)

### ESP32-S3 + 6 Zone + 24V AC
![ESP32-S3 6 Zone 24V AC](docs/images/esp32s3-6zone-24vac.png)

---

## Notes

- Sequential mode is recommended unless your power supply can handle multiple solenoids at once
- Tank sensor input must not exceed **3.3 V**
- Local IP access may be more reliable than mDNS on some networks
