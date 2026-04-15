![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S3-blue)
![Zones](https://img.shields.io/badge/Zones-1%E2%80%9316-green)
![Web UI](https://img.shields.io/badge/Web%20UI-Local-orange)
![Weather](https://img.shields.io/badge/Weather-Aware-success)
![OTA](https://img.shields.io/badge/Updates-OTA-informational)

# 🌱 ESP32 1–16 Zone Irrigation Controller

**ESP32-based irrigation controller** for managing **1–16 irrigation zones/solenoids**, with optional **tank/mains source switching**, **optional weather-aware delay logic**, and a easy to use **local web interface**.

Built for real-world irrigation setups, from small home gardens to larger multi-zone systems.

---

## ✨ Features

- **1–16 irrigation zones**
- **Two start times per zone**
- **7-day scheduling**
- **Minute + second runtime control**
- **Sequential or concurrent operation**
- **Editable zone names**
- Optional **tank ↔ mains switching**
- Optional **Rain and wind-aware delays**
- **Live weather display**
- **Local web dashboard**
- **OTA firmware updates**
- **Event logging with CSV export**
- Optional **TFT, OLED, or LCD display support**
- Compatible with **ESP32**, **ESP32-S3**, and **KC868-A6/A8**
- Watering logic continues to run locally if Wi-Fi drops

---

## 📋 Feature Overview

| Feature | Support |
|---|---|
| 1–16 zones | ✅ |
| Two start times per zone | ✅ |
| 7-day scheduling | ✅ |
| Minute + second precision | ✅ |
| Sequential mode | ✅ |
| Concurrent mode | ✅ |
| Editable zone names | ✅ |
| Tank / mains switching | ✅ Optional |
| Weather integration | ✅ |
| Rain delay | ✅ |
| Wind delay | ✅ |
| OTA updates | ✅ |
| Event logging | ✅ |
| TFT / OLED / LCD support | ✅ Optional |
| KC868-A6/A8 support | ✅ |

---

## 🖥 Supported Hardware

| Variant | Description |
|---|---|
| **ESP32 + 240×320 SPI TFT** | Full-colour display for system status, active zones, delays, and water source mode |
| **ESP32 + I²C OLED** | Compact low-pin-count display option |
| **ESP8266 + I²C LCD** | Lightweight 16×2 LCD version |
| **KC868-A6/A8** | Relay-expander based setup with built-in I/O convenience |

Optional power saving:
- Add a **photoresistor + resistor** for display backlight control
- Screen can dim or turn off when the enclosure door is closed

---

## ⏱ Scheduling

- **1–16 configurable zones**
- **Two start times per zone**
- Optional second run with its **own duration**
- **7-day scheduling**
- **Minute and second precision**
- Run zones:
  - **Sequentially** (default)
  - **All at once** if your power supply allows
- Zone names are stored directly on the controller

---

## 🌦 Weather Integration

Use **Open-Meteo** to get your latitude and longitude:

1. Visit Open-Meteo
2. Enter your coordinates in **Setup**
3. Save settings

### Live Weather Data

The controller can display:

- Temperature
- Feels like
- Humidity
- Wind speed
- Rainfall (**1 hour / 24 hour**)
- Daily high / low
- Pressure
- Sunrise / sunset
- Current conditions

### Smart Delay Logic

Supports:

- Rain delay from weather data or rain sensor
- Wind delay with configurable threshold
- Rain cooldown timer
- 24-hour rainfall threshold limits
- Rolling rainfall totals for **1 hour** and **24 hours**

---

## 📊 Dashboard

The dashboard includes:

- **Tank level (%)**
- **Water source mode**
  - Auto: Tank
  - Auto: Mains
  - Forced mode
- **Live weather snapshot**
- **Next watering event**
  - Zone
  - Start time
  - Duration
  - ETA
- **Delay status**
  - Rain / wind block reason
- **Zone cards**
  - Progress bars
  - Manual On / Off controls

---

## 🔌 Hardware and I/O

### KC868 Support

- PCF8574 expanders:
  - `0x24` for relays
  - `0x22` for inputs
- Automatic I²C relay detection
- Falls back to normal GPIO mode if KC868 hardware is not detected

### Configurable I/O

Pins can be assigned for:

- Zone outputs
- Tank valve
- Mains valve
- Rain sensor
- Tank level sensor
- Output polarity

Changes are applied after reboot.

### Optional Display Support

- SPI TFT
- I²C OLED
- I²C LCD

---

## 🌐 Networking

- **WiFiManager captive portal**
  - AP name: `ESPIrrigationAP`
- **mDNS local access**
  - `http://espirrigation.local/`
- **OTA hostname**
  - `ESP32-Irrigation`

### Event Logging

- CSV format
- Includes weather snapshot for each event
- Downloadable through the web UI

---

## ⚙ Safety Logic

Watering is cancelled and logged if blocked by:

- Rain delay
- Wind delay
- Master off
- After Rain Cooldown period

### Rain Delay

Scheduled watering is cancelled when:

- It is currently raining
- Rainfall threshold has been reached
- After Rain cooldown is active.

- Manual zone activation can be used at all times.

### Wind Delay

If wind speed rises above the configured threshold, watering is delayed until conditions return to normal.

---

## 📦 Requirements

- Reliable **Wi-Fi connection**
- **ESP32**, **ESP32-S3**, or **KC868-A6/A8**
- Optional **240×320 TFT**, **OLED**, or **LCD display**
- **1–16 relay outputs** if not using KC868 hardware
- **Tank level sensor**
  - **0–3.3 V analog output**
- **Solenoid power supply**
  - Approx. **10 W per solenoid**
  - Typically **12 V DC** or **12/24 V AC**

---

## 🧰 Typical Materials

- KC868-A6 or ESP32 development board + relay module
- 1–16 irrigation solenoids
- 7-core irrigation cable
- Tank level sensor
- External solenoid power supply

---

## 🔧 Typical Wiring

- Solenoid returns → **GND / COM**
- **12/24 V** supply → relay **COM**
- Solenoid active lead → relay **N.O.**
- Relays 1–4 → Zones 1–4
- Relay 5 → Mains valve
- Relay 6 → Tank valve
- Tank sensor → **IO36**
- Rain sensor → **IO27** *(configurable)*

> **Important:** Tank sensor input must not exceed **3.3 V**

---

## 🚀 Flashing and Setup

### Arduino IDE

Add this board manager URL:

https://dl.espressif.com/dl/package_esp32_index.json

Install **ESP32 by Espressif Systems**.

Recommended boards:

- **ESP32 Dev Module**
- **ESP32S3 Dev Module**

Recommended partition scheme:

- **Large APP (4MB)**

### KC868 Library

If you are using KC868 hardware, install the **Kincony PCF8574** library.

---

## 📡 First-Time Wi-Fi Setup

1. Connect to **ESPIrrigationAP**
2. Open **http://192.168.4.1**
3. Enter your Wi-Fi credentials
4. Wait for the controller to reboot
5. Open **espirrigation.local** or the assigned IP address

Then configure:

- **Timezone**
- **Weather location**
- **GPS coordinates**
- **Zone count**
- **Pin assignments**
- **Other system options**

---

## 🌍 Web Endpoints

| Path | Description |
|---|---|
| `/` | Dashboard |
| `/setup` | Configuration |
| `/status` | JSON status |
| `/events` | Event log |
| `/tank` | Tank calibration |
| `/download/events.csv` | Download event log |
| `/i2c-test` | Relay test |
| `/stopall` | Stop all zones |
| `/valve/on/<z>` | Start zone |
| `/valve/off/<z>` | Stop zone |
| `/reboot` | Reboot controller |

---

## 📸 Screenshots

### Main Dashboard

<img width="439" height="870" alt="Main Dashboard" src="https://github.com/user-attachments/assets/33c5237b-4b2d-442d-9c54-6c1d7b5c92db" />

### ESP32-S3 + 6 Zone + 24V AC

<img width="484" height="457" alt="ESP32-S3 6 Zone 24V AC" src="https://github.com/user-attachments/assets/f9d2c234-1e8a-4ff2-8e8d-51a5bdf73905" />
