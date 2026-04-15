![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S3-blue)
![Zones](https://img.shields.io/badge/Zones-1%E2%80%9316-green)
![Web UI](https://img.shields.io/badge/Web%20UI-Local-orange)
![Weather](https://img.shields.io/badge/Weather-Aware-success)
![OTA](https://img.shields.io/badge/Updates-OTA-informational)

# 🌱 ESP32 1–16 Zone Irrigation Controller
---

A feature-rich ESP32-based irrigation controller for managing from 1–16 irrigation zones/solenoids, with optional automatic tank/mains water source switching, live weather-aware control, and a local web interface.

Built for real-world garden and irrigation installations, from small home systems through to larger multi-zone setups.
---

✨ Features
Supports 1–16 irrigation zones
Optional automatic tank ↔ mains switching
Optional rain and wind-aware scheduling
Modern web-based control interface
Worldwide timezone support
Live weather display
Optional TFT, OLED, or LCD display support
Compatible with ESP32, ESP32-S3, and KC868-A6/A8
Watering logic continues to run locally if Wi-Fi disconnects, and will automatically reconnect when available
---

⏱ Scheduling
1–16 configurable zones
Two start times per zone
Optional second start time with its own duration
7-day scheduling
Minute and second precision
Run zones:
Sequentially (default)
All at once where power supply capacity allows
Editable zone names
Stored directly on the controller
---

🖥 Supported Hardware Variants

External display showing:
System status
Active zones
Rain and wind delays
Water source status
ESP32 + I²C OLED
ESP32 + 240×320 SPI TFT
Add photoreststor for Power saving

Compact, low-pin-count display option ideal for smaller builds.

ESP8266 + I²C LCD

Lightweight version using a standard 16×2 LCD.

🌦 Weather Integration

---
Use Open-Meteo to obtain your latitude and longitude:

Visit Open-Meteo
Enter your location coordinates in Setup
Save settings
Live Weather Data

Displays:

Temperature
Feels like
Humidity
Wind speed
Rainfall (1 hour / 24 hour)
Daily high / low
Pressure
Sunrise / sunset
Current conditions (rain, clear, thunderstorm, etc.)
Smart Delay Logic

Supports:

Rain delay from weather data or rain sensor
Wind delay with configurable speed threshold
Rain cooldown timer
24-hour rainfall threshold limits
Rolling rainfall totals for 1 hour and 24 hours
---

📊 Dashboard

The dashboard provides:

Tank level (%)
Water source mode
Auto: Tank
Auto: Mains
Forced mode
Live weather snapshot
Next watering event
Zone
Start time
Duration
ETA
Delay status
Rain / wind blocking reason
Zone control cards
Progress bars
Manual On / Off control
🔌 Hardware and I/O
KC868-A6/A8 support
PCF8574 expanders:
0x24 for relays
0x22 for inputs
Automatic I²C relay detection
Falls back to standard GPIO mode if KC868 hardware is not detected
Configurable pins
Zone outputs
Tank and mains valves
Sensors
Output polarity
Changes are applied after reboot
Optional display support:
SPI TFT
I²C OLED
Optional backlight control:
Photoresistor + 100k resistor
Can turn screen off when enclosure door is closed
🌐 Networking and User Experience
WiFiManager captive portal
AP name: ESPIrrigationAP
mDNS local access
http://espirrigation.local/
OTA firmware updates
Hostname: ESP32-Irrigation
Event logging
CSV format
Includes weather snapshot for each event
Downloadable through the web UI
---

⚙ Behaviour and Safety Logic

Watering is cancelled and logged if blocked by:

Rain delay
Wind delay
Master off
Cooldown period

Manual zone activation follows the same safety rules.

Rain Delay

Scheduled watering is cancelled when rain conditions are active, such as:

It is currently raining
Rainfall threshold has been reached
Rain cooldown is active

Watering remains delayed for the configured time period.

Wind Delay

If wind speed rises above the configured threshold, watering is delayed and will resume automatically once wind speed drops below the threshold.
---

📦 Requirements
Reliable Wi-Fi connection
ESP32 board
ESP32
ESP32-S3
KC868-A6/A8 recommended
Optional 240×320 TFT with backlight pin or OLED display
1–16 relay outputs if not using KC868 hardware
Tank level sensor
0–3.3 V analog output
Solenoid power supply
Approx. 10 W per solenoid
Typically 12 V DC or 12/24 V AC
---

🧰 Typical Materials
KC868-A6 or ESP32 development board + relay module
1–16 irrigation solenoids
7-core irrigation cable
Tank level sensor
External solenoid power supply
---

🔧 Typical Wiring
Tie all solenoid returns to supply GND / COM
Feed 12/24 V into each relay COM
Solenoid active lead → relay N.O.
Relays 1–4 → Zones 1–4
Relay 5 → Mains valve
Relay 6 → Tank valve
Tank sensor → IO36 (A1)
Important: must not exceed 3.3 V
Rain sensor → IO27
(Configurable)
---

🚀 Flashing and Setup
Arduino IDE

Add the ESP32 board package URL:

https://dl.espressif.com/dl/package_esp32_index.json

Install:

ESP32 by Espressif Systems

Recommended board selections:

ESP32 Dev Module
ESP32S3 Dev Module

Recommended partition scheme:

Large APP (4MB)
KC868-A Library

Download the Kincony PCF8574 library from the Kincony forum.
---

📡 First-Time Wi-Fi Setup

1. Connect WiFi to ESPIrrigation

2. Open http://192.168.4.1

3. Enter your Wi-Fi credentials

4. The controller will reboot and join your network

5. Open espirrigation.local or use the IP address shown at startup

In Setup, configure:
Timezone
Weather location
GPS coordinates
Zone count
Pin assignments
Other system options
---

🌍 Web Endpoints
Path	Description
/	Dashboard
/setup	Configuration
/status	JSON status
/events	Event log
/tank	Tank calibration
/download/events.csv	Download event log
/i2c-test	Relay test
/stopall	Stop all zones
/valve/on/<z>	Start zone
/valve/off/<z>	Stop zone
/reboot	Reboot controller
---

## Screenshots

### Main Dashboard
<img width="439" height="870" alt="image" src="https://github.com/user-attachments/assets/33c5237b-4b2d-442d-9c54-6c1d7b5c92db" />

### ESP32-S3 + 6 Zone + 24V AC
<img width="484" height="457" alt="image" src="https://github.com/user-attachments/assets/f9d2c234-1e8a-4ff2-8e8d-51a5bdf73905" />

---

## Notes

- Sequential mode is recommended unless your power supply can handle multiple solenoids at once
- Local IP access may be more reliable than mDNS on some networks
