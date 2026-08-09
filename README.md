![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S3-blue)
![Zones](https://img.shields.io/badge/Zones-1%E2%80%9316-green)
![Web UI](https://img.shields.io/badge/Web%20UI-Local-orange)
![Weather](https://img.shields.io/badge/Weather-Aware-success)
![OTA](https://img.shields.io/badge/Updates-OTA-informational)
![MQTT](https://img.shields.io/badge/MQTT-Supported-purple)

# 🌱 ESP32 DIY 1–16 Zone Irrigation Controller

A configurable **ESP32-based irrigation controller** for managing **1–16 irrigation zones/solenoid valves** with scheduling, weather-aware watering delays, optional tank/mains switching, moisture and environmental sensing, MQTT and an easy-to-use local web dashboard.

Designed for real-world irrigation systems ranging from small home gardens to larger multi-zone installations.

The controller runs its watering logic locally, so configured schedules can continue operating even if the Internet connection is unavailable.

---

# 🚀 Flash Directly From Your Browser

### No Arduino IDE required

Use the ESP Web Flasher to install supported firmware directly over USB:

### 👉 [Open ESP32 Irrigation Web Flasher](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)

<p align="center">
<img width="400" alt="ESP32 Irrigation Web Flasher" src="https://github.com/user-attachments/assets/50a83c68-66b7-4adb-b288-8f777f0b0462" />
</p>

After flashing:

1. Connect to the Wi-Fi network **ESPIrrigationAP**
2. Open `http://192.168.4.1`
3. Enter your Wi-Fi details
4. Allow the controller to reboot
5. Open `http://espirrigation.local`
6. Configure your zones, GPIO pins, weather location and schedules

---

# 📸 Screenshots

## Main Dashboard

The dashboard provides an overview of watering status, weather conditions, tank level, active delays and upcoming watering events.

<p align="center">
<img width="650" alt="ESP32 Irrigation Main Dashboard" src="https://github.com/user-attachments/assets/ab7c199f-7b89-4def-9b42-57d2f057a21e" />
</p>

---

## ⚙️ Setup

Configure zones, schedules, GPIO assignments, weather options, sensors and other controller settings directly from the browser.

<p align="center">
<img width="350" alt="ESP32 Irrigation Setup" src="https://github.com/user-attachments/assets/ebdb1294-a135-4ca0-a4f9-31c44ddc50de" />
</p>

---

## 📋 Events

Watering activity and system events can be viewed from the controller and exported as CSV.

<p align="center">
<img width="500" alt="ESP32 Irrigation Event Log" src="https://github.com/user-attachments/assets/46bd8e74-fc39-48e6-be24-0bccc911c33f" />
</p>

---

# 🔌 Wiring Diagrams

## Controller Wiring Diagram

<p align="center">
<img width="1000" alt="ESP32 Irrigation Controller Wiring Diagram" src="https://github.com/user-attachments/assets/df3b2d56-5021-43df-a429-55fd85ebc625" />
</p>

---

## ESP32-S3 + 6 Relay + 24 V AC Example

Example six-zone ESP32-S3 irrigation controller using a relay board and 24 V AC irrigation transformer.

<p align="center">
<img width="650" alt="ESP32-S3 6 Zone 24V AC Irrigation Controller" src="https://github.com/user-attachments/assets/f9d2c234-1e8a-4ff2-8e8d-51a5bdf73905" />
</p>

### Example Hardware Build

<p align="center">
<img width="450" alt="ESP32 Irrigation Controller Hardware Build" src="https://github.com/user-attachments/assets/2caa1f7a-3e10-4306-ac2d-97c6a0ce8309" />
</p>

> ⚠️ **Important:** Wiring varies between ESP32 boards, relay modules and irrigation systems. Always verify the GPIO assignments and relay voltage requirements before applying power.

---

# ✨ Features

* **1–16 irrigation zones**
* **Two independent start times per zone**
* **7-day scheduling**
* **Minute + second runtime control**
* **Sequential or concurrent zone operation**
* **Editable zone names**
* Optional **tank ↔ mains water source switching**
* Optional **rain-aware watering delays**
* Optional **wind-aware watering delays**
* **After-rain cooldown period**
* **24-hour rainfall threshold**
* Optional **Cool / Hot / Very Hot runtime adjustment**
* Optional **soil moisture probe**
* Optional **temperature / humidity sensor**
* **Live weather information**
* **Tank level monitoring**
* **Next watering event calculation**
* **Manual zone control**
* **Master watering control**
* **Local web dashboard**
* **Event logging**
* **CSV event export**
* **OTA firmware updates**
* **MQTT support**
* Optional **TFT display**
* Optional **I²C OLED**
* Optional **I²C LCD**
* Compatible with:

  * **ESP32**
  * **ESP32-S3**
  * **KC868-A6**
  * **KC868-A8**
* Watering schedules continue to run locally if Internet access is lost

---

# ⚡ Quick Start

## 1. Flash the Controller

The easiest method is the browser-based Web Flasher:

### 👉 [ESP32 Irrigation Web Flasher](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)

Alternatively, compile and upload the firmware using the Arduino IDE.

---

## 2. Connect to the Setup Wi-Fi

After first boot, connect your phone or computer to:

```text
ESPIrrigationAP
```

Then open:

```text
http://192.168.4.1
```

Enter your Wi-Fi credentials and save.

---

## 3. Open the Controller

Once connected to your home network, open:

```text
http://espirrigation.local
```

or use the controller's assigned IP address.

---

## 4. Configure the System

Configure:

* Timezone
* Latitude / longitude
* Number of irrigation zones
* Zone names
* GPIO assignments
* Relay polarity
* Watering days
* Start times
* Zone durations
* Rain delay
* Wind delay
* Temperature adjustment
* Moisture sensor
* Tank monitoring
* Tank / mains switching
* MQTT
* Display options

---

## 5. Test Outputs

Before enabling automatic schedules, manually test each zone from the web interface and confirm that the correct valve operates.

---

# ⏱ Scheduling

Each irrigation zone can be individually configured.

Supported options include:

* **1–16 zones**
* **Two start times per zone**
* Independent duration for the second start time
* **7-day scheduling**
* Minute and second runtime precision
* Editable zone names

Zones can operate:

### Sequentially

This is the default and recommended mode for most irrigation systems.

Only one irrigation zone operates at a time.

### Concurrently

Multiple zones may run simultaneously if the:

* Transformer
* Power supply
* Relay system
* Pipework
* Water source

can safely support the combined load.

---

# 🌦 Weather Integration

The controller supports weather-aware irrigation using **Open-Meteo**.

Enter your location's:

* Latitude
* Longitude

in the **Setup** page.

---

## 🌡 Live Weather Information

Depending on enabled options and sensors, the controller can display:

* Temperature
* Feels-like temperature
* Humidity
* Wind speed
* Rainfall
* 1-hour rainfall
* 24-hour rainfall
* Daily minimum temperature
* Daily maximum temperature
* Atmospheric pressure
* Sunrise
* Sunset
* Current weather conditions
* Soil moisture

---

# 🌧 Smart Watering Delay Logic

The controller can automatically stop or adjust scheduled watering according to environmental conditions.

## Rain Delay

Scheduled watering can be cancelled when:

* Rain is currently detected
* Weather data reports rain
* A physical rain sensor is active
* The configured rainfall threshold is exceeded
* The after-rain cooldown period is active

Manual watering remains available when required.

---

## Rain Cooldown

After rain has stopped, the controller can prevent scheduled watering for a configurable period.

This helps prevent watering immediately after useful rainfall.

---

## 24-Hour Rainfall

The controller can track rainfall totals and prevent irrigation when the configured rainfall threshold is reached.

Displayed rainfall information can include:

* Last 1 hour
* Last 24 hours

---

## 💨 Wind Delay

If wind speed exceeds the configured threshold, scheduled watering can be prevented until conditions return to normal.

This is especially useful for sprinkler and pop-up irrigation zones where strong wind can cause poor water distribution.

---

# 🌡 Temperature / Moisture Runtime Adjustment

Optional environmental runtime adjustment can modify watering duration according to configured conditions.

Modes can include:

* **Cool**
* **Normal**
* **Hot**
* **Very Hot**

Optional inputs include:

* Weather temperature
* Soil moisture probe
* Temperature / humidity sensor

These features can be enabled or disabled depending on the installation.

---

# 💧 Tank and Mains Water Control

The controller can optionally manage two water sources:

* **Rainwater tank**
* **Mains water**

Possible modes include:

* **Auto: Tank**
* **Auto: Mains**
* **Force Tank**
* **Force Mains**

A tank level sensor can be used to automatically select the appropriate source.

---

# 📊 Dashboard

The local dashboard can display:

### System

* Controller status
* Wi-Fi signal
* Uptime
* Master watering state

### Water

* Tank level %
* Current water source
* Tank / mains mode

### Weather

* Current temperature
* Humidity
* Wind
* Rainfall
* Daily high / low
* Sunrise / sunset

### Watering

* Current zone
* Zone progress
* Next zone
* Next start time
* Runtime
* ETA

### Delays

* Rain status
* Wind status
* Rain cooldown
* Delay/block reason

### Zones

Each zone has its own card with:

* Zone name
* Current status
* Progress
* Manual **ON**
* Manual **OFF**

---

# 🖥 Supported Hardware

| Hardware                    | Description                                          |
| --------------------------- | ---------------------------------------------------- |
| **ESP32**                   | Standard ESP32 development-board installation        |
| **ESP32-S3**                | Newer ESP32-S3 based controller                      |
| **ESP32 + 240×320 SPI TFT** | Full-colour status display                           |
| **ESP32 + I²C OLED**        | Compact low-pin-count display                        |
| **ESP8266 + I²C LCD**       | Lightweight LCD controller version                   |
| **KC868-A6**                | ESP32 relay-controller hardware with screw terminals |
| **KC868-A8**                | Eight-relay KC868 controller option                  |

---

# 💡 Optional Display Power Saving

For installations with a display inside an enclosure, a photoresistor can optionally be used for automatic backlight control.

For example:

* Door open → display on
* Door closed / dark → display dimmed or off

This reduces unnecessary display power consumption and backlight wear.

---

# 🔌 Hardware and I/O

## Configurable GPIO

GPIO pins can be assigned for:

* Zone outputs
* Tank valve
* Mains valve
* Rain sensor
* Tank level sensor
* Soil moisture sensor
* Temperature / humidity sensor
* Display
* Output polarity

> GPIO availability differs between **ESP32**, **ESP32-S3**, KC868 and other boards. Check your board's pinout before selecting GPIO numbers.

Changes to GPIO configuration may require a reboot before they take effect.

---

# 🧠 KC868 Support

Supported KC868 configurations use PCF8574 I/O expanders.

Default addresses include:

```text
0x24 = Relay outputs
0x22 = Inputs
```

Features include:

* Automatic I²C relay detection
* PCF8574 relay control
* Screw-terminal I/O
* GPIO fallback where supported
* Configurable output polarity

---

# 🔴 Relay Polarity

Relay modules may be:

```text
Active HIGH
```

or:

```text
Active LOW
```

The controller allows output polarity to be configured.

This is important because many ESP32 relay modules are **LOW = ON**.

Always test relay behaviour before connecting irrigation valves.

---

# ⚠️ Solenoid Protection

Irrigation valves are inductive loads and can generate voltage spikes when switched.

These spikes can cause:

* ESP32 resets
* Relay interference
* Wi-Fi instability
* Unexpected reboots
* Electrical noise

The correct protection depends on whether the valve uses **DC or AC**.



# ⚠️ Sensor Input Voltage

Sensor outputs connected directly to an ESP32 GPIO must remain within the safe input range.

### Do not apply more than 3.3 V directly to an ESP32 GPIO.

For sensors with:

* 5 V output
* 10 V output
* 0–10 V output

use an appropriate:

* Voltage divider
* Signal-conditioning circuit
* Level converter

before connecting the signal to the ESP32.

---

# 📦 Requirements

Typical installation requirements include:

* **ESP32**
* **ESP32-S3**
* or **KC868-A6/A8**
* 1–16 relay outputs
* Irrigation solenoid valves
* Suitable valve transformer / power supply
* Wi-Fi for initial configuration and network features

Optional hardware:

* Tank level sensor
* Rain sensor
* Soil moisture sensor
* Temperature / humidity sensor
* TFT display
* OLED display
* LCD display
* Tank valve
* Mains valve

---

# 🧰 Typical Materials

A basic build may use:

* ESP32 or ESP32-S3 development board
* 4 / 6 / 8 / 16-channel relay module
* 1–16 irrigation solenoid valves
* Irrigation multicore cable
* 24 V AC irrigation transformer
* Weatherproof enclosure
* Terminal blocks
* Suitable fuse protection

Optional:

* Tank level sensor
* Rain sensor
* Soil moisture probe
* Temperature / humidity sensor
* TFT / OLED / LCD display

---

# ⚡ Power Supply

Irrigation solenoids commonly use:

* **12 V DC**
* **24 V DC**
* **12 V AC**
* **24 V AC**

Select the transformer or power supply according to the actual valve specifications.

Allow sufficient capacity if operating multiple valves at the same time.

---

# 📡 First-Time Wi-Fi Setup

When no Wi-Fi credentials are stored, the controller starts a WiFiManager access point.

### Access Point

```text
ESPIrrigationAP
```

### Setup address

```text
http://192.168.4.1
```

### Steps

1. Power the controller
2. Connect to `ESPIrrigationAP`
3. Open `http://192.168.4.1`
4. Select your Wi-Fi network
5. Enter the Wi-Fi password
6. Save
7. Wait for the ESP32 to reboot

Then open:

```text
http://espirrigation.local
```

or use the IP address assigned by your router.

---

# 🌐 Networking

Supported networking features include:

### WiFiManager

```text
AP: ESPIrrigationAP
```

### mDNS

```text
http://espirrigation.local/
```

### OTA hostname

```text
ESP32-Irrigation
```

### MQTT

MQTT can be used to integrate the controller with systems such as:

* Home Assistant
* Node-RED
* Custom MQTT dashboards
* Other home-automation systems

---

# 📋 Event Logging

Controller events can be logged locally.

Logs may include:

* Zone start
* Zone stop
* Manual watering
* Scheduled watering
* Rain delay
* Wind delay
* Rain cooldown
* Master off
* Weather information

Logs can be exported in:

```text
CSV
```

format through the web interface.

---

# ⚙️ Safety Logic

Scheduled watering can be cancelled and logged when blocked by:

* Rain
* Rainfall threshold
* Rain cooldown
* Excessive wind
* Master OFF
* System pause

Manual controls remain available where permitted by the controller configuration.

---

# 🔄 OTA Firmware Updates

OTA allows firmware updates over the local network without reconnecting the ESP32 to USB.

The OTA hostname is:

```text
ESP32-Irrigation
```

> An **OTA-capable partition scheme must be selected** if OTA updates are required.

---

# 🛠 Arduino IDE Installation

If you prefer to compile the firmware manually, install the ESP32 Arduino core.

## Board Manager URL

Add:

```text
https://dl.espressif.com/dl/package_esp32_index.json
```

to Arduino IDE's **Additional Boards Manager URLs**.

Then install:

```text
ESP32 by Espressif Systems
```

Recommended board selections include:

```text
ESP32 Dev Module
```

or:

```text
ESP32S3 Dev Module
```

depending on your hardware.

---

# 💾 Partition Scheme

The required partition scheme depends on the firmware build and whether OTA is enabled.

### Using OTA

Choose a partition layout that contains OTA application partitions.

### Not Using OTA

Larger single-app layouts such as **Huge APP** or **No OTA** may provide more application/storage space.

> ⚠️ Do **not** select a **No OTA** partition if you intend to use OTA firmware updates.

---

# 🧩 KC868 Library

When compiling manually for KC868 hardware, install the required Kincony PCF8574 library:

[Kincony PCF8574 Library](https://www.kincony.com/forum/attachment.php?aid=1697)

---

# 🌍 Web Endpoints

| Path                   | Description              |
| ---------------------- | ------------------------ |
| `/`                    | Main dashboard           |
| `/setup`               | Controller configuration |
| `/status`              | JSON system status       |
| `/events`              | Event log                |
| `/tank`                | Tank sensor calibration  |
| `/download/events.csv` | Download event log       |
| `/i2c-test`            | I²C / relay test         |
| `/stopall`             | Stop all active zones    |
| `/valve/on/<z>`        | Manually start a zone    |
| `/valve/off/<z>`       | Manually stop a zone     |
| `/reboot`              | Reboot the controller    |

---

# 🛠 Troubleshooting

## ESP32 Resets When a Valve Turns Off

Possible causes include electrical noise from the solenoid or relay wiring.

For **DC valves**:

* Install a correctly orientated flyback diode across the valve coil
* Check power supply stability
* Keep solenoid wiring away from ESP32 signal wiring
* Ensure grounds are correctly connected where required

For **AC valves**:

* Do not use a normal DC flyback diode across the valve
* Consider an appropriately rated MOV or RC snubber
* Check transformer capacity
* Check relay contact ratings

---

## Relay Works Backwards

Your relay board may be **active LOW**.

Open **Setup** and change the output polarity.

---

## `espirrigation.local` Does Not Open

Try opening the controller using its IP address instead.

Check your router's connected-device list to find the ESP32 IP address.

---

## Weather Is Not Updating

Check:

* Internet connection
* Latitude
* Longitude
* Timezone
* DNS/network access

Local watering schedules can continue independently of live weather updates.

---

## Wrong Valve Activates

Check:

* GPIO assignments
* Zone numbering
* Relay channel wiring
* Active HIGH / LOW setting

Always manually test every zone before enabling schedules.

---

# ⚠️ Electrical Safety

This project commonly switches **low-voltage irrigation circuits** such as 12 V or 24 V AC/DC.

If a 230/240 V mains-powered transformer is used, mains wiring must be:

* Properly insulated
* Enclosed
* Fused/protected
* Installed according to applicable electrical requirements

Where required, mains-voltage installation should be completed by a suitably qualified person.

Keep mains-voltage wiring physically separated from:

* ESP32 wiring
* Sensor wiring
* Low-voltage irrigation wiring

---

# 🌱 Typical Applications

The controller can be used for:

* Lawn irrigation
* Garden beds
* Drip irrigation
* Vegetable gardens
* Greenhouses
* Rainwater tank irrigation
* Multi-zone residential systems
* Automated garden watering

---

# 💚 Project Goals

The ESP32 Irrigation Controller is designed to provide a flexible DIY alternative to commercial irrigation controllers while keeping the important control logic on the device itself.

The system can be expanded from a simple single-zone controller to a larger installation with:

* 16 irrigation zones
* Weather integration
* Tank management
* Moisture sensing
* Environmental monitoring
* MQTT
* Displays
* Local automation

---

# 🔗 Useful Links

### 🌐 Web Flasher

[Flash ESP32 Irrigation Controller](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)

### 💻 GitHub Repository

[ESP32-Irrigation-Controller](https://github.com/numerik11/ESP32-Irrigation-Controller)

---

## ⭐ Support the Project

If you find the project useful, consider giving the repository a **⭐ Star** on GitHub.

Bug reports, testing and suggestions welcome.

Beau.
