![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S3-blue)
![Zones](https://img.shields.io/badge/Zones-1%E2%80%9316-green)
![Web UI](https://img.shields.io/badge/Web%20UI-Local-orange)
![Weather](https://img.shields.io/badge/Weather-Aware-success)
![OTA](https://img.shields.io/badge/Updates-OTA-informational)
![MQTT](https://img.shields.io/badge/MQTT-Supported-purple)

# ESP32 Irrigation Controller

Control **1–16 watering zones** from your phone, tablet or computer. Create schedules, turn valves on and off, and adjust watering to suit the weather.

[Install Firmware](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/) · [Android App](https://github.com/numerik11/ESP32-Irrigation-Controller/releases/tag/AndroidAPK) 

## What can it do?

- Set watering days and up to **two start times per zone**.
- Start, stop or pause watering from your browser.
- Adjust watering for temperature and optional soil-moisture readings.
- Delay watering during rain, strong winds or selected no-watering times.
- Monitor an optional rainwater tank and switch between tank and mains water.
- Connect to **Home Assistant using MQTT**.
- Support optional displays and firmware updates over Wi-Fi.

Schedules run on the controller without internet once programed.

## Using the controller

### Dashboard

See which zones are running, what is scheduled next, current weather and any watering delays. Start or stop watering from here.

![ESP32 irrigation dashboard](<img width="859" height="909" alt="image" src="https://github.com/user-attachments/assets/a5c6dad5-7141-4d3a-ac56-83265e99e835" />)

### Setup

Change your watering schedules, outputs, weather rules and optional features.

![Irrigation controller setup page](https://github.com/user-attachments/assets/446509b8-eeec-40c7-93b4-62bde1730a58)

### Events

Review watering activity and find out why a run was delayed or skipped. You can also download the log as a CSV file.

![Irrigation event log](https://github.com/user-attachments/assets/1bc1d7e9-1449-4f20-a51b-a0eaa4b347e2)

## Watering schedules

Each zone supports selected watering days and **two start times**, each with its own duration.

Choose how zones run:

- **Sequential:** One zone at a time. Suitable for most systems.
- **Concurrent:** Multiple zones together. Your water supply and electrical equipment must support the combined load.

### Smart Watering

Smart Watering adjusts watering duration using your temperature settings and optional moisture sensor.

To let watering shorten without a minimum limit, set **Minimum Adjusted Runtime** to **0**. A higher value prevents watering from shortening below that setting.

### Weather delays

Enter your latitude and longitude to enable online weather information. You can configure delays for rain, recent rainfall, after-rain cooldown and strong winds.

Check the **Dashboard** or **Events** to see why watering has been delayed.

### No-watering periods

Under **Setup → No-Watering Periods**, add up to three times when automatic watering is blocked.

For example, block watering on weekends from **11:00 am to 5:00 pm**. Overnight periods are also supported.

Automatic watering stops during these periods, and affected runs are cancelled. Manual watering remains available.

## Optional tank and mains control

Connect a compatible tank-level sensor to monitor your water supply. With the required valve outputs, the controller can select tank or mains water automatically, or let you choose manually.

Use the **tank calibration page** to set the sensor’s empty and full readings.

## Getting started

### 1. Install the firmware

Connect your board to your computer by USB and open the **[Web Flasher](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)**.

Choose the firmware for your board and follow the instructions. No Arduino IDE is needed.

### 2. Connect to your Wi-Fi

1. Join the **ESPIrrigationAP** Wi-Fi network from your phone or computer.
2. If it says “no internet”, choose to stay connected.
3. Open **http://192.168.4.1**.
4. Select your home Wi-Fi and enter its password.
5. Once setup finishes, reconnect your phone or computer to your home Wi-Fi.

The address `192.168.4.1` is only used during setup.

### 3. Open the controller

Use the **[Android app](https://github.com/numerik11/ESP32-Irrigation-Controller/releases/tag/AndroidAPK)** or open **http://espirrigation.local** while connected to the same network.

If that address does not work, use the controller’s IP address from the app or your router’s connected-device list.

### 4. Set up your zones

Open **Setup** and enter:

- Your zone names, output pins and relay polarity.
- Your timezone and correct local time.
- Watering days, start times and durations.
- Any optional weather, sensor, tank or display settings.

**Save your settings and manually test each valve before enabling automatic watering.**

## Hardware and wiring

Choose firmware that matches your hardware. Options include:

| Hardware | Use |
| --- | --- |
| ESP32 / ESP32-S3 | Standard controller |
| ESP32 with TFT or OLED | Controller with a local display |
| KC868-A6 / A8 | Controller with built-in relays |
| ESP8266 with I²C LCD | Separate lightweight firmware |

You will also need suitable relays, irrigation valves, a valve-rated power supply, wiring and a protective enclosure.

### Wiring example

Check all connections against your board and components before applying power.

![ESP32 irrigation controller wiring diagram](https://github.com/user-attachments/assets/df3b2d56-5021-43df-a429-55fd85ebc625)

### ESP32-S3 with six relays and 24 V AC valves

![ESP32-S3 six-zone wiring example](https://github.com/user-attachments/assets/f9d2c234-1e8a-4ff2-8e8d-51a5bdf73905)

### Optional TFT display

![TFT controller status display](https://github.com/user-attachments/assets/42fbe90b-aeb4-48c1-9cfa-cd5753487815)

### Wiring essentials

- Match the power supply’s voltage and AC/DC type to your valves.
- Never apply more than **3.3 V** to an ESP32 GPIO.
- Set relay polarity to match your module: **active HIGH** or **active LOW**.
- Use suitable surge protection: a flyback diode for DC valves or a rated MOV/RC snubber for AC valves. Never place a flyback diode across an AC valve.
- Keep valve wiring separate from sensor wiring.
- Protect mains connections and use a qualified person where required.

## Home Assistant

Enable **MQTT** in Setup and enter your broker details. The controller supports Home Assistant discovery for zone switches and selected status sensors.

To display today’s planned watering in a Home Assistant Webpage card:

```yaml
type: iframe
url: http://espirrigation.local/schedule-html
aspect_ratio: 60%
```

You can also open that address directly in your browser. Replace the hostname with the controller’s IP address if needed.

The schedule shows estimated watering times. Weather delays and pauses can change what actually runs. Check **Events** for watering history.

## Updating the firmware

For installations with browser update support:

1. Open **Setup → Firmware Updates** and set an OTA password of **8–64 characters**.
2. Open `http://<controller-ip>/update`.
3. Sign in with username **admin** and your OTA password.
4. Select the matching **application `.bin`** file and upload it.
5. Keep the controller powered until it finishes and restarts.

Use an application binary, not a merged full-flash image.

Older installations may need a USB update first to enable Wi-Fi updates. **Back up your configuration and schedules before erasing, as erasing removes saved settings.**

## Building with Arduino IDE

If you prefer to compile the firmware yourself:

1. Install **ESP32 by Espressif Systems** in Boards Manager.
2. Open the [ESP32 sketch](firmware/ESP32-Irrigation/ESP32-Irrigation.ino).
3. Select **ESP32 Dev Module** or **ESP32S3 Dev Module** for your board.
4. Choose **Minimal SPIFFS (Large APPS with OTA)** to support Wi-Fi updates.
5. Install the libraries required by your hardware, then compile and upload.

KC868 builds also require the [Kincony PCF8574 library](https://www.kincony.com/forum/attachment.php?aid=1697).

The [ESP8266 firmware](firmware/ESP8266-Irrigation/ESP8266-Irrigation.ino) is a separate version and uses different board settings.

## Troubleshooting

| Problem | What to check |
| --- | --- |
| Dashboard will not open | Connect to the same Wi-Fi and try the controller’s IP address. |
| Scheduled watering does not start | Check local time, watering days, master control, pause and weather delays. |
| Wrong valve turns on | Check output pin assignments and relay polarity. |
| Smart Watering does not shorten a run | Check **Minimum Adjusted Runtime**. |
| Browser update page is unavailable | Your installation may need a USB update first. |

## Help and feedback

Visit the [GitHub project](https://github.com/numerik11/ESP32-Irrigation-Controller) to report a bug or suggest an improvement.

If you find the controller useful, consider giving the project a star.

Beau
