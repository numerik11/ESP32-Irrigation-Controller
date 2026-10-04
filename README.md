![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S3-blue)
![Zones](https://img.shields.io/badge/Zones-1%E2%80%9316-green)
![Web UI](https://img.shields.io/badge/Web%20UI-Local-orange)
![Weather](https://img.shields.io/badge/Weather-Aware-success)
![OTA](https://img.shields.io/badge/Updates-OTA-informational)
![MQTT](https://img.shields.io/badge/MQTT-Supported-purple)

# ESP32 Irrigation Controller

Control **1–16 irrigation zones** from your phone, tablet or computer. Set watering schedules, operate valves manually, adjust watering for weather conditions, and monitor an optional rainwater tank—all through a local web interface.

The controller runs schedules locally. Internet access is needed for online weather updates, and the controller must have the correct local time for scheduled watering. Loss of internet access does not itself stop local schedules while the controller's clock remains valid.

**Get started:** [Install firmware](#1-install-the-firmware) · [Connect to Wi-Fi](#2-connect-the-controller-to-wi-fi) · [Android app](#using-the-android-app) · [Troubleshooting](#troubleshooting)

**Learn more:** [Scheduling](#scheduling-and-smart-watering) · [Hardware](#hardware-and-wiring) · [Home Assistant](#home-assistant-and-mqtt) · [Firmware updates](#firmware-updates) · [Web endpoints](#web-endpoints)

## What it can do

| Feature | What it provides |
| --- | --- |
| **Flexible schedules** | Named zones, selected weekdays and up to two starts per zone, each with its own runtime. |
| **Manual control** | Start and stop zones from the dashboard, with master and pause controls. |
| **Smart Watering** | Adjust runtimes for configured temperature and moisture conditions. |
| **Weather delays** | Delay watering for rain, rainfall thresholds, after-rain cooldown and wind. |
| **No-watering periods** | Up to three periods that prevent automatic watering, including overnight periods. |
| **Tank and mains support** | Optional tank-level monitoring and automatic or forced water-source selection. |
| **Local dashboard** | View active zones, upcoming watering, weather, delays and system status. |
| **Logs and diagnostics** | Review events, export CSV logs and inspect diagnostic data. |
| **Home automation** | MQTT integration, Home Assistant discovery and an embeddable daily schedule. |
| **Firmware updates** | Update supported installations through a browser over Wi-Fi. |
| **Optional displays** | TFT, I²C OLED and I²C LCD options, depending on the firmware and board. |

## Getting started

### 1. Install the firmware

Connect your board to your computer by USB and open the **[ESP32 Irrigation Web Flasher](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)**. Select the image appropriate for your hardware and follow the flasher instructions. Arduino IDE is not required for this method.

For compiling and uploading the source yourself, see [Manual installation](#manual-installation).

### 2. Connect the controller to Wi-Fi

On first setup, or when the controller enters its configuration portal:

1. On your phone or computer, join the **ESPIrrigationAP** Wi-Fi network.
2. If your device reports **no internet**, choose to stay connected.
3. Open **[http://192.168.4.1](http://192.168.4.1)**.
4. Select your normal Wi-Fi network and enter its password.
5. Allow the controller to connect and restart if required.
6. Reconnect your phone or computer to your normal Wi-Fi network.

**`192.168.4.1` is the setup address.** Once the controller joins your normal Wi-Fi, your router generally assigns it a different IP address.

### 3. Open the controls

Use the Android app, or open **[http://espirrigation.local](http://espirrigation.local)** in a browser on the same network.

If the `.local` address does not work, use the controller's IP address—for example, `http://192.168.1.113`. Find it through the Android app or your router's connected-device list. A DHCP reservation in your router can keep that address consistent.

### 4. Configure and test your installation

Open **Setup** and configure:

1. **Zones and outputs:** zone count, names, GPIO assignments and relay polarity.
2. **Time:** timezone and correct controller local time.
3. **Watering:** selected days, start times, runtimes and sequential/concurrent mode.
4. **Optional features:** weather location, delay rules, sensors, tank/mains control, display and MQTT.

Save your changes, then manually test each output before enabling automatic watering. Confirm that the intended valve opens and closes correctly.

## Using the Android app

Download the APK from the **[Android app release page](https://github.com/numerik11/ESP32-Irrigation-Controller/releases/tag/AndroidAPK)** and install it on your phone. Android may ask you to allow installation from the browser or file manager used to open the APK.

The app displays the controller's existing web interface. It does not require a separate set of schedules or controls.

### Find a controller on your normal Wi-Fi

Connect your phone to the controller's network and open the app. It checks for the setup portal, then tries the saved controller address and local hostnames. If needed, it scans the connected Wi-Fi network for a matching irrigation controller.

- A single match opens automatically; multiple scan matches let you choose a controller.
- The app remembers the normal-network IP for the next launch.
- Use **Enter IP** when you already know the address.
- Once connected, the app header and buttons hide. Press **Android Back** to reveal them, then tap the connected status text to hide them again.

Addresses such as `172.16.99.103` are supported. Discovery depends on your phone's network and subnet, not on addresses starting with `192.168`. The app scans subnets containing up to 1,024 addresses; on larger networks, it limits scanning to the phone's local `/24` range. Use **Enter IP** for a reachable controller outside that range.

### Open the setup hotspot

In app version **1.2**, the app can recognise the **ESPIrrigationAP** setup portal at `http://192.168.4.1`, even though that portal does not provide the normal irrigation status API.

1. Tap **Connect to ESPIrrigationAP / Wi-Fi settings**.
2. In Android's Wi-Fi settings, select **ESPIrrigationAP**.
3. Stay connected if Android warns that the network has no internet.
4. Return to the app. It checks again and opens the recognised setup portal.
5. After saving the controller's Wi-Fi details, reconnect your phone to your normal network and tap **Find controller**.

The app opens Wi-Fi settings; you select the network yourself. Joining the setup hotspot does not overwrite the saved normal-network controller address.

> The phone must be able to communicate with the ESP. Guest Wi-Fi or client isolation can prevent both discovery and direct IP access.

## Web interface

### Dashboard

View active and upcoming zones, watering progress, current weather, tank level, water source and reasons for delayed watering.

<p align="center">
<img width="862" height="899" alt="ESP32 irrigation dashboard" src="https://github.com/user-attachments/assets/cf75fb58-65f0-445e-abe4-f65609e4c525" />
</p>

### Setup

Configure outputs, schedules, relay polarity, weather rules, sensors, MQTT and displays.

<p align="center">
<img width="694" height="799" alt="Irrigation controller setup page" src="https://github.com/user-attachments/assets/446509b8-eeec-40c7-93b4-62bde1730a58" />
</p>

### Events

Review watering and system activity, including weather-related delays. Export the event log as CSV for further analysis.

<p align="center">
<img width="650" height="492" alt="Irrigation event log" src="https://github.com/user-attachments/assets/1bc1d7e9-1449-4f20-a51b-a0eaa4b347e2" />
</p>

## Scheduling and Smart Watering

Each zone has selected watering days and up to **two start times**, with an independent duration for each start. Runtimes can be set in minutes and seconds.

### Sequential or concurrent operation

- **Sequential:** runs one zone at a time. This suits most installations with limited water flow or power capacity.
- **Concurrent:** allows multiple zones to run together. The supply, relays, transformer and pipework must support the combined load.

### Runtime adjustments

Smart Watering adjusts scheduled runtimes using the configured rules, including Cool, Normal, Hot and Very Hot temperature settings and optional moisture input.

**Minimum Adjusted Runtime (minutes)** sets the lower limit for adjusted runtimes. Set it to **0** to allow the full Smart Watering reduction.

For example, an 11% factor applied to a five-minute schedule gives approximately 33 seconds. If the saved minimum is five minutes, that same schedule remains five minutes long. Check this setting if watering does not shorten as expected.

New configurations default to a zero minimum. Upgrades preserve your existing saved value.

### Weather information and delays

With latitude and longitude configured, Open-Meteo provides online weather information such as temperature, apparent temperature, humidity, wind, pressure, rainfall, daily high/low, sunrise, sunset and current conditions.

Automatic watering can be blocked by:

- Current rain or an active physical rain sensor.
- A configured rainfall threshold, including 24-hour rainfall rules.
- An after-rain cooldown period.
- Excessive wind.
- Master OFF, system pause or an active no-watering period.

Use the dashboard and event log to see why a run was delayed or skipped. Manual watering remains available where the applicable controls permit it.

## No-watering periods

Open **Setup → No-Watering Periods** to configure up to three periods. For each period, select the days, set a start and end time, enable it and save.

**Example:** to prevent automatic watering on weekends from 11 am to 5 pm, select Saturday and Sunday and enter `11:00`–`17:00`.

The periods use the controller's local time:

- The start is included; the end is excluded.
- Overnight periods continue into the next day.
- Equal start and end times block the whole selected day.
- Automatic runs stop during the period. Scheduled or queued runs are cancelled, rather than held for later.
- Manual watering remains available.

## Tank and mains control

An optional tank-level sensor supports monitoring and water-source selection. The interface provides **Auto: Tank**, **Auto: Mains**, **Force Tank** and **Force Mains** modes/statuses.

Configure the relevant sensor and valve outputs for your installation, and use the tank calibration page to set the sensor's empty and full readings.

## Home Assistant and MQTT

Enable MQTT in Setup and configure your broker connection. MQTT can connect the controller to Home Assistant, Node-RED or other automation systems.

After connecting to the broker, the controller publishes retained Home Assistant discovery configuration for:

- Rainfall and cooldown sensors.
- Master, pause, rain and wind binary sensors.
- One switch for each configured irrigation zone.

Zone switches use the configured zone names and the zone command topics, such as `espirrigation/cmd/zone/<index>` with the default base topic.

### Embed today's schedule

Open **`http://espirrigation.local/schedule-html`**, or substitute the controller's IP address, for a compact read-only schedule page.

It shows all zones scheduled for the controller's current local day, including earlier starts. Each zone displays up to two enabled start/end ranges in chronological order—for example, `11:30 - 12:00` and `17:30 - 18:00`. Empty zone names fall back to the zone number.

The page refreshes every 60 seconds and follows the browser's light/dark preference. End times use current Smart Watering durations; zero-duration adjustments are marked as skipped. An end time after midnight includes its date. If the clock is not synchronised, the page shows a waiting message.

**This is an estimated schedule, not a watering history.** Rain, wind, pauses and other delays can change actual watering. Use Events to review what happened.

For a Home Assistant [Webpage card](https://www.home-assistant.io/dashboards/iframe/):

```yaml
type: iframe
url: http://espirrigation.local/schedule-html
aspect_ratio: 60%
```

For another dashboard:

```html
<iframe src="http://espirrigation.local/schedule-html"
        title="Today's irrigation schedule"
        style="width:100%;height:360px;border:0"></iframe>
```

The viewing browser must be able to reach the controller. For an HTTPS dashboard, serve the controller page through an HTTPS reverse proxy to avoid blocked HTTP iframe content. Keep Home Assistant's default iframe sandbox enabled.

### Custom schedule styling

Open **Setup → Schedule HTML Styles** and enter rules in **Custom CSS**. The saved CSS is added after the built-in styles, allowing it to override them. Styling on the parent dashboard does not cross into the iframe.

Available selectors:

```css
.schedule-page
.schedule-content
.schedule-heading
.schedule-date
.schedule-table
.schedule-columns
.schedule-zone
.schedule-zone-name
.schedule-times
.schedule-time
.schedule-status
```

## Hardware and wiring

Features and pin assignments depend on the selected board and firmware variant.

| Hardware | Typical use |
| --- | --- |
| **ESP32** | Standard development-board installation. |
| **ESP32-S3** | ESP32-S3 installation with the matching firmware. |
| **ESP32 + 240 × 320 SPI TFT** | Full-colour status display. |
| **ESP32 + I²C OLED** | Compact display using fewer pins. |
| **ESP8266 + I²C LCD** | Separate lightweight controller variant. |
| **KC868-A6 / A8** | ESP32 relay controllers with screw terminals. |

A typical installation uses a compatible controller, relay module, irrigation valves, a valve-rated power supply, terminal blocks, fuse protection, irrigation cable and a weatherproof enclosure. Use the voltage and AC/DC supply specified for your valves; do not assume all valves use the same supply.

Optional equipment includes rain, soil-moisture, tank-level and temperature/humidity sensors; tank and mains valves; and a display. A photoresistor can switch or dim an enclosure display when the door is closed.

### Wiring examples

Confirm the wiring against your particular board and components before applying power.

<p align="center">
<img width="1000" alt="ESP32 irrigation controller wiring diagram" src="https://github.com/user-attachments/assets/df3b2d56-5021-43df-a429-55fd85ebc625" />
</p>

**ESP32-S3 with six relays and 24 V AC valves:**

<p align="center">
<img width="650" alt="ESP32-S3 six-zone 24 V AC irrigation wiring example" src="https://github.com/user-attachments/assets/f9d2c234-1e8a-4ff2-8e8d-51a5bdf73905" />
</p>

**TFT status display:**

<p align="center">
<img width="215" height="113" alt="TFT showing irrigation controller status" src="https://github.com/user-attachments/assets/42fbe90b-aeb4-48c1-9cfa-cd5753487815" />
</p>

### GPIO assignments and relay polarity

Assign GPIOs for zone outputs, tank/mains valves, sensors and displays in Setup. Check that each selected pin is suitable for its function on your board. GPIO changes may require a reboot.

Outputs can be **active HIGH** or **active LOW**. Many relay modules turn on when their input is LOW; choose the polarity that matches your module and verify it with a manual test.

### KC868 boards

KC868 configurations use PCF8574 I/O expanders, with automatic detection, configurable polarity and GPIO fallback where supported. Default addresses are:

| Address | Function |
| --- | --- |
| `0x24` | Relay outputs |
| `0x22` | Inputs |

Manual compilation requires the [Kincony PCF8574 library](https://www.kincony.com/forum/attachment.php?aid=1697).

### Electrical safety

- Use a power supply or transformer rated for the valves and the maximum number that may run together.
- Never apply more than **3.3 V** directly to an ESP32 GPIO. Higher-voltage sensor signals require suitable signal conditioning.
- Use appropriate inductive-load suppression: a correctly oriented flyback diode for DC valves, or a suitably rated MOV/RC snubber for AC valves. Do not use a DC flyback diode across an AC valve.
- Keep solenoid wiring separate from controller and sensor wiring to reduce interference and resets.
- Enclose and protect mains wiring, separate it from low-voltage wiring, and use a qualified person where required.

## Firmware updates

### Update through your browser

Browser updates require firmware that includes the `/update` route and a partition layout with space for two OTA application slots.

1. In **Setup → Firmware Updates**, save an OTA password of **8–64 characters**.
2. Obtain a compatible application `.bin` for your board. If compiling it yourself, use **Minimal SPIFFS (Large APPS with OTA)** and **Sketch → Export Compiled Binary**.
3. Open `http://<controller-ip>/update`.
4. Sign in with username **`admin`** and your OTA password.
5. Select the application `.bin` and start the update. Keep the controller powered until it finishes and restarts.

Use the application binary for this route, not a merged full-flash image. The controller stops active valves before writing, validates the image and switches boot slots only after a complete upload.

The OTA password also protects ArduinoOTA and configuration downloads. It is stored locally in the controller's configuration, as is the MQTT password. The browser updater does not depend on Arduino IDE network discovery; ArduinoOTA remains an alternative.

### Upgrade an older installation

If the installed firmware does not have `/update`, or uses a single-application partition layout, a USB/Web Serial installation may be needed first. Uploading an application alone cannot convert a single-slot layout to an OTA layout.

Before erasing, download your configuration and schedule and record your settings. **Erasing removes saved data.** Flash the full OTA-compatible image over USB/Web Serial with the appropriate partition table. Later application updates can use the browser updater.

## Manual installation

Open the [ESP32 sketch](firmware/ESP32-Irrigation/ESP32-Irrigation.ino) in Arduino IDE for ESP32 or ESP32-S3 builds.

1. Add `https://dl.espressif.com/dl/package_esp32_index.json` to **Additional Boards Manager URLs**.
2. Install **ESP32 by Espressif Systems**.
3. Select **ESP32 Dev Module** or **ESP32S3 Dev Module** to match your board.
4. Select **Minimal SPIFFS (Large APPS with OTA)** for sufficient application space and two OTA slots.
5. Compile and upload with the dependencies required by your chosen hardware.

Do not select **Huge APP** or **No OTA** if you intend to update over Wi-Fi; those layouts normally provide only one application slot.

The [ESP8266 sketch](firmware/ESP8266-Irrigation/ESP8266-Irrigation.ino) is a separate variant. The ESP32 board and partition settings above do not apply to it.

### Verify OTA builds on Windows

From the repository, run:

```powershell
.\tests\verify-ota.ps1
```

The default checks ESP32 Dev Module using Minimal SPIFFS. Select another target with:

```powershell
.\tests\verify-ota.ps1 -Target esp32s3
.\tests\verify-ota.ps1 -Target all
```

The helper keeps Arduino's persistent compilation cache enabled. The first build can take several minutes; later builds with matching settings can reuse the cache.

## Networking reference

| Service | Address or hostname |
| --- | --- |
| Setup Wi-Fi network | `ESPIrrigationAP` |
| Setup portal while connected to the hotspot | `http://192.168.4.1` |
| Dashboard on your normal network | `http://espirrigation.local/` or the assigned IP |
| ArduinoOTA hostname | `ESP32-Irrigation` |

### Web endpoints

Available routes depend on the firmware variant and enabled build options.

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Main dashboard. |
| GET | `/setup` | Controller configuration. |
| GET | `/schedule-html` | Read-only daily schedule. |
| GET | `/status` | JSON system status. |
| GET | `/diagnostics` | Browser diagnostics. |
| GET | `/diagnostics.json` | JSON diagnostics. |
| GET | `/events` | Event log. |
| GET | `/tank` | Tank sensor calibration. |
| GET / POST | `/update` | OTA page and firmware upload; requires OTA support and authentication. |
| GET | `/download/config.txt` | Saved configuration; requires OTA credentials when an OTA password is configured. |
| GET | `/download/schedule.txt` | Saved schedule. |
| GET | `/download/events.csv` | Event log CSV. |
| GET | `/i2c-test` | **Pulses relay outputs**; only available with debug routes enabled. |
| POST | `/stopall` | Stop all active zones. |
| POST | `/valve/on/<z>` | Start a zone manually. |
| POST | `/valve/off/<z>` | Stop a zone manually. |
| POST | `/reboot` | Reboot the controller. |

Zone paths use a **zero-based index**: `0` means zone 1, `1` means zone 2, and so on. A rejected manual start returns HTTP `409`.

Use the dashboard for normal operation. Typing a POST endpoint into the browser address bar sends a GET request and does not perform the action.

## Troubleshooting

Start with **`/diagnostics`** for controller health and **`/events`** for watering history and delay reasons.

| Problem | What to check |
| --- | --- |
| `espirrigation.local` will not open | Try the Android app or use the IP from your router's connected-device list. Confirm both devices are on a network that permits communication. |
| App cannot find the controller | Check power and Wi-Fi. Use Enter IP if the controller is outside the scanned range. Guest/client isolation can block access. |
| Setup page at `192.168.4.1` will not open | Join ESPIrrigationAP first and stay connected despite any no-internet warning. This address is for the setup hotspot. |
| App controls have disappeared | They hide when connected. Press Android Back to show them again. |
| ESP32 resets when a valve switches | Check supply capacity, inductive-load suppression, grounding, relay ratings and cable separation. |
| Relay works backwards | Check active HIGH/LOW polarity in Setup. |
| Wrong valve operates | Check zone numbering, GPIO assignments, relay wiring and output polarity. |
| Weather does not update | Check internet access, latitude/longitude, timezone and DNS. Local schedules need a valid controller clock. |
| Smart Watering does not shorten a run | Check Minimum Adjusted Runtime and the current adjustment factor. |
| Automatic watering is skipped | Check master/pause state, weather delays, no-watering periods, selected days and controller local time. |
| Embedded schedule does not load | Check reachability from the viewing browser and HTTP/HTTPS restrictions. |
| Browser firmware update is unavailable | Check that the installed firmware includes `/update`, OTA is supported and the partition layout has two application slots. |

When reporting a problem, include your board model, firmware version, symptoms and relevant diagnostic or event-log entries.

## Project links

- [GitHub repository](https://github.com/numerik11/ESP32-Irrigation-Controller)
- [Web Flasher](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)
- [Android app releases](https://github.com/numerik11/ESP32-Irrigation-Controller/releases/tag/AndroidAPK)

Bug reports, testing and suggestions are welcome. If the project is useful to you, consider giving it a star on GitHub.

Beau
