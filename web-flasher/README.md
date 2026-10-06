# ESP32 Irrigation Firmware Update

This folder contains a browser-based USB firmware installer for the ESP32 irrigation firmware.

Goto: https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/

## Boards

- ESP32 Dev Module
- ESP32-S3 DevKitC-1

## Use

### Version 3.3.0: independent source relay disabling

Setting City Water Relay GPIO to **-1** disables the mains source output and
frees onboard relay 5 for zone 5. Setting Tank Relay GPIO to **-1** disables
the tank source output and frees onboard relay 6 for zone 6. Set the zone
count to **6** to use all six KC868-A6 relays as zones. Each source can be
disabled independently; an unused relay stays OFF. Source switching preserves
running zones 5 and 6. The **Enable Tank** option still controls water-source
selection: unchecked selects mains.

The zone 5/6 correction retains version **3.3.0**; reinstall it if you already
installed the earlier 3.3.0 build.

### Version 3.2.10: KC868-A6 I2C fix

I2C now starts after the saved pin settings load. Relay expanders use those same
pins, initialize their outputs OFF before starting, and report initialization
failure. OLED initialization preserves the configured bus.

For KC868-A6, choose **ESP32 Dev Module**. The ESP32 defaults are now SDA **4**
and SCL **15**. Existing saved settings are preserved: after updating, set these
values in Setup, save, and reboot. The onboard inputs and relays should appear
at **0x22** and **0x24**; the configured SSD1306 address is **0x3C**.

Use Chrome or Edge. ESP Web Tools needs Web Serial, so Firefox and Safari will not work.

When GitHub Pages is enabled for this repository, open:

```text
https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/
```

If that URL returns `404`, GitHub Pages has not been enabled or has not finished deploying yet.

For local testing:

```powershell
cd web-flasher
python -m http.server 8080
```

Then open:

```text
http://localhost:8080
```

Choose the correct board, click Install, and select the ESP serial port when the browser asks.

## Browser OTA

Choose the matching board, then download the application .bin or copy its GitHub firmware URL. Enter the controller's IP address or hostname in the online updater and select **Open controller OTA** to open its /update page in a new tab. The default address is espirrigation.local. Sign in with the OTA password configured on the controller, then upload the file or install from the copied URL.

The online updater does not start an OTA installation automatically. The browser must be able to reach the controller's address.

## Refresh Binaries

After rebuilding firmware, copy the generated files from `.pio/build/<env>/` into the matching board folder:

- `bootloader.bin`
- `partitions.bin`
- `firmware.bin`

Also include `boot_app0.bin` from:

```text
%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin
```

Set both manifest `version` values and `updaterVersion` in `index.html` to the firmware version. Update `updaterBuild` and each manifest part's `?v=` token whenever publishing new binaries so browsers fetch the refreshed files.
