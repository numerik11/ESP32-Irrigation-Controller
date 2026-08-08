# ESP32 Irrigation Web Flasher

This folder contains a browser-based USB firmware installer for the ESP32 irrigation firmware.

## Boards

- Wemos D1 R32
- ESP32-S2 Mini
- ESP32-S3 DevKitC-1

## Use

Open `index.html` from a local web server or host this folder on HTTPS, then use Chrome or Edge.

If GitHub Pages is enabled for this repository, the flasher should be available at:

```text
https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/
```

For local testing:

```powershell
cd web-flasher
python -m http.server 8080
```

Then open:

```text
http://localhost:8080
```

## Refresh Binaries

After rebuilding firmware, copy the generated files from `.pio/build/<env>/` into the matching board folder:

- `bootloader.bin`
- `partitions.bin`
- `firmware.bin`

Also include `boot_app0.bin` from:

```text
%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin
```
