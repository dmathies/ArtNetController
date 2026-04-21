# ArtNetController

An ESP32-based Art-Net lighting controller. The repository contains two firmware variants that share the same Wi-Fi, web UI, OTA update flow, and LittleFS-backed configuration storage.

## What This Project Does

- Receives Art-Net DMX on UDP port `6454`
- Exposes a built-in web UI on port `80` for status, configuration, and OTA updates
- Stores Wi-Fi and runtime configuration in LittleFS
- Supports two application modes:
  - `BLDC` controller: maps one DMX channel to a motor speed controller over UART
  - `LED` controller: maps four 16-bit Art-Net values to four PWM LED outputs

## Hardware Targets

The project currently has PlatformIO environments for:

- `seeed_xiao_esp32s3`
- `seeed_xiao_esp32s3_led`
- `nodemcu-32s`
- `nodemcu-32s_led`

The `_led` environments build the LED receiver firmware. The non-`_led` environments build the BLDC motor controller firmware.

## Firmware Variants

### BLDC Variant

Source entry point: `src/main_bldc.cpp`
The target hardware is the inexpensive Sperax personal treadmill.  Sold by for example [amazon](https://www.amazon.co.uk/dp/B0F9B2W1XB).  The output is intended to go through a level converter befor being connected to the motor controller UART header.  
**WARNING**: if you do choose to mod this treadmill, be aware that the BLDC motor controller board is references to mains voltages (there is no isolation).

Functions:
- Listens for Art-Net DMX
- Reads a single DMX value from the configured start address
- Converts `0..255` to BLDC controller steps `0..60`
- Sends motor commands over `Serial2`
- Publishes live status to the dashboard over HTTP and WebSocket

Default BLDC UART pins are set per board in `platformio.ini`.

For `seeed_xiao_esp32s3`:

- `TX = GPIO43`
- `RX = GPIO44`
- `OE = -1`

For `nodemcu-32s`:

- `OE = GPIO19`
- Default UART pins in code: `TX = GPIO33`, `RX = GPIO32`

### LED Variant

Source entry point: `src/main_led.cpp`

- Listens for Art-Net DMX
- Reads four consecutive 16-bit values starting at the configured start address
- Drives four PWM outputs (12-bit resolution)

Default LED pin mapping in code:

- `LED 0 -> GPIO1`
- `LED 1 -> GPIO2`
- `LED 2 -> GPIO3`
- `LED 3 -> GPIO21`

For `nodemcu-32s_led`, the build overrides the mapping to avoid the serial console pins and to let the onboard LED act as `LED 3`:

- `LED 0 -> GPIO16`
- `LED 1 -> GPIO17`
- `LED 2 -> GPIO21`
- `LED 3 -> GPIO2` (onboard LED on many NodeMCU-32S boards)


## Project Layout

```text
src/
  main.cpp            Shared runtime, HTTP server, OTA, WebSocket status
  main_bldc.cpp       BLDC Art-Net controller
  main_led.cpp        4-channel LED Art-Net receiver
  WifiManager.*       Wi-Fi connection and management AP logic
  Configuration.*     Persistent config stored in LittleFS
  bldc_uart.h         BLDC serial protocol helper

data/
  wifi-manager/       Web UI files served by the ESP32
  *.txt               Default persisted config values

platformio.ini        Board environments and build configuration
partitions.csv        OTA + LittleFS partition table
```

## Build Configuration

Important PlatformIO settings:

- Framework: `arduino`
- Platform: `espressif32 @ 6.5.0`
- Filesystem: `littlefs`
- Partition table: `partitions.csv`
- Build type: `debug`
- Dependency:
  - `bblanchon/ArduinoJson@^6.21.3`

Partition layout:

- `app0` OTA slot
- `app1` OTA slot
- `littlefs` data partition at `0x310000`, size `0x40000`

## Common Commands

Build BLDC firmware for XIAO ESP32S3:

```powershell
pio run -e seeed_xiao_esp32s3
```

Build LED firmware for XIAO ESP32S3:

```powershell
pio run -e seeed_xiao_esp32s3_led
```

Upload firmware:

```powershell
pio run -e seeed_xiao_esp32s3 --target upload
```

Build filesystem image:

```powershell
pio run -e seeed_xiao_esp32s3 --target buildfs
```

Upload filesystem image:

```powershell
pio run -e seeed_xiao_esp32s3 --target uploadfs
```

Open serial monitor:

```powershell
pio device monitor -b 115200
```

Use the matching environment name for the board and firmware variant you want.

## Web Interface

The firmware starts a native HTTP server on port `80`.

Main pages:

- `/` or `/index.html` - status dashboard
- `/settings.html` - Wi-Fi, Art-Net, and network settings
- `/ota.html` - firmware and filesystem OTA upload page

API endpoints:

- `GET /status`
- `GET /networks`
- `GET /credentials`
- `PUT /credentials`
- `GET /update`
- `POST /update`
- `POST /updatefs`
- `GET /ws` - WebSocket status stream

If stored Wi-Fi credentials are missing or connection fails, the device starts a management access point:

- SSID: `WIFI-MANAGER`

## Stored Configuration

Configuration is persisted in LittleFS as plain text files:

- `/ssid.txt`
- `/pass.txt`
- `/hostname.txt`
- `/address.txt`
- `/universe.txt`
- `/dhcp.txt`
- `/ip.txt`
- `/gateway.txt`
- `/subnet.txt`
- `/dns1.txt`
- `/dns2.txt`
- `/start_value.txt`

The web settings page edits these values through `PUT /credentials`, then schedules a restart.

## Runtime Behavior

Shared runtime services in `src/main.cpp` provide:

- Art-Net packet parsing
- Wi-Fi reconnect handling
- HTTP file serving from LittleFS
- WebSocket status broadcasting
- OTA firmware and filesystem upload handlers
- health and task runtime reporting

The dashboard displays items such as:

- Art-Net activity age
- output value(s)
- Wi-Fi RSSI, IP, and MAC address
- heap usage
- MCU temperature
- task state and stack high-water marks

## OTA Update Notes

The built-in OTA page accepts:

- firmware image upload to the active OTA slot via `POST /update`
- filesystem image upload to the LittleFS partition via `POST /updatefs`

The web page currently refers to the filesystem image as `spiffs.bin`, but the project is configured to use `LittleFS`. Build and upload the filesystem image using the PlatformIO filesystem targets for the selected environment.

## Notes For Development

- `src/main.cpp` is the shared runtime layer used by both variants
- `build_src_filter` in `platformio.ini` selects either the BLDC or LED entry point
- Wi-Fi scanning is performed on demand from the settings page instead of at boot
- If Art-Net traffic goes idle for long enough, the UDP listener attempts a rebind

## License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](/c:/Users/derek/OneDrive/Documents/PlatformIO/Projects/CableCar/LICENSE).
