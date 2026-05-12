# ArtNet Controller

An ESP32-based Art-Net lighting controller. The repository contains multiple firmware variants that share the same Wi-Fi, web UI, OTA update flow, and LittleFS-backed configuration storage.

## What This Project Does

- Receives Art-Net DMX on UDP port `6454`
- Exposes a built-in web UI on port `80` for status, configuration, and OTA updates
- Stores Wi-Fi and runtime configuration in LittleFS
- Supports multiple application modes:
  - `BLDC` controller: maps one DMX channel to a motor speed controller over UART
  - `LED` controller: maps four 16-bit Art-Net values to four PWM LED outputs
  - `Relay` controller: maps one DMX channel to a binary relay output

## Hardware Targets

The project currently has PlatformIO environments for:

- `seeed_xiao_esp32s3`
- `seeed_xiao_esp32s3_led`
- `seeed_xiao_esp32s3_relay`
- `nodemcu-32s`
- `nodemcu-32s_led`
- `nodemcu-32s_relay`

The `_led` environments build the LED receiver firmware, the `_relay` environments build the relay firmware, and the remaining environments build the BLDC motor controller firmware.

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

### Relay Variant

Source entry point: `src/main_relay.cpp`

- Listens for Art-Net DMX
- Reads one 8-bit DMX value from the configured start address
- Switches the relay `OFF` for values `< 128`
- Switches the relay `ON` for values `>= 128`

Default relay pin mapping:

- `seeed_xiao_esp32s3_relay -> GPIO1`
- `nodemcu-32s_relay -> GPIO16`

The relay logic defaults to active-high output. Set `RELAY_ACTIVE_HIGH=0` in build flags if your relay board is active-low.


## Project Layout

```text
src/
  main.cpp            Shared runtime, HTTP server, OTA, WebSocket status
  main_bldc.cpp       BLDC Art-Net controller
  main_led.cpp        4-channel LED Art-Net receiver
  main_relay.cpp      Single-channel relay Art-Net receiver
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
  - `h2zero/NimBLE-Arduino`

Partition layout:

- `app0` OTA slot
- `app1` OTA slot
- `littlefs` data partition at `0x310000`, size `0x40000`

### Important Build Flags

The project uses a few non-default build flags in `platformio.ini` to tune behavior per board family:

- `APP_ASYNC_WEB_ENABLE`
  - `1` uses `ESPAsyncWebServer`
  - `0` uses the lighter synchronous `WebServer`
  - XIAO builds use async web; NodeMCU builds use sync web to reduce heap pressure

- `WEB_SOCKET_ENABLE`
  - enables the `/ws` status stream for the dashboard
  - enabled on XIAO builds, disabled on NodeMCU builds

- `APP_MDNS_ENABLE`
  - controls `.local` mDNS advertising
  - usually safe on XIAO
  - NodeMCU can run it, but `0` is the preferred low-memory setting if heap margin gets tight

- `APP_BLE_ENABLE`
  - master BLE enable switch

- `APP_BLE_GATT_ENABLE`
  - `1` enables the full NimBLE GATT service
  - `0` leaves BLE in advertising-only mode

- `APP_BLE_LOG_TAIL_BYTES`
- `APP_BLE_LAST_LOG_BYTES`
- `APP_BLE_MTU`
  - trim the BLE logging payload sizes and negotiated MTU

- `CONFIG_ASYNC_TCP_RUNNING_CORE`
- `CONFIG_ASYNC_TCP_STACK_SIZE`
  - tune the async networking task used by the async web backend
  - mainly relevant to the XIAO async-web builds

## Common Commands

Build BLDC firmware for XIAO ESP32S3:

```powershell
pio run -e seeed_xiao_esp32s3
```

Build LED firmware for XIAO ESP32S3:

```powershell
pio run -e seeed_xiao_esp32s3_led
```

Build relay firmware for XIAO ESP32S3:

```powershell
pio run -e seeed_xiao_esp32s3_relay
```

Upload firmware:

```powershell
pio run -e seeed_xiao_esp32s3 --target upload
```

Build filesystem image:

```powershell
pio run -e seeed_xiao_esp32s3 --target buildfs
```

Build a combined OTA bundle:

```powershell
pio run -e seeed_xiao_esp32s3 --target buildota
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
- `POST /updatebundle`
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
- OTA firmware, filesystem, and combined bundle upload handlers
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

- combined firmware + filesystem bundle upload via `POST /updatebundle`
- firmware image upload to the active OTA slot via `POST /update`
- filesystem image upload to the LittleFS partition via `POST /updatefs`

The recommended one-file OTA artifact is `.pio/build/<env>/ota_bundle.ota`. It is generated from the firmware image plus the LittleFS image and can be uploaded from the Combined tab on `/ota.html`.

`pio run -e <env> --target buildota` builds the firmware, builds the filesystem image, and writes the combined bundle. The same bundle is also refreshed automatically whenever both `firmware.bin` and `littlefs.bin` exist in the build directory.

## Notes For Development

- `src/main.cpp` is the shared runtime layer used by both variants
- `build_src_filter` in `platformio.ini` selects the BLDC, LED, or relay entry point
- Wi-Fi scanning is performed on demand from the settings page instead of at boot
- If Art-Net traffic goes idle for long enough, the UDP listener attempts a rebind

## License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](/c:/Users/derek/OneDrive/Documents/PlatformIO/Projects/ArtNetController/LICENSE).
