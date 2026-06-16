# T1000-E Setup

This document covers the practical setup flow for RemEX T1000-E tracker firmware: flashing, Bluetooth app configuration, current firmware defaults, and where those defaults live in the repo.

## Setup Flow

1. Flash the tracker firmware. 
2. Reboot the device and connect with the Bluetooth configuration app (Sensecraft). Need to triple-click the button to put it into BLE pairing mode, then click Scan.
3. It reads the current config
4. Confirm LoRaWAN region, activation mode (keep ABP), data-rate bounds (set minimum DR above 0), positioning strategy (wifi / ble / gnss, or just ble / gnss), scan limits (default is 6 secs for BLE), and beacon UUID settings.
5. Reset the device after changing LoRaWAN or scan settings: on exiting the setup, the device reboots with the new settings.
6. Confirm the device joins or transmits on the expected LoRaWAN port. To see on Terminal, need to disconnect and reconnect the data cable once after it beeps.

Normal firmware flashing does not necessarily erase saved config. The firmware stores config in FDS flash records and applies RemEX defaults only when the saved `param_version` is older than `REMEX_CREW_CONFIG_VERSION`.

## Flashing

### Firmware Files

Firmware artifacts are in `firmware/`.

- `firmware/t1000_e_tracker_latest.uf2` is the easiest file to flash through the UF2 bootloader.
- `firmware/t1000_e_tracker_latest.hex` is for SWD/J-Link flashing.
- Versioned files use `t1000_e_tracker_vMAJOR.MINOR.PATCH-bBUILD.uf2` and `.hex`.
- The compiled firmware version is defined in `t1000_e/tracker/inc/firmware_version.h`.

As of this checkout, `firmware/t1000_e_tracker_latest.uf2` points to `t1000_e_tracker_v1.0.0-b51.uf2`, and `FIRMWARE_VERSION_STRING` is `1.0.0-b51`.

### UF2 Bootloader Method

Use this for normal field updates.

1. Connect the T1000-E over USB with a data-capable cable.
2. Enter UF2 bootloader mode. To put into bootloader mode: Hold button on tracker, connect / disconnect / connect the charger cable in quick succession, release button
3. A removable USB drive should appear.
4. Copy `firmware/t1000_e_tracker_latest.uf2` onto that drive.
5. Wait for the copy to finish. The device should flash, disconnect, and reboot automatically.
6. Reconnect with the Bluetooth app and check `AT+VER=?` or `AT+CONFIG=?`. OR
7. To see on Terminal, need to disconnect and reconnect the data cable once after it beeps. Then look for the port. Use 115200kbps, and CoolTerm, Serial Web App (Capuf), or Terminal in vsCode.

If the USB drive does not appear, verify the cable supports data, try the alternate button/reset timing, and confirm the T1000-E bootloader is installed.

### SWD/J-Link Method

Use this for recovery, bootloader work, or when UF2 is unavailable.

1. Connect a J-Link or compatible SWD debugger.
2. Flash the `.hex` image with SEGGER Embedded Studio or `nrfjprog`.
3. If recovering a fully blank device, make sure the SoftDevice and bootloader requirements are satisfied. The repo includes `firmware/s140_nrf52_7.3.0_softdevice.hex` and bootloader artifacts.
4. Reboot and verify with `AT+VER=?`.

### Building New Firmware

The repo has `./build_firmware.sh`. It reads version values from `t1000_e/tracker/inc/firmware_version.h`, builds with SEGGER Embedded Studio, creates `.hex`/`.uf2` outputs, and updates the `latest` links.

The expected local tool paths are documented in `firmware/README.md`, including SEGGER Embedded Studio and the nRF5 SDK.

## Config Via Bluetooth App

The T1000-E advertises over BLE as `T1000-E XXXX`, where `XXXX` is derived from the device BLE address. The firmware exposes Nordic UART Service, and the SenseCAP-style app sends AT commands over that BLE UART link.

The same commands can also be sent over serial when available. For BLE, responses are returned through notifications.

### Command Format

- Help: `AT?`
- Command help: `AT+CONFIG?`
- Read value: `AT+CONFIG=?`
- Set value: `AT+POS_INT=1`
- Reset: `ATZ`

Commands must end with CRLF. The parser returns `OK`, `AT_PARAM_ERROR`, `AT_SAVE_FAILED`, or another AT status string.

Configuration changes are saved immediately after a successful set command. The save path is `check_save_param_type()` plus `save_Config()` in `t1000_e/tracker/src/app_at.c` and `t1000_e/tracker/src/app_at_command.c`.

### Basic Verification Commands

```text
AT+VER=?
AT+CONFIG=?
AT+BAT=?
AT+LBDADDR=?
```

`AT+CONFIG=?` returns a JSON-like summary including model, DevEUI, firmware version, region, sub-band, join type, keys, DR settings, positioning settings, scan settings, battery voltage, and test mode.

### LoRaWAN Commands

| Setting | Command | Current default | Options / validation |
| --- | --- | --- | --- |
| Platform | `AT+PLATFORM` | `1` (`IOT_PLATFORM_OTHER`) | `0` SenseCAP TTN, `1` Other, `2` Helium, `3` TTN, `4` SenseCAP Helium |
| Region | `AT+BAND` | `5` EU868 | Supported IDs include `0` TTN AS923_2, `1` AU915, `5` EU868, `6` KR920, `7` IN865, `8` US915, `9` RU864, `10`-`14` Helium AS923 variants, `15` TTN AS923_1, `16`-`18` AS923 groups. CN470, CN779, and EU433 are rejected by the setter. |
| Channel group | `AT+CHANNEL` | `1` | `0`-`7`, valid only for US915 and AU915. Runtime modem sub-band is `ChannelGroup + 1`. |
| Activation | `AT+TYPE` | `1` ABP | `1` ABP or `2` OTAA. The help mentions `0` none, but the setter maps non-OTAA valid values to ABP. |
| Retry | `AT+RETRY` | `2` 1N | `1` 1C or `2` 1N. |
| ADR enabled | `AT+LR_ADR_EN` | `0` disabled | `0` or `1`. |
| DR minimum | `AT+LR_DR_MIN` | `1` | Region-limited. EU868/KR920/IN865/RU864 allow `0`-`5`; US915 allows `1`-`4`; AU915 allows `3`-`6`; AS923 variants allow `3`-`5`. Must be `<= LR_DR_MAX`. |
| DR maximum | `AT+LR_DR_MAX` | `5` | Same regional validation as DR minimum. Must be `>= LR_DR_MIN`. |

LoRaWAN identity commands are `AT+APPEUI`, `AT+APPKEY`, `AT+NWKKEY`, `AT+NWKSKEY`, `AT+APPSKEY`, `AT+DADDR`, and `AT+DEUI`.

Important current behavior:

- ABP is the RemEX default.
- DevAddr, NwkSKey, and AppSKey can be derived deterministically from the factory DevEUI and build-time RemEX ABP settings.
- Recent firmware treats DevEUI as factory identity. The version history notes that config app / AT DevEUI writes are rejected in newer builds.
- OTAA remains selectable, but OTAA credentials must be provisioned correctly before use.

### Tracker Behavior Commands

| Setting | Command | Current default | Options / validation |
| --- | --- | --- | --- |
| Positioning strategy | `AT+POS_STRATEGY` | `6` BLE then GNSS | `0` GNSS only, `1` WiFi only, `2` WiFi then GNSS, `3` GNSS then WiFi, `4` BLE only, `5` BLE then WiFi, `6` BLE then GNSS, `7` BLE then WiFi then GNSS. |
| Routine interval | `AT+POS_INT` | `1` minute | `1` to `10080` minutes. |
| SOS mode | `AT+SOS_MODE` | `1` | `0` or `1`. |
| Accelerometer | `AT+ACC_EN` | `0` disabled | `0` or `1`. |
| GNSS timeout | `AT+STA_OT` | `30` seconds | `10` to `120` seconds. |
| WiFi result max | `AT+WIFI_MAX` | `3` | `3` to `5`. |
| Beacon scan timeout | `AT+BEAC_OT` | `6` seconds | `3` to `10` seconds. |
| Beacon result max | `AT+BEAC_MAX` | `3` | `3` to `5`. |
| Extra beacon UUID | `AT+BEAC_UUID` | empty | Hex string up to 16 bytes. For production BLE RF hints, exact full 16-byte UUID matching is expected. |
| Test mode | `AT+TESTMODE_TYPE` | `0` | `0` or `1`. |

The production BLE RF hint whitelist is compile-time configured in `t1000_e/tracker/inc/crew_dr_strategy_config.h`. The current built-in UUID is `FDA50693-A4E2-4FB1-AFCF-C6EB07647825`. The Bluetooth app's beacon UUID field acts as one additional approved UUID for trials or one-off installs.

BLE iBeacon Minor values `1`-`5` are used as LoRaWAN DR hints. LinkCheck can later refine the DR based on network margin.

## Current Firmware Defaults

The first-boot/default config is defined in `apps/common/default_config_settings.h` and assigned into `app_param` in `t1000_e/tracker/src/app_config_param.c`.

Current RemEX defaults:

| Field | Default macro | Value |
| --- | --- | --- |
| Config version | `REMEX_CREW_CONFIG_VERSION` | `2` |
| Platform | `REMEX_DEFAULT_PLATFORM` | `IOT_PLATFORM_OTHER` (`1`) |
| Region | `REMEX_DEFAULT_ACTIVE_REGION` | `LORAMAC_REGION_EU868` (`5`) |
| Channel group | `REMEX_DEFAULT_CHANNEL_GROUP` | `1` |
| Activation | `REMEX_DEFAULT_ACTIVATION_TYPE` | `ACTIVATION_TYPE_ABP` (`1`) |
| Retry | `REMEX_DEFAULT_RETRY` | `RETRY_STATE_1N` (`2`) |
| ADR | `REMEX_DEFAULT_LORAWAN_ADR_ENABLED` | `false` |
| DR min / max | `REMEX_DEFAULT_LORAWAN_DR_MIN/MAX` | `1` / `5` |
| Accelerometer | `REMEX_DEFAULT_ACCELEROMETER_ENABLED` | `false` |
| SOS mode | `REMEX_DEFAULT_SOS_MODE` | `1` |
| Uplink interval | `REMEX_DEFAULT_UPLINK_INTERVAL_MIN` | `1` minute |
| Position strategy | `REMEX_DEFAULT_POSITION_STRATEGY` | `6` BLE then GNSS |
| GNSS max scan | `REMEX_DEFAULT_GNSS_MAX_SCAN_TIME_S` | `30` seconds |
| iBeacon max | `REMEX_DEFAULT_IBEACON_SCAN_MAX` | `3` |
| iBeacon timeout | `REMEX_DEFAULT_IBEACON_SCAN_TIMEOUT_S` | `6` seconds |
| Extra group UUID length | `REMEX_DEFAULT_GROUP_UUID_LEN` | `0` |

Some values in `app_config_param.c` are initialized directly rather than through a macro. `wifi_max` is currently initialized to `3`.

## Where To Find Things

- Firmware version and changelog: `t1000_e/tracker/inc/firmware_version.h`
- First-boot RemEX defaults: `apps/common/default_config_settings.h`
- Default `app_param` object: `t1000_e/tracker/src/app_config_param.c`
- Config struct and enum definitions: `t1000_e/tracker/inc/app_config_param.h`
- AT command names: `t1000_e/tracker/inc/app_at.h`
- AT command table and parser: `t1000_e/tracker/src/app_at_command.c`
- AT command implementation and validation: `t1000_e/tracker/src/app_at.c`
- Saved config / FDS migration logic: `t1000_e/tracker/src/app_at_fds_datas.c`
- BLE app transport / Nordic UART Service: `t1000_e/tracker/src/app_ble_all.c`
- Position strategy constants: `apps/examples/11_lorawan_tracker/main_lorawan_tracker.h`
- BLE DR hint policy and production UUID whitelist: `t1000_e/tracker/inc/crew_dr_strategy_config.h`
- Build and release artifacts: `firmware/README.md`, `build_firmware.sh`, and `firmware/`

## Notes For Fleet Setup

- For EU868 RemEX crew tags, the current defaults should already be ABP, EU868, DR1-DR5, one-minute routine interval, and BLE-then-GNSS positioning.
- Reflashing alone may preserve older saved settings. If a unit does not show expected defaults in `AT+CONFIG=?`, either update the config through BLE or intentionally bump `REMEX_CREW_CONFIG_VERSION` in firmware when a fleet-wide migration is required.
- For US915/AU915 deployments, set `AT+BAND` first, then set `AT+CHANNEL` to the desired 0-based channel group.
- After changing LoRaWAN network settings, reset the device with `ATZ` or power-cycle it before validating join/uplink behavior.
- Keep `AT+BEAC_UUID` empty for production if the built-in UUID whitelist is sufficient. Use it only for an extra trial UUID.
