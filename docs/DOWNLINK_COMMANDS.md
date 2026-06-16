# Downlink Commands

This file tracks application downlink commands implemented by the T1000-E
tracker firmware. Add new command IDs here when they are added to firmware.

## FPort 5 - Tracker Application Commands

FPort 5 downlinks are decoded by `app_lora_packet_downlink_decode()`. Byte 0 is
the command ID.

| Command ID | Name | Payload | Effect | Persistence |
| --- | --- | --- | --- | --- |
| `0x81` | Set periodic interval | `81 xx xx mm mm` | Sets routine tracking interval in minutes. Valid range: `1..10080`. `mm mm` is a 16-bit big-endian integer read from bytes 3-4. | Saved to FDS |
| `0x82` | Buzzer/LED prompt | `82 00` or `82 01` | `01` starts the LoRa downlink beep/LED prompt; `00` returns beep/LED to idle. | Runtime only |
| `0x86` | Set tracking type | `86 tt` | Sets positioning strategy. Accepted values: `0`, `1`, and `3..7`. | Saved to FDS |
| `0x88` | Send power/status uplink | `88` | Immediately sends the power/status uplink. | Runtime only |
| `0x89` | Reboot | `89` | Calls `hal_mcu_reset()`. | Runtime only |
| `0x8D` | SOS continuous control | `8D 00` or `8D 01` | `01` enters continuous SOS mode; `00` exits continuous SOS mode. | Runtime only |
| `0x8E` | Power off | `8E` | Requests firmware soft power-off through the normal deferred shutdown path. Long press can still power the tag back on from soft-off. | Runtime only |

## FPort 10 - Gateway Assistance Commands

FPort 10 is reserved for gateway assistance. The current message is handled by
`gateway_assistance_handle_downlink()`.

| Message Type | Name | Payload | Effect | Persistence |
| --- | --- | --- | --- | --- |
| `0x01` | Gateway position update | `01 lat[4] lon[4]` | Caches vessel/gateway latitude and longitude, then sends a `PAIR600` assistance command to GNSS. `lat` and `lon` are signed `int32` values scaled by `1e7` in firmware struct byte order. | Runtime cache |

## Notes

- FPort 5 command IDs are defined in `t1000_e/tracker/inc/app_lora_packet.h`.
- FPort 10 is defined by `GATEWAY_ASSISTANCE_PORT` in `t1000_e/tracker/inc/crew_lorawan_ports.h`.
- Standard LoRaWAN MAC commands, such as DeviceTimeReq/Ans, are outside this application command table.
