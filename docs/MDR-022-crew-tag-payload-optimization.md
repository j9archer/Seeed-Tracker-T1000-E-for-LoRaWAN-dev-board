---
title: "MDR-022: Crew Tag Payload Optimization and Decoder Slimming"
type: "mdr"
status: "accepted"
tags:
  - mob
  - mob/mdr
  - mob/lorawan
  - mob/crew-tags
  - payload
  - ble
  - wifi
aliases:
  - "MDR-022"
  - "Crew Tag Payload Optimization and Decoder Slimming"
---

# MDR-022: Crew Tag Payload Optimization and Decoder Slimming

Parent: [[Major Decision Records Index]]

## Status

Accepted

## Context

The current tracker payloads are still shaped around the stock SenseCAP T1000-E uplink model. The firmware has already diverged in important crew-tag ways:

- Routine active traffic uses FPort 5, alert/pass-through traffic uses FPort 6, and routine charger/spare traffic uses FPort 7 per MDR-018 and MDR-019.
- MOB/PIW alert payloads on FPort 6 are already compact custom records: position is 13 bytes, cancellation is 5 bytes, and no-fix is 5 bytes.
- BLE routine payloads use custom Data IDs `0x27` and `0x28` carrying iBeacon Major, Minor, and RSSI instead of stock BLE MAC/RSSI records.
- Routine payloads append a one-byte `locationAgeMin` trailer.
- BLE and WiFi are currently used as vessel-presence hints, not as full positioning inputs. MDR-012 uses the strongest approved BLE beacon as the movement and DR hint. MDR-020 uses WiFi as accepted/provisional presence, not as WiFi positioning.
- The gateway shim can route on FPort before decoding the encrypted application payload. FPort is therefore more useful than a Data ID byte for deciding whether a frame may be sampled or must pass to the master LNS.

The stock payload shape still spends airtime on fields that are not currently important to the MOB product path:

- Temperature and light are sent on every routine sensor/location uplink, but they are not part of current POB, onboard, uncertain, or MOB decisioning.
- Accelerometer X/Y/Z can be sent as three 16-bit samples, but current operation mostly needs event flags such as shock/motion rather than raw vectors.
- Routine WiFi payloads are padded to the configured WiFi record count during testing, even when only one accepted/provisional AP is operationally useful.
- Routine BLE payloads can send several iBeacon records, even though current behavior only needs the strongest approved beacon.
- The decoder keeps a broad stock parser surface for many unused SenseCAP frame types and setting helpers, which makes crew-tag behavior harder to audit.

## Current Information Sent

Routine stock-derived header:

- Data ID.
- Event state byte.
- Battery percentage.
- Temperature, 0.1 C resolution.
- Light sensor value.
- Optional raw accelerometer X/Y/Z.
- Optional scan records.
- Routine location-age trailer.

Routine BLE custom record:

- Count byte.
- Up to the configured BLE count, capped by firmware.
- Each record is iBeacon Major, iBeacon Minor, and RSSI, 5 bytes per beacon.
- Current BLE logic independently selects the strongest approved beacon for movement and DR behavior.

Routine WiFi record:

- Count byte.
- MAC and RSSI, 7 bytes per AP.
- Current firmware may insert an `FF` placeholder for provisional mobile/random-only acceptance.
- Current firmware pads shorter WiFi results to `wifi_scan_max * 7` bytes.

Routine GNSS:

- The legacy routine GNSS builder exists, but routine GNSS fallback is currently suppressed unless SOS/MOB state is active. MOB/PIW sends separate compact FPort 6 payloads.

Alert/MOB:

- `0x20` MOB position: mode/flags, latitude, longitude, HDOP, quality flags, battery.
- `0x21` MOB cancellation: elapsed seconds, battery, flags.
- `0x22` MOB no-fix: mode/flags, elapsed seconds, battery.

## Required Information By Phase

### Onboard / Routine

Required:

- Product phase or presence state: onboard/routine.
- Source used for onboard decision: BLE, WiFi fixed/global, WiFi provisional, GNSS, none.
- Event flags: on charge, SOS active, user triggered, GNSS ready, shock/motion if implemented later.
- Battery.
- Location age.
- For BLE success today: strongest approved iBeacon Major, Minor, and RSSI.
- For WiFi success today: strongest accepted/provisional BSSID, RSSI, and enough flags to distinguish fixed/global from provisional/random/mobile.

Not required on every routine uplink:

- Temperature.
- Light.
- Raw accelerometer vector.
- More than one BLE beacon.
- More than one WiFi AP.

### Uncertain

Required:

- Product phase or presence state: uncertain.
- Which methods were attempted and failed or were inconclusive.
- Battery.
- Event flags.
- Location age.
- Optional last local hint source and age if available.

Uncertain traffic should use FPort 6 when it is operationally significant and must bypass routine gateway sampling.

### MOB / PIW

Required:

- MOB/PIW phase.
- Latitude and longitude when available.
- Fix quality indicator, currently HDOP and quality flags.
- Movement hint when available: COG and SOG.
- Battery.
- Elapsed time since activation for no-fix/cancel paths.
- On-charge metadata when detected.

The existing compact FPort 6 MOB payloads are already close to the desired shape. COG/SOG should be added as a compact three-byte extension to position reports. These values should be firmware-derived from successive accepted fixes over a known movement interval, not copied from receiver instantaneous RMC/VTG COG/SOG.

### Future Indoor Positioning

Required for RSSI fingerprinting or multi-anchor positioning:

- Explicit record count.
- Multiple BLE and/or WiFi records.
- RSSI for every record.
- Stable identity per record: iBeacon Major/Minor for BLE, BSSID for WiFi.
- Optional record-source flags so backend solvers can mix BLE, fixed WiFi, and provisional WiFi deliberately.

This should be a deliberate expanded payload mode, not the default onboard heartbeat.

## Decision

Introduce a RemEX-native compact payload family and use FPort as the first-level message type and delivery class.

FPort should carry routing intent before payload subtype:

- Keep FPort 5 for sampled routine active presence.
- Keep FPort 6 for unfiltered alert, uncertain, MOB, and PIW traffic.
- Keep FPort 7 for sampled routine on-charge/spare presence.
- Add an unfiltered low-rate health/event port for operational information that must reach the master LNS even when routine presence is sampled.
- Add optional future ports for expanded RF fingerprinting only if the gateway/backend needs different filtering from routine presence.

When an FPort uniquely defines the payload schema, omit the Data ID byte and let the decoder select the schema by FPort. Keep a subtype byte only on broad ports that intentionally carry several related schemas.

Recommended FPort classes:

| FPort | Class | Shim policy | Payload typing |
|-------|-------|-------------|----------------|
| 0 | MAC/control | Forward | LoRaWAN MAC-only |
| 5 | Active presence | Sampled | Port-specific compact presence, no Data ID |
| 6 | Alert/uncertain/MOB | Unfiltered | Subtype retained because multiple alert schemas share this priority |
| 7 | Charger/spare presence | Independently sampled | Same compact presence schema as FPort 5 |
| 8 | Health/event | Unfiltered or strongly preserved | Port-specific health/event schema; subtype only when needed |
| 10 | Gateway assistance | Downlink only | Existing gateway/vessel assistance |
| 11 | RF fingerprint, optional | Deployment-defined | Expanded indoor-positioning records |

Recommended payload schemas:

- `CREW_PRESENCE_COMPACT`: routine onboard/presence payload for FPort 5 or FPort 7, no Data ID byte.
- `CREW_HEALTH_EVENT`: low-rate health/event payload for FPort 8, no Data ID when the port is narrowed to one event family.
- `CREW_ALERT_COMPACT`: uncertainty/pass-through payload for FPort 6. Keep existing MOB subtypes for now.
- `CREW_LOCAL_FINGERPRINT`: expanded local RF fingerprint payload for future indoor positioning on FPort 11, or FPort 6 only if operationally urgent.
- Keep existing `0x21` and `0x22` FPort 6 MOB cancellation/no-fix payloads unchanged for now.
- Extend `0x20` FPort 6 MOB position payload with compact COG/SOG when decoder and backend support are ready.

### FPort 5/7 CREW_PRESENCE_COMPACT

Common prefix:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Schema + phase | High nibble schema version, low nibble phase |
| 1 | 1 | Event flags | Existing event bits: on charge, SOS, user, GNSS ready, future shock/motion |
| 2 | 1 | Battery | Battery percentage |
| 3 | 1 | Location age | Minutes, `255` unknown, `254` saturated |
| 4 | 1 | Source flags | Source and quality flags |

Source flags:

- Bits 0-2: source type: `0` none, `1` BLE, `2` WiFi fixed/global, `3` WiFi provisional, `4` GNSS/vessel assistance, `5-7` reserved.
- Bit 3: source accepted for onboard decision.
- Bit 4: RSSI present.
- Bit 5: source identity present.
- Bits 6-7: reserved.

BLE source extension, present when source is BLE:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 5 | 2 | Major | iBeacon Major, little-endian to match current firmware copy behavior |
| 7 | 2 | Minor | iBeacon Minor, little-endian |
| 9 | 1 | RSSI | signed int8 dBm |

WiFi source extension, present when source is WiFi:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 5 | 6 | BSSID | WiFi MAC |
| 11 | 1 | RSSI | signed int8 dBm |
| 12 | 1 | WiFi flags | channel/origin/provisional detail, exact bit split TBD |

Expected sizes:

- No local source: 5 bytes.
- BLE strongest source: 10 bytes.
- WiFi strongest source: 13 bytes.

### FPort 8 CREW_HEALTH_EVENT

FPort 8 is for low-rate operational data that should normally reach the master LNS even when routine presence is sampled. The goal is not to send these fields on every presence uplink; it is to send them occasionally or on meaningful change.

Candidate health/event scheduling:

- Battery health: every 12 hours, or when battery drops by a configured percentage since the last health report, whichever is earlier.
- Low-battery threshold crossing: immediately, with retry/backoff policy distinct from routine presence.
- Shock/impact event: immediately or near-immediately, unfiltered, with optional compact magnitude/class.
- Light event: on threshold crossing or state transition, not every routine uplink.
- Temperature event: only if a product threshold or diagnostic requirement exists.
- Downlink-counter sync: after a suspected stale `FCntDown` downlink failure, and then rate-limited until any valid downlink is accepted.

Narrow battery-health schema, no subtype:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Schema + event family | High nibble schema version, low nibble `1` battery health |
| 1 | 1 | Event flags | On charge, low battery, charging transition, reserved |
| 2 | 1 | Battery | Battery percentage |
| 3 | 1 | Delta since last | Percentage-point drop since previous health report, saturated |
| 4 | 1 | Age/interval | Hours since previous health report, saturated |

General event schema, used only if FPort 8 carries several event families:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Schema + event family | High nibble schema version, low nibble event family |
| 1 | 1 | Event flags | Event-specific flags |
| 2 | 1 | Battery | Battery percentage for context |
| 3 | 1 | Value 1 | Event-specific compact value |
| 4 | 1 | Value 2 | Event-specific compact value or age |

This is still smaller than carrying temperature, light, and accelerometer on every routine presence frame, and the FPort lets the shim preserve it.

Downlink-counter sync schema, event family `6`:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Schema + event family | High nibble schema version, low nibble `6` downlink-counter sync |
| 1 | 1 | Flags | Bit 0 sync pending, bit 1 stale downlink seen, bits 2-7 reserved |
| 2 | 1 | Battery | Battery percentage for context |
| 3 | 4 | Device `FCntDown` | Last accepted device downlink counter, `uint32_t` little-endian |

The active LNS must treat the reported counter as the device's last accepted downlink counter and update only upward. Its next downlink must use a counter greater than the reported value, then pending downlink queue entries should be flushed or regenerated. This schema is symmetric: it applies to Relay/Master and Master/Relay authority changes.

### FPort 6 CREW_UNCERTAIN_COMPACT

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Subtype | Uncertain compact subtype, retained because FPort 6 also carries MOB schemas |
| 1 | 1 | Schema + phase | High nibble schema version, low nibble uncertain phase |
| 2 | 1 | Event flags | Same event bit semantics as routine |
| 3 | 1 | Battery | Battery percentage |
| 4 | 1 | Location age | Minutes, `255` unknown |
| 5 | 1 | Attempt flags | BLE attempted, WiFi attempted, GNSS attempted, local RF failed, GNSS failed |
| 6 | 1 | Reason | No approved BLE, WiFi provisional rejected at cap, GNSS unavailable, timeout, reserved |

Expected size: 7 bytes.

### FPort 6 CREW_SOS_CONTEXT

SOS uses BLE/WiFi as fast local evidence, but it must also preserve GNSS state.
When a user presses SOS, the backend needs to know whether the tag has an
accepted local vessel-presence hint, whether GNSS was attempted, and whether a
valid GNSS fix was available by the time the fast local checks completed. A
GNSS-attempted/no-fix state is operationally useful as "uncertain", but it is
not MOB proof until a valid GNSS fix allows distance from the vessel to be
calculated.

SOS is time critical. GNSS must be initialized and available for SOS-capable
firmware even when routine positioning prefers BLE/WiFi first. GNSS must be
powered as soon as the SOS run is accepted, before BLE/WiFi scans start, so the
receiver can search while local positioning checks run. The first SOS uplink
must not wait for a full GNSS timeout after BLE/WiFi completes. At local scan
completion, firmware samples the current GNSS state once:

- If a valid fix is already available, include the GNSS extension in the same
  SOS context uplink.
- If no valid fix is available, set the GNSS-attempted and no-fix-yet flags and
  send immediately.
- Stop or pause GNSS before LoRa TX so AG3335 and LR11xx high-current paths do
  not overlap.

Do not send a separate interim MOB/no-fix uplink for SOS. SOS can describe an
onboard emergency, an on-dock emergency within BLE range, or an actual MOB. The
first alert should therefore be one SOS context uplink that carries the local
evidence and the current GNSS state without conflating SOS with MOB/PIW.

Use a dedicated FPort 6 subtype `0x23` for SOS context. Do not reuse `0x22`
`MOB no-fix` for SOS no-fix, because that conflates a user SOS with a
MOB/PIW separation state.

Operational timing:

| Step | Behavior |
|------|----------|
| SOS accepted | Start GNSS immediately if it is not already active. SOS-capable firmware must initialize GNSS before this point, even if BLE/WiFi are preferred for routine/local presence. |
| Local checks | Run the configured BLE/WiFi scan order normally while GNSS continues searching. |
| Local checks complete | Sample current GNSS quality once; do not wait for an additional GNSS timeout. |
| Before LoRa TX | Stop prestarted GNSS, or pause background GNSS, then send the FPort 6 emergency burst. |
| After TX | Normal charger/background GNSS policy may restart maintenance if applicable. |

Common prefix:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Subtype | `0x23` |
| 1 | 1 | Schema + phase | High nibble schema version, low nibble SOS phase |
| 2 | 1 | Event flags | Same event bit semantics as routine; SOS bit must be set |
| 3 | 1 | Battery | Battery percentage |
| 4 | 1 | Local source flags | Bits 0-2 source type, bit 3 accepted/matched, bit 4 RSSI present, bit 5 identity present |
| 5 | 1 | Evidence flags | BLE attempted, WiFi attempted, GNSS attempted, local source accepted, GNSS fix valid, GNSS quality OK, no-fix-yet |
| 6 | 1 | Reason | None, no approved BLE, WiFi provisional rejected, GNSS unavailable, GNSS timeout, GNSS no-fix-yet |

Evidence flag allocation:

| Bit | Meaning |
|-----|---------|
| 0 | BLE attempted |
| 1 | WiFi attempted |
| 2 | GNSS attempted |
| 3 | Local source accepted/matched |
| 4 | GNSS fix valid |
| 5 | GNSS quality OK |
| 6 | No fix yet / uncertain |
| 7 | Reserved |

Local source type values:

| Value | Meaning |
|-------|---------|
| 0 | None |
| 1 | BLE iBeacon |
| 2 | WiFi fixed/global |
| 3 | WiFi provisional |
| 4 | GNSS only |

Optional BLE extension, present when source type is BLE and identity-present is set:

| Size | Field | Encoding |
|------|-------|----------|
| 2 | iBeacon Major | `uint16_t`, little-endian |
| 2 | iBeacon Minor | `uint16_t`, little-endian |
| 1 | RSSI | Optional signed int8 dBm when RSSI-present is set |

Optional WiFi extension, present when source type is WiFi and identity-present is set:

| Size | Field | Encoding |
|------|-------|----------|
| 6 | BSSID | Raw MAC bytes |
| 1 | WiFi flags | Origin/provisional detail |
| 1 | RSSI | Optional signed int8 dBm when RSSI-present is set |

Optional GNSS extension, present only when GNSS-fix-valid is set:

| Size | Field | Encoding |
|------|-------|----------|
| 4 | Latitude | `int32_t`, little-endian, degrees x1e6 |
| 4 | Longitude | `int32_t`, little-endian, degrees x1e6 |
| 1 | HDOP | HDOP x10 |
| 1 | Satellites | SV count used for fix |

Quality semantics:

- GNSS attempted means the firmware powered or reused AG3335 for the SOS path.
- GNSS fix valid means `gnss_get_quality_fix()` reported a valid receiver fix at
  local scan completion.
- GNSS quality OK means the valid fix also met the configured HDOP/HACC
  thresholds used for PIW/MOB quality decisions.
- No-fix-yet means GNSS was attempted but no valid fix was available when the
  first SOS context uplink was sent.

Decoder contract:

- Always emit a `4200` Event Status measurement with `sosActive=true` for
  `0x23`, so the existing relay flow makes the tag visible and records the SOS.
- Always emit a `6000` Crew Alert measurement for `0x23`.
- If GNSS was attempted but no valid fix is available, set
  `gpsValid=false`, `gnssAttempted=true`, `gnssFixValid=false`,
  `noFixYet=true`, and `uncertain=true` when there is no accepted BLE/WiFi
  local source.
- Do not emit latitude/longitude measurements unless GNSS-fix-valid is set.
  This preserves the existing "No GPS fix" relay path and avoids fabricating
  separation proof.
- When GNSS-fix-valid is set, emit the normal latitude/longitude measurements
  so Node-RED can calculate vessel separation and promote to MOB only when the
  configured distance threshold is exceeded.
- Keep `mobConfirmed=false` for SOS context. MOB confirmation is a downstream
  decision based on valid GNSS separation, not the presence of SOS alone.
- Preserve `sosActive=true` independently of MOB/PIW state. Node-RED and the
  webapp should not treat SOS no-fix as a MOB no-fix event unless a later valid
  GNSS/vessel-separation decision promotes it.

Expected sizes:

- No local source and no GNSS fix: 7 bytes.
- BLE source plus no GNSS fix: 12 bytes with RSSI.
- WiFi source plus no GNSS fix: 15 bytes with WiFi flags and RSSI.
- No local source plus GNSS fix: 17 bytes.
- BLE source plus GNSS fix: 22 bytes with RSSI.
- WiFi source plus GNSS fix: 25 bytes with WiFi flags and RSSI.

### FPort 6 MOB_POSITION Movement Extension

Existing `0x20` MOB position reports are 13 bytes. Add three bytes for COG/SOG:

| Field | Size | Encoding | Description |
|-------|------|----------|-------------|
| COG | 2 | `uint16_t`, little-endian, degrees x2 | Course over ground at 0.5 degree resolution, `0..719`; `0xFFFF` unknown |
| SOG | 1 | `uint8_t`, 0.1 m/s | Speed over ground, `0..253`; `0xFE` saturated at `>=25.4 m/s`; `0xFF` unknown |

Expected position payload size with extension: 16 bytes.

COG/SOG are firmware-derived from a recent history of valid PIW fixes, not receiver-derived from instantaneous GNSS RMC/VTG. The firmware keeps a configurable sliding window, estimates each fix's horizontal uncertainty from HACC or HDOP, and fits a weighted east/north velocity line. Poor fixes are retained with lower weight because an older poor fix can provide a more useful velocity baseline than a recent high-quality fix. Implausible fixes are rejected against the fitted track residual, not against the immediately previous fix. Quality flag bit 3 indicates that the extension contains a valid firmware-derived vector; when it is clear, COG is `0xFFFF` and SOG is `0xFF`.

### FPort 11 CREW_LOCAL_FINGERPRINT

Use this only when the deployment or backend explicitly needs multi-record RSSI data.

Common prefix:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Schema + phase | High nibble schema version, low nibble phase |
| 1 | 1 | Event flags | Same event bit semantics |
| 2 | 1 | Battery | Battery percentage |
| 3 | 1 | Location age | Minutes |
| 4 | 1 | Record count | Number of following records |

BLE fingerprint record:

| Size | Field | Description |
|------|-------|-------------|
| 1 | Record type | `0x01` BLE iBeacon |
| 2 | Major | iBeacon Major |
| 2 | Minor | iBeacon Minor |
| 1 | RSSI | signed int8 dBm |

WiFi fingerprint record:

| Size | Field | Description |
|------|-------|-------------|
| 1 | Record type | `0x02` WiFi |
| 6 | BSSID | WiFi MAC |
| 1 | RSSI | signed int8 dBm |
| 1 | Flags | channel/origin/provisional detail, exact bit split TBD |

Expected sizes:

- Three BLE records: 24 bytes.
- Six BLE records: 42 bytes.
- Three WiFi records: 33 bytes.
- Mixed records scale linearly and should be gated by data rate, interval, and explicit configuration.

## Airtime Impact

Approximate LoRa airtime for 125 kHz, CR 4/5, explicit header, CRC on, 8-symbol preamble, comparing application payload bytes:

| Payload | Bytes | SF7 | SF12 |
|---------|-------|-----|------|
| Compact no-source routine | 5 | ~31 ms | ~827 ms |
| Compact battery health | 5 | ~31 ms | ~827 ms |
| Compact BLE strongest | 10 | ~41 ms | ~991 ms |
| Compact WiFi strongest | 13 | ~46 ms | ~1155 ms |
| Current custom BLE, 1 beacon, no accel | 15 | ~46 ms | ~1155 ms |
| Current custom BLE, 3 beacons, no accel | 25 | ~62 ms | ~1483 ms |
| Current WiFi, padded to 3 APs, no accel | 31 | ~72 ms | ~1810 ms |
| SOS context, no local source, no GNSS fix | 7 | ~36 ms | ~827 ms |
| SOS context, BLE source, no GNSS fix | 12 | ~46 ms | ~1155 ms |
| SOS context, BLE source plus GNSS fix | 22 | ~57 ms | ~1319 ms |
| Current MOB position | 13 | ~46 ms | ~1155 ms |
| MOB position with COG/SOG | 16 | ~51 ms | ~1319 ms |

The biggest immediate win is WiFi: a routine accepted/provisional WiFi presence frame drops from about 31 bytes to about 13 bytes when only one AP is needed. BLE also improves when three beacons are sent today, but the more important product win is making one-strongest-beacon the default contract while reserving an expanded fingerprint port for future positioning. Moving battery/light/shock into low-rate event traffic also removes pressure to keep sending environmental fields on every routine presence frame.

## SF9 Regional Payload Ceiling

Mesh transmission uses SF9, so any unfragmented payload that must remain valid across supported LoRaWAN regions should fit the SF9 application payload limit for the selected region and dwell-time setting.

The LoRaWAN Regional Parameters tables define:

- `M`: maximum MACPayload size.
- `N`: maximum application payload when FOpts is empty.
- Effective application limit: `N - FOptsLen`.

The local LoRa Basics Modem implementation enforces the same rule as `application_size + tx_fopts_current_length <= M - 8`.

SF9 uplink ceilings from the regional tables:

| Region | SF9 uplink DR | Repeater-compatible `M` | `N` with empty FOpts | `N` with 15-byte FOpts | Notes |
|--------|---------------|-------------------------|----------------------|------------------------|-------|
| EU863-870 | DR3 | 123 | 115 | 100 | Also applies to EU433-style SF9 DR3 sizing |
| US902-928 | DR1 | 61 | 53 | 38 | Constraining US case |
| AU915-928, dwell off | DR3 | 123 | 115 | 100 | Only after dwell-time state permits |
| AU915-928, 400 ms dwell | DR3 | 61 | 53 | 38 | Constraining AU boot/dwell case |
| CN470-510 | DR3 | 192 | 184 | 169 | Least constraining at SF9 |
| AS923, dwell off | DR3 | 123 | 115 | 100 | Only after dwell-time state permits |
| AS923, 400 ms dwell | DR3 | 61 | 53 | 38 | Constraining AS923 boot/dwell case |
| KR920-923 | DR3 | 123 | 115 | 100 | LBT-based regional limit |
| IN865 | DR3 | 123 | 115 | 100 | No dwell-time limit in the regional table |
| RU864-870 | DR3 | 123 | 115 | 100 | No dwell-time limit in the regional table |

Design rule:

- Any payload that must work at SF9 in all supported regions should be no more than 53 bytes when FOpts is expected to be empty.
- Safety-critical compact payloads should target 38 bytes or less so they still fit when the MAC layer has up to 15 bytes of FOpts pending.
- Expanded RF fingerprint payloads can exceed 53 bytes only when explicitly region-gated, DR-gated, fragmented, or sent through a path that is not constrained to SF9 mesh delivery.

## Decoder Slimming

Decoder work should be staged after the compact payload contract is accepted:

- Add explicit FPort-based decode paths for compact presence, health/event, uncertain, and fingerprint payloads.
- Keep legacy `0x1E` and `0x20`-`0x28` decoders temporarily for migration and field-log replay.
- Split FPort 6 alert decoding from routine decoding permanently; numeric Data IDs overlap with stock routine IDs.
- Decode narrow FPort schemas without requiring a Data ID byte.
- Remove unused stock setting decoders and stock frame helpers that are not emitted by this firmware.
- Replace string-slicing length inference with byte-array parsing for new compact formats.
- Keep semantic booleans in output: `onCharge`, `sosActive`, `userTriggered`, `gnssReady`, `sourceType`, `sourceAccepted`, `wifiProvisional`.
- Preserve raw bytes and schema version in decoded output so field logs remain debuggable.

## Migration Plan

1. Add compact FPort-based decoder support while legacy payloads remain active.
2. Reserve and configure the new unfiltered health/event FPort in the gateway shim.
3. Add firmware builders for compact presence and health/event payloads behind a compile-time or config flag.
4. Switch routine BLE and WiFi success paths to compact presence on FPort 5/7 in test firmware.
5. Stop WiFi padding once the backend accepts compact presence.
6. Add low-rate battery health on the event FPort using a 12-hour or configured battery-drop trigger.
7. Add shock/light event reports only when event thresholds and backend behavior are agreed.
8. Keep existing MOB `0x20`/`0x21`/`0x22` FPort 6 payloads unchanged.
9. Emit SOS as FPort 6 `0x23` SOS_CONTEXT: prestart GNSS on SOS entry, sample
   the current fix when BLE/WiFi finish, and send one combined SOS context
   uplink without waiting for a post-scan GNSS timeout.
10. Add expanded RF fingerprint payloads only when an indoor-positioning backend path exists.
11. After field validation, remove unused stock decoder branches and update `UPLINK_PAYLOADS.md` to make compact payloads primary and legacy payloads historical.

## Alternatives Considered

- Keep stock payloads and only reduce configured scan counts.
- Reuse `0x27` and `0x28` for shorter BLE-only frames.
- Make a fully generic TLV payload for every routine uplink.
- Keep a Data ID byte in every compact payload instead of using FPort as the top-level schema.
- Allocate a separate FPort for every single event type, such as battery, shock, light, and temperature.
- Send no RF identity for onboard presence, only source flags.
- Immediately build an indoor-positioning payload with many BLE/WiFi records.
- Send a second interim no-fix uplink for SOS before local evidence is known.
- Wait for a full GNSS timeout after BLE/WiFi completes before sending SOS.

## Reason for Decision

- A RemEX-native compact family makes the product state explicit instead of encoding it through stock sensor payload names.
- FPort-based typing saves one byte on common compact schemas and gives the gateway shim reliable routing without decrypting or decoding payload content.
- Low-rate health/event traffic gives battery, shock, and light changes a preserved path to the master LNS without bloating routine presence.
- Strongest BLE beacon and strongest accepted/provisional WiFi source match the current product behavior.
- SOS gets maximum useful GNSS search time by powering GNSS at SOS acceptance,
  but it still sends the first alert as soon as fast local evidence is complete.
- FPort policy remains simple and unchanged.
- The common prefix keeps the frequent fields stable while source extensions avoid spending bytes on absent data.
- A dedicated fingerprint port gives a clear future path for RSSI fingerprinting without taxing the current routine heartbeat.
- Decoder slimming becomes safer once the emitted payload set is small and explicit.

## Known Costs

- Backend, Node-RED, and any app ingest code must understand the new FPort schemas before firmware defaults can switch.
- MDR-018 and `crew_lorawan_ports.h` must be updated before new event/fingerprint FPorts are emitted.
- Gateway shim allow/filter rules must be deployed before relying on FPort 8 health/event delivery.
- New compact payloads reduce passive environmental telemetry unless temperature, light, or accelerometer vectors are added as explicit future extensions.
- WiFi flags need a final bit allocation before implementation.
- Legacy and compact decoders will coexist during migration.

## Mitigations

- Keep legacy emission behind a config fallback until compact ingest is proven.
- Preserve raw payload bytes, schema version, FPort, and source flags in decoded records.
- Keep temperature/light/raw accelerometer available as future optional extension records if a product requirement appears.
- Use expanded RF fingerprint payloads only when explicitly enabled and preferably only at suitable data rates or intervals.
- Keep FPort 8 event scheduling rate-limited and threshold-driven so unfiltered delivery does not become a high-volume bypass of routine sampling.

## Related Notes

- [[MDR-012: BLE Hint and LinkCheck Based Crew Tag DR Strategy]]
- [[MDR-018: Crew Tag FPort and Uplink Policy]]
- [[MDR-019: Crew Tag Charger State Routing and POB Counting]]
- [[MDR-020: Crew Tag Geolocation Method Order and WiFi Presence Detection]]
- [[MDR-023: LNS FCntDown Synchronization for Crew Tags]]
- `UPLINK_PAYLOADS.md`
- `RemEX_T1000E_Decoder.js`
- `apps/examples/11_lorawan_tracker/main_lorawan_tracker.c`
- `t1000_e/tracker/src/app_ble_all.c`
- `t1000_e/peripherals/src/wifi_helpers.c`

#mob #mob/mdr #mob/lorawan #mob/crew-tags #payload #ble #wifi
