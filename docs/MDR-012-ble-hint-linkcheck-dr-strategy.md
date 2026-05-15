---
title: "MDR-012: BLE Hint and LinkCheck Based Crew Tag DR Strategy"
type: "mdr"
status: "accepted"
tags:
  - mob
  - mob/mdr
  - mob/rf
  - mob/crew-tags
aliases:
  - "MDR-012"
  - "BLE Hint and LinkCheck Based Crew Tag DR Strategy"
---

# MDR-012: BLE Hint and LinkCheck Based Crew Tag DR Strategy

Parent: [[Major Decision Records Index]]

## Status

Accepted

## Context

Crew tags move through very different RF environments inside a vessel. A tag may move from a high-margin bridge area to a low-margin engine-room area between normal uplinks. Standard ADR is not a good fit because crew tags are mobile, uplinks are sparse, and recovery scenarios value fresh position density more than repeated MAC-layer redundancy.

Confirmed uplinks can indicate whether an ACK was received, but they provide only a binary result and depend on whichever LNS terminates the LoRaWAN session. LinkCheckReq provides a more useful network-side signal because it returns demodulation margin and gateway count.

BLE beacons can provide fast local RF hints before each uplink. The T1000-E does not continuously listen for beacons; it scans only before an uplink, typically for about 6 seconds, then sends and sleeps. Because this is a periodic snapshot rather than continuous tracking, continuous RSSI packet-count hysteresis is not appropriate. Any hysteresis must operate across complete scan sessions.

## Decision

Use approved BLE beacon observations as the fast movement/location signal, use valid BLE Minor values as the optional direct DR hint, and use LinkCheckReq as the slower validation and fine-tuning mechanism.

BLE hint behavior:

- Only consider iBeacons with approved UUIDs. Unapproved BLE devices must not influence movement detection, DR selection, or LinkCheck timing.
- Select the strongest approved iBeacon from the current BLE scan as the movement/location hint.
- Use the strongest approved iBeacon for movement/stability even when its Minor is not configured.
- Only use beacon Minor values from 1 to 5 for direct DR selection.
- Treat default or out-of-range Minor values, such as 19641, as movement/location signals only. They must not directly set DR.
- Treat a valid Minor value as an RF profile, not as a raw margin value.

Minor mapping:

- Minor 1: excellent RF, use normal/max DR.
- Minor 2: good RF, use normal/max DR.
- Minor 3: medium RF, use persistence DR.
- Minor 4: weak RF, use persistence DR.
- Minor 5: poor RF, use SOS-low/minimum-safe DR.

Movement and hysteresis:

- Each BLE scan is treated as a per-uplink snapshot.
- Movement is based on the strongest approved iBeacon identity/location, not only on valid Minor values.
- A change in strongest approved iBeacon must be confirmed across a configurable number of scan sessions before it resets the movement/stability baseline.
- If the new accepted BLE profile is more conservative than the current profile, apply it immediately.
- If the new accepted BLE profile is less conservative than the current profile, require confirmation across a configurable number of later scan sessions or wait for LinkCheck validation.
- If the strongest approved beacon changes but has no valid Minor, accept it as movement after hysteresis but keep the current DR until LinkCheck or a valid Minor changes it.
- If the strongest approved beacon changes but maps to the same DR profile, update the beacon identity after hysteresis but do not force a DR change.
- Stability is based on the approved beacon/location remaining stable. A valid Minor can additionally provide a DR profile, but Minor configuration is not required for movement detection.

LinkCheck behavior:

- Replace confirmed health uplinks as the default vessel RF validation signal.
- When an approved BLE movement/location hint has been stable for a configurable period, issue a LinkCheckReq to validate and fine-tune the setting.
- Reset the periodic LinkCheck counter after any early LinkCheck forced by a stable BLE hint.
- Continue periodic LinkCheckReqs even when BLE hints are enabled.
- Increase the periodic LinkCheck interval when approved BLE hints are present, because the beacon hint provides local movement/location context.
- Send forced LinkCheckReq as a dedicated MAC-only uplink so it is not hidden inside an application uplink that a gateway shim may filter.

DR precedence:

- SOS and MOB burst rules always override BLE and LinkCheck.
- Cancellation uses persistence DR and a confirmed cancellation uplink.
- BLE Minor values 1 through 5 provide immediate vessel/idle DR selection.
- Approved beacons with unconfigured Minor values provide movement/stability context only.
- LinkCheck fine-tunes after the approved beacon/location has been stable.
- All selected DRs are clamped by the configured `lr_DR_min` and `lr_DR_max` range.

TX power:

- Do not enable BLE-driven TX power changes initially.
- Leave TX power hinting behind a separate future config flag after field validation of modem TX power offset behavior.

## Alternatives considered

- Use confirmed uplinks for health probing and DR fallback.
- Use LinkCheckReq only, without BLE hints.
- Enter raw demodulation margin, RSSI, or SNR into BLE beacon fields.
- Use BLE RSSI scan-count hysteresis.
- Use BLE hints for both DR and TX power immediately.

## Reason for decision

- BLE hints react quickly when a person moves through the vessel, without waiting for network feedback.
- LinkCheckReq provides network-side demodulation margin and gateway count, which is more useful than a binary confirmed-uplink ACK.
- Minor values 1 to 5 keep beacon installation fast when RF profiling is available: measure LoRa signal, choose a simple profile, set the Minor, and mount the beacon.
- Minor values are optional for movement detection. A beacon with an approved UUID and default Minor can still indicate that the crew tag has moved through the vessel.
- Snapshot-based hysteresis matches the T1000-E duty cycle better than packet-count hysteresis because BLE scans happen only before uplinks.
- Immediate conservative changes reduce packet loss when a crew member enters a poor RF area.
- Delayed less-conservative changes avoid oscillation when BLE RSSI fluctuates.
- The decision supports [[Design Principles]] around operational simplicity, resilient MOB behavior, and conserving airtime/power where possible.

## Known costs

- BLE beacon placement and Minor configuration become part of vessel commissioning.
- Incorrect beacon Minor values can cause suboptimal DR choices.
- Approved beacons with default Minor values can trigger movement/stability changes and therefore LinkCheck timing changes, even though they do not directly set DR.
- BLE hints are local estimates and may not reflect current LoRaWAN gateway load, downlink availability, or backhaul state.
- LinkCheckReq still depends on the LNS that terminates the LoRaWAN session.
- If no approved beacon is visible, the tag must fall back to LinkCheck/current baseline behavior.
- TX power is not optimized by the initial design.

## Mitigations

- Only approved UUIDs are used for movement hints, DR hints, and BLE-driven LinkCheck timing.
- Only Minor values 1 through 5 are valid for direct DR selection.
- Unknown, default, or out-of-range Minor values are ignored for direct DR selection but still count as movement/location hints when the UUID is approved.
- DR values are always clamped to `lr_DR_min` and `lr_DR_max`.
- More conservative BLE hints apply immediately, while less conservative hints require confirmation or LinkCheck validation.
- LinkCheckReq is run after the approved beacon/location has been stable for a configurable period.
- Periodic LinkCheckReq remains enabled, with a longer interval when approved BLE hints are present.
- TX power changes remain disabled until field testing justifies enabling them.

## Related notes

- [[System Goals]]
- [[Design Principles]]
- [[Known Limitations and Risk Register]]

#mob #mob/mdr #mob/rf #mob/crew-tags
