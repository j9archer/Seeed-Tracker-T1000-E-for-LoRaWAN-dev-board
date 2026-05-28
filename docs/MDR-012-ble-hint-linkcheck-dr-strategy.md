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
- Treat a valid Minor value as the LoRaWAN field tester's proposed DR. The firmware applies that DR directly, clamped to the configured regional/user DR range, until LinkCheckReq fine-tunes it.

Minor mapping:

- Minor 1: propose DR1, worst accepted vessel connection.
- Minor 2: propose DR2.
- Minor 3: propose DR3.
- Minor 4: propose DR4.
- Minor 5: propose DR5, best accepted vessel connection.

Beacon identity convention:

- Use iBeacon Major as the commissioned beacon identity using `MNNNN`.
- `M` is a single decimal model/location-class digit, such as interior versus deck beacon type.
- `NNNN` is the decimal beacon sequence number within that model class.
- Keep iBeacon Minor reserved for the proposed DR above.
- Keep location, vessel, deck, zone, and human-readable label mapping in the backend database when required.
- Firmware must not apply behavior based on the Major naming convention; it treats Major as beacon identity metadata only.
- Normal operation should remain passive BLE scanning. Do not require BLE local-name scan responses for production behavior.

Movement and hysteresis:

- Each BLE scan is treated as a per-uplink snapshot.
- Movement is based on the strongest approved iBeacon identity/location, not only on valid Minor values.
- A change in strongest approved iBeacon must be confirmed across a configurable number of scan sessions before it resets the movement/stability baseline.
- If the new accepted BLE Minor proposes a lower DR than the current DR, apply it immediately.
- If the new accepted BLE Minor proposes a higher DR than the current DR, require confirmation across a configurable number of later scan sessions or wait for LinkCheck validation.
- If the strongest approved beacon changes but has no valid Minor, accept it as movement after hysteresis but keep the current DR until LinkCheck or a valid Minor changes it.
- If the strongest approved beacon changes but proposes the same DR, update the beacon identity after hysteresis but do not force a DR change.
- Stability is based on the approved beacon/location remaining stable. A valid Minor can additionally provide a DR profile, but Minor configuration is not required for movement detection.

LinkCheck behavior:

- Replace confirmed health uplinks as the default vessel RF validation signal.
- When an approved BLE movement/location hint has been stable for a configurable period, issue a LinkCheckReq to validate and fine-tune the setting.
- Add a deterministic DevEUI-derived phase to the stable-location LinkCheck delay so a group of tags that woke together do not all request LinkCheck at the same wall-clock time.
- Reset the periodic LinkCheck counter after any early LinkCheck forced by a stable BLE hint.
- Continue periodic LinkCheckReqs even when BLE hints are enabled.
- Increase the periodic LinkCheck interval when approved BLE hints are present, because the beacon hint provides local movement/location context.
- Phase periodic LinkCheck by DevEUI and purpose, rather than firing on the same uplink count for every device.
- Send forced LinkCheckReq as a dedicated MAC-only uplink so it is not hidden inside an application uplink that a gateway shim may filter.
- Treat a missing LinkCheckAns as inconclusive, not as weak RF. Only a received LinkCheckAns with low margin may reduce DR.
- If LinkCheckAns is missing, keep the normal FPort 5 application DR unchanged and schedule LinkCheck-only retry probes at lower DRs.
- The first missing-answer retry waits a short time plus DevEUI jitter. Later missing-answer retries are spaced by a larger uplink-count interval plus DevEUI phase.
- Reset the missing-answer retry ladder when movement is accepted, the strongest approved beacon changes, or a LinkCheckAns is received.
- Do not use the retry probe DR as the normal vessel uplink DR. The probe DR exists only to improve the chance of the MAC-only LinkCheckReq reaching the LNS.

DR precedence:

- SOS and MOB burst rules always override BLE and LinkCheck.
- Cancellation uses persistence DR and a confirmed cancellation uplink.
- BLE Minor values 1 through 5 provide immediate proposed vessel/idle DR selection.
- Approved beacons with unconfigured Minor values provide movement/stability context only.
- LinkCheck fine-tunes after the approved beacon/location has been stable.
- LinkCheck-refined DR takes precedence over a stable BLE Minor until movement or a new accepted hint resets the baseline.
- Missing LinkCheckAns does not take precedence over BLE Minor. It may only change future LinkCheck probe DRs, not normal FPort 5 application DR.
- A received strong LinkCheckAns keeps 10 dB as spare link margin, then spends surplus margin in 5 dB chunks per upward DR step.
- All selected DRs are clamped by the configured `lr_DR_min` and `lr_DR_max` range.

Fleet de-synchronisation:

- Use deterministic jitter derived from the whole DevEUI rather than true random values or only the final DevEUI byte.
- Use separate purpose constants for separate schedules so startup, stable-location LinkCheck, retry LinkCheck, and periodic LinkCheck do not bunch together on the same device.
- Add startup jitter before the first routine power/status uplink after boot/join to handle charger-bank wakeups.
- Do not apply startup jitter to SOS, MOB, cancellation, or direct user alert traffic.
- Use uplink-count phase offsets for recurring control work, such as periodic LinkCheck, so tags that woke together do not all request control downlinks on their 10th, 20th, or 60th uplink.

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
- A missing LinkCheckAns can be caused by gateway congestion, relay shim filtering, duplicate handling, downlink scheduling, or LNS path issues. It is therefore unsafe to treat absence of an answer as poor RF for normal application uplinks.
- LinkCheck-only lower-DR probes preserve normal application airtime while still giving the network a more robust chance to answer control validation.
- A 10 dB spare-margin reserve is less conservative than the earlier 15 dB reserve while still leaving allowance above the approximate 2.5-3 dB EU868 sensitivity cost per DR step around SF10..SF7.
- Minor values 1 to 5 keep beacon installation fast when RF profiling is available: use the LoRaWAN field tester to choose the proposed DR, set that DR as the Minor, and mount the beacon.
- Major values keep beacon identity immediately inspectable during field work while preserving Minor for DR hints.
- Minor values are optional for movement detection. A beacon with an approved UUID and default Minor can still indicate that the crew tag has moved through the vessel.
- Snapshot-based hysteresis matches the T1000-E duty cycle better than packet-count hysteresis because BLE scans happen only before uplinks.
- Immediate conservative changes reduce packet loss when a crew member enters a poor RF area.
- Delayed less-conservative changes avoid oscillation when BLE RSSI fluctuates.
- The decision supports [[Design Principles]] around operational simplicity, resilient MOB behavior, and conserving airtime/power where possible.
- Deterministic DevEUI jitter addresses fleet-level congestion when many tags wake from charge together or progress through periodic control schedules together.

## Known costs

- BLE beacon placement and Minor configuration become part of vessel commissioning.
- Beacon Major assignment and backend identity mapping become part of vessel commissioning.
- Incorrect beacon Minor values can cause suboptimal DR choices.
- Approved beacons with default Minor values can trigger movement/stability changes and therefore LinkCheck timing changes, even though they do not directly set DR.
- BLE hints are local estimates and may not reflect current LoRaWAN gateway load, downlink availability, or backhaul state.
- LinkCheckReq still depends on the LNS that terminates the LoRaWAN session.
- LinkCheck-only retry probes consume additional airtime, especially at low DRs.
- DevEUI-derived schedules are stable across reboot, which is desirable for fleet spreading but means two devices with unlucky close phases can remain close until the configured purpose/window changes.
- If no approved beacon is visible, the tag must fall back to LinkCheck/current baseline behavior.
- TX power is not optimized by the initial design.

## Mitigations

- Only approved UUIDs are used for movement hints, DR hints, and BLE-driven LinkCheck timing.
- Only Minor values 1 through 5 are valid for direct DR selection.
- Unknown, default, or out-of-range Minor values are ignored for direct DR selection but still count as movement/location hints when the UUID is approved.
- DR values are always clamped to `lr_DR_min` and `lr_DR_max`.
- Lower-DR BLE hints apply immediately, while higher-DR hints require confirmation or LinkCheck validation.
- LinkCheckReq is run after the approved beacon/location has been stable for a configurable period.
- Periodic LinkCheckReq remains enabled, with a longer interval when approved BLE hints are present.
- Stable-location and periodic LinkCheck schedules are DevEUI-phased.
- Missing LinkCheckAns retry probes step down separately from the normal application DR and are spaced with DevEUI jitter/phase to avoid synchronized fleet retries.
- TX power changes remain disabled until field testing justifies enabling them.

## Related notes

- [[System Goals]]
- [[Design Principles]]
- [[Known Limitations and Risk Register]]

#mob #mob/mdr #mob/rf #mob/crew-tags
