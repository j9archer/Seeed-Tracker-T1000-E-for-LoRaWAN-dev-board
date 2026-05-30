---
title: "MDR-018: Crew Tag FPort and Uplink Policy"
type: "mdr"
status: "accepted"
tags:
  - mob
  - mob/mdr
  - mob/lorawan
  - mob/crew-tags
aliases:
  - "MDR-018"
  - "Crew Tag FPort and Uplink Policy"
---

# MDR-018: Crew Tag FPort and Uplink Policy

Parent: [[Major Decision Records Index]]

## Status

Accepted

## Context

Crew-tag uplinks pass through a relay gateway shim before reaching the master LNS. The shim can sample routine traffic but must not drop alert, uncertain, or MOB traffic. FPort therefore carries routing intent as well as application payload type.

The tag also sends startup/status and periodic telemetry that can wake in large groups, especially when many uncharged tags are placed on a charger and then powered together. These routine messages must not be misclassified as alerts.

## Decision

FPort policy:

- FPort 0 is reserved for LoRaWAN MAC-only tasks such as LinkCheckReq and DeviceTimeReq.
- FPort 5 is routine crew-tag traffic and may be sampled by the relay shim.
- FPort 6 is alert/pass-through traffic and must be forwarded by the relay shim without nth filtering.
- FPort 7 is routine on-charge/spare crew-tag traffic and may use a separate nth filtering policy.
- FPort 10 is reserved for gateway assistance downlinks.

Routine FPort 5 traffic:

- Includes periodic tracker telemetry, BLE/WiFi/GNSS scan results, sensor readings, and the power-on/status payload.
- Excludes routine traffic from tags currently on charge; those packets use FPort 7 per MDR-019.
- May include accepted on-vessel context such as BLE beacon records.
- Appends `locationAgeMin` only on normal on-vessel uplinks where the field is useful.
- Does not use `locationAgeMin` on alert/MOB uplinks, because those already carry their own GNSS/MOB state when available.

Routine on-charge FPort 7 traffic:

- Includes periodic tracker telemetry, sensor/location scan results, and power-on/status payloads while charger detect is active.
- Uses the same payload body formats as routine traffic unless the payload type has no event-state byte.
- Sets event-state bit `0x04` on routine sensor/location uplinks that carry an event-state byte.
- Is intended for spare/unused tags and may use a separate relay/backend filtering policy from active routine FPort 5 traffic.
- Must never take precedence over FPort 6 alert routing.

Alert FPort 6 traffic:

- Includes MOB position, MOB no-fix, MOB cancellation, SOS-triggered alert traffic, and other pass-through alert or uncertain state traffic.
- Must not be nth-filtered by the relay shim.
- May use a multi-DR initial burst for MOB/SOS announcement.
- Takes precedence over charger routing; SOS/MOB traffic from an on-charge tag still uses FPort 6 and carries charger state as metadata where available.

Power-on/status payload:

- The power-on/status payload is normally routine FPort 5 traffic.
- If charger detect is active, the power-on/status payload is routine on-charge FPort 7 traffic.
- The battery byte must not be interpreted as an event-state byte. A battery value such as `100` is `0x64`, which includes bit `0x40`; using that byte as an SOS flag falsely routes the status payload to FPort 6.
- Startup/status uses routine scheduling and is not emergency-priority traffic.

Fleet de-synchronisation:

- Add deterministic DevEUI-derived jitter before the first routine power/status uplink after boot/join.
- Use the whole DevEUI hash rather than only the final byte so the distribution is less likely to bunch.
- Do not delay SOS, MOB, cancellation, or direct user alert traffic.
- Use separate purpose constants when deriving jitter/phase for different subsystems so startup, LinkCheck, and other control tasks do not align on the same device.

Gateway shim policy:

- Allow FPort 0.
- Allow FPort 5 with configured nth filtering for routine traffic.
- Allow FPort 6 without nth filtering.
- Allow FPort 7 with independently configured nth filtering for on-charge/spare routine traffic.
- Allow joins only from approved DevEUI prefixes where that policy is required by the deployment.
- Default action may remain `drop` if the explicit allow rules above are present.

## Reason for decision

- FPort separation gives the gateway shim a simple deterministic rule: routine traffic may be sampled, alert traffic must pass.
- A separate charger FPort lets spare/charger-bank traffic be sampled differently from active worn-tag routine traffic.
- Keeping startup/status on FPort 5 prevents routine boot traffic from bypassing shim sampling.
- Keeping `locationAgeMin` on routine on-vessel uplinks makes it useful for normal telemetry without wasting alert airtime.
- Deterministic DevEUI jitter reduces charger-bank congestion while remaining repeatable and debuggable.

## Known costs

- FPort 5 sampling means the backend may not see every routine telemetry frame.
- FPort 7 filtering means the backend may not see every spare/on-charge telemetry frame.
- FPort 6 must be protected from accidental use by routine payloads.
- DevEUI-derived jitter can delay first routine status visibility by the configured maximum.
- LinkCheckReq behavior still depends on the LoRaWAN modem and LNS handling of MAC-only traffic.

## Mitigations

- Keep FPort constants in `crew_lorawan_ports.h`.
- Decode FPort 6 alert Data IDs separately from routine FPort 5 Data IDs that share numeric values.
- Treat the power/status payload as a distinct Data ID before evaluating event-state/SOS bits.
- Route charger-state routine traffic only after alert/SOS/MOB precedence has been checked.
- Log startup jitter, selected FPort, LinkCheck probe DR, and application DR so field traces show the routing and scheduling decisions.

## Related notes

- [[MDR-012: BLE Hint and LinkCheck Based Crew Tag DR Strategy]]
- [[MDR-020: Crew Tag Geolocation Method Order and WiFi Presence Detection]]
- [[MDR-019: Crew Tag Charger State Routing and POB Counting]]
- `UPLINK_PAYLOADS.md`
- `t1000_e/tracker/inc/crew_lorawan_ports.h`

#mob #mob/mdr #mob/lorawan #mob/crew-tags
