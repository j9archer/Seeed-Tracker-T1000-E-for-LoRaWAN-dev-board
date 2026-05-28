---
title: "MDR-019: Crew Tag Charger State Routing and POB Counting"
type: "mdr"
status: "accepted"
tags:
  - mob
  - mob/mdr
  - mob/lorawan
  - mob/crew-tags
aliases:
  - "MDR-019"
  - "Crew Tag Charger State Routing and POB Counting"
---

# MDR-019: Crew Tag Charger State Routing and POB Counting

Parent: [[Major Decision Records Index]]

## Status

Accepted

## Context

Crew tags left on a charger for a prolonged period are likely spare or unused tags rather than tags currently worn by persons on board. Charger banks can also wake many tags together, creating routine telemetry bursts that are less operationally important than active crew traffic.

The firmware can detect charger presence with `CHARGER_ADC_DET`. The existing payload format has an `Event State` byte on normal sensor/location uplinks, and bit `0x04` is reserved in the payload documentation for `On Charge`.

MOB/SOS handling must remain conservative. A tag that appears to be on charge could still produce a real emergency signal, or charger detection could be affected by salt water across the charging contacts. Charger state should therefore annotate emergency handling, not suppress it.

## Decision

Charger state is a live device state, not a one-shot event.

Firmware behavior:

- Sample charger state at payload-build/routing time.
- Set `Event State` bit `0x04` on every routine sensor/location uplink while charger detect is active.
- Compose live state bits into the outgoing event byte, instead of storing charger state in the transient `event_state` that is reset after transmit.
- Keep SOS/user/MOB event precedence over charger routing.
- Send routine on-charge uplinks on FPort 7.
- Send SOS, MOB, cancellation, and uncertain alert traffic on FPort 6 even when charger detect is active.
- Include the on-charge event bit in FPort 6 alert/MOB payloads where the payload format has an event-state field or can otherwise carry charger metadata.
- Route power-on/status payload `0x1E` on FPort 7 when charger detect is active, even though `0x1E` does not contain an event-state byte.

Decoder behavior:

- Decode event bit `0x04` as `onCharge`.
- Preserve the raw event-state byte for diagnostics.
- Expose semantic booleans such as `onCharge`, `sosActive`, and `userTriggered`, rather than relying only on text event names.
- Do not map bit `0x04` to motionless or any other legacy meaning in the crew-tag decoder path.
- For MOB/SOS records, include `onCharge=true` when charger state is present so downstream systems can show the warning without suppressing the alert.

Node-RED and SAR Web App behavior:

- Continue processing MOB/SOS/FPort 6 traffic from on-charge tags.
- Highlight on-charge emergency records prominently as `On charger` or `Charge pins active`.
- Do not automatically exclude an on-charge tag from MOB response in the initial implementation.
- Keep a future policy hook for excluding on-charge tags from MOB alert generation, but default it off.
- Count routine on-charge tags separately from active POB.
- Display POB as an active count plus a charger/spare count, for example `POB: 15 active / 5 charging`.
- Update the charger/spare bucket from routine FPort 7 traffic and any alert traffic that carries `onCharge=true`.
- Keep a tag in the charger/spare bucket until it reports off charge or its state times out.

Gateway and relay behavior:

- Allow FPort 7 through the relay shim.
- Apply charger/spare nth filtering independently from normal routine FPort 5 filtering.
- Never apply charger/spare filtering to FPort 6 alert traffic.
- Treat FPort 7 as routine traffic for priority purposes, but preserve enough traffic to maintain charger/spare counts and device health visibility.

## Reason for decision

- Separating on-charge routine traffic gives the relay shim and backend a simple way to reduce noise from spare tags without weakening alert handling.
- Keeping FPort 6 as the highest-priority alert path avoids missing MOB/SOS events caused by wet contacts, charger-state false positives, or unusual operational cases.
- Encoding charger state in the event byte lets backends and UIs make consistent decisions from the payload, while FPort 7 lets infrastructure apply different filtering.
- Splitting POB into active and charging counts better reflects operational reality than treating every online tag as a person currently on board.

## Known costs

- FPort 7 must be added to firmware constants, relay shim allow rules, decoders, Node-RED flows, and SAR Web App ingest logic before rollout.
- Existing decoders that only expect FPort 5 and FPort 6 may ignore charger traffic until updated.
- Charger detect can flap or false-trigger in wet/salty conditions.
- POB count semantics become more nuanced: total online tags may no longer equal active POB.

## Mitigations

- Keep the payload body format unchanged for routine sensor/location uplinks; only the event bit and FPort routing change.
- Log selected FPort and sampled charger state during uplink routing.
- Consider short debounce/stability handling for routine charger routing, while never delaying or suppressing SOS/MOB alerts.
- Keep `exclude_on_charge_from_mob` or equivalent future policy disabled by default.
- Label POB counts clearly in UI copy, such as `active / charging`, rather than showing an ambiguous bare fraction.

## Related notes

- [[MDR-018: Crew Tag FPort and Uplink Policy]]
- `UPLINK_PAYLOADS.md`
- `docs/background-gnss-charging.md`
- `t1000_e/tracker/inc/crew_lorawan_ports.h`

#mob #mob/mdr #mob/lorawan #mob/crew-tags
