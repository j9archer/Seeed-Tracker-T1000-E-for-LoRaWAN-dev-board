---
title: "MDR-023: LNS FCntDown Synchronization for Crew Tags"
type: "mdr"
status: "accepted"
tags:
  - mob
  - mob/mdr
  - mob/lorawan
  - mob/crew-tags
  - mob/relay-lns
aliases:
  - "MDR-023"
  - "LNS FCntDown Synchronization"
  - "Crew Tag FCntDown Synchronization"
---

# MDR-023: LNS FCntDown Synchronization for Crew Tags

Parent: [[Major Decision Records Index]]

## Status

Accepted.

Implemented in T1000-E firmware and decoder as FPort 8 health/event family `6`.

## Context

Crew Tags can receive downlinks from different LoRaWAN Network Servers depending on vessel connectivity:

- In Isolated Mode, the vessel Relay LNS can be the active downlink authority.
- In Relay/Connected Mode, the Master LNS can be the active downlink authority.

Both LNSs can hold the same deterministic ABP session material for a tag, but their downlink frame-counter state can diverge. When one LNS sends downlinks, the device's internal `FCntDown` advances. If authority later moves to the other LNS, that LNS may still have an older downlink counter or may be at zero on first contact.

The Semtech LoRa Basics Modem stack rejects or fails MIC verification for such downlinks. A stale lower 16-bit `FCntDown` is interpreted as possible 16-bit rollover, so MIC verification uses the wrong reconstructed full counter when the LNS actually signed with an older full counter. The field symptom is a generic bad downlink path such as:

```text
rx_packet_type = 0
Receive a bad packet on RX1
```

`rx_packet_type = 0` is not specific to counter failure; it is the generic `NO_MORE_VALID_RX_PACKET` result.

## Decision

Keep LoRaWAN replay protection on the device. Do not accept stale downlinks and do not reset the device's `FCntDown` as a general handover mechanism.

Instead, synchronize LNS downlink-counter state through a preserved maintenance uplink:

1. The device detects suspected stale downlink-counter failures inside the LoRaWAN stack.
2. The stack latches a `FCntDown` sync-pending flag.
3. The tracker emits a FPort 8 health/event maintenance uplink with the device's last accepted `FCntDown`.
4. The active LNS decoder and repair script update that LNS's downlink counter state only upward.
5. The active LNS flushes or regenerates pending downlink queue entries after the counter repair.
6. The device clears sync-pending only after any valid downlink is accepted.

This applies symmetrically:

- Isolated Mode -> Relay/Connected Mode, where the Master LNS may be stale.
- Relay/Connected Mode -> Isolated Mode, where the vessel Relay LNS may be stale.

Both LNS ingestion paths must therefore decode the maintenance uplink and run the counter-repair logic.

## FPort 8 payload

Use the MDR-018/MDR-022 preserved health/event path:

```text
FPort: 8
Family: 6
```

Payload:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Schema + family | High nibble schema version, low nibble `6` downlink-counter sync |
| 1 | 1 | Flags | Bit 0 sync pending, bit 1 stale downlink seen, bits 2-7 reserved |
| 2 | 1 | Battery | Battery percentage for context |
| 3 | 4 | Device `FCntDown` | Last accepted device downlink counter, `uint32_t` little-endian |

The reported `FCntDown` is the device's last accepted downlink counter. The receiving LNS must treat it as a lower bound and schedule its next downlink with a counter greater than the reported value. The LNS must not reduce its stored counter based on this payload.

## Implementation notes

Firmware changes:

- `lr1_stack_mac_rx_frame_decode()` identifies suspected stale downlink-counter failures after `DevAddr` match and non-advancing `FCntDown` LSB.
- The LoRaWAN API exposes the last accepted downlink counter and sync-pending state.
- The tracker sends a 7-byte FPort 8 health/event family `6` uplink while sync is pending.
- The current implementation sends this as an extended uplink after successful non-SOS tracker uplinks, rate-limited to once per 60 seconds.
- Valid downlink acceptance clears the sync-pending latch.

### Device-side stale-counter detection

Detection happens inside the downlink receive/decode path, as soon as the radio receives a downlink that appears to be for this device but uses a stale downlink counter. It does not depend on confirmed uplinks, application acknowledgements, a later retry cycle, or any backend-side timeout.

The device first checks whether the downlink `DevAddr` matches its session. If the address does not match, the packet is ignored as unrelated and does not affect counter-sync state.

After `DevAddr` match, the stack compares the received lower 16 bits of `FCntDown` with the device's last accepted downlink counter:

- If the lower 16-bit value is non-advancing, meaning it is equal to or behind the device's current counter window, the stack treats the packet as suspected stale LNS state and latches `FCntDown` sync-pending.
- If the lower 16-bit value could be interpreted as rollover but MIC verification fails, this is also treated as suspected stale LNS state because the LNS likely signed the downlink with an older full `FCntDown` while the device reconstructed a newer full counter.

In both cases, the device does not accept the stale downlink and does not move its own counter backwards. The only local action is to latch sync-pending and report the last accepted device `FCntDown` on the preserved FPort 8 maintenance uplink. Any later valid downlink clears the latch.

Decoder changes:

- `RemEX_T1000E_Decoder.js` decodes FPort 8 family `6` as `FCntDown sync`.
- Decoded output includes `flags.syncPending`, `flags.staleDownlinkSeen`, and `fcntDown`.

LNS script requirements:

- Decode FPort 8 family `6` on both Master LNS and Relay LNS ingestion paths.
- Update the active LNS downlink counter only upward.
- Flush or regenerate queued downlinks after counter repair.
- Add expiry to pending downlink queue entries so old commands do not survive mode/authority transitions.

## Alternatives considered

- **Disable device-side replay protection.** This would let stale LNS downlinks through, but it would also allow old valid radio frames to be replayed.
- **Reset the device counter on authority change.** This is fragile because the device may not know which LNS is authoritative until after it receives a valid downlink.
- **Synchronize LNS state over backhaul only.** This is clean when available, but it fails when an LNS is first contacted with no current counter state.
- **Always include `FCntDown` in every routine uplink.** This is robust but wastes bytes on sampled FPort 5/7 traffic and does not guarantee preserved delivery.

## Reason for decision

- Preserves LoRaWAN replay protection on the device.
- Treats counter divergence as an LNS session-state synchronization problem.
- Uses FPort 8, which MDR-018 and MDR-022 reserve for low-rate preserved health/event and maintenance traffic.
- Works in both handover directions.
- Keeps routine FPort 5/7 payloads compact and sampled as designed.

## Known costs

- Both LNS paths must deploy decoder and counter-repair logic.
- FPort 8 must be allowed by the relay shim and backend.
- The first stale downlink after a handover can still fail; the repair happens on a following uplink.
- Extended uplink scheduling may interact with other extra queued uplinks, so SOS/MOB paths must keep priority.
- A false-positive stale-counter detection may cause a harmless repair uplink.

## Mitigations

- Only latch sync-pending after `DevAddr` matched and the received downlink counter did not advance.
- Do not accept stale downlinks on the device.
- Do not lower LNS counters from device reports.
- Rate-limit the maintenance uplink.
- Skip counter-sync extended uplinks during SOS paths.
- Flush or regenerate pending downlink queue entries after counter repair.

## Related notes

- [[MDR-018: Crew Tag FPort and Uplink Policy]]
- [[MDR-021 Dual Relay LNS Implementation on Relay Gateways]]
- [[MDR-022: Crew Tag Payload Optimization and Decoder Slimming]]
- `UPLINK_PAYLOADS.md`
- `RemEX_T1000E_Decoder.js`

#mob #mob/mdr #mob/lorawan #mob/crew-tags #mob/relay-lns
