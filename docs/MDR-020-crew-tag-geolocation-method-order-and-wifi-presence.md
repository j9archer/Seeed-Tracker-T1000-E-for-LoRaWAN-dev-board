---
title: "MDR-020: Crew Tag Geolocation Method Order and WiFi Presence Detection"
type: "mdr"
status: "accepted"
tags:
  - mob
  - mob/mdr
  - mob/rf
  - mob/crew-tags
  - wifi
  - ble
aliases:
  - "MDR-020"
  - "Crew Tag Geolocation Method Order and WiFi Presence Detection"
---

# MDR-020: Crew Tag Geolocation Method Order and WiFi Presence Detection

Parent: [[Major Decision Records Index]]

## Status

Accepted

## Context

Crew tags need a low-power way to decide whether the wearer is likely still on board before escalating to more expensive or slower geolocation methods. The vessel may already have WiFi coverage, especially on larger or temporary installations, while BLE beacons can fill coverage gaps and provide more deterministic vessel-local hints.

For this use case, WiFi is not being used for positioning. It is used as a boolean presence hint: if any acceptable vessel-area WiFi signal is visible, the tag can treat the wearer as likely on board. If WiFi is absent or ambiguous after bounded effort, the tag should continue to fallback methods such as BLE and GNSS according to the configured scan mode.

The T1000-E hardware can sniff WiFi through the LR11xx radio but does not provide a general WiFi station interface for joining an access point and downloading application data. The WiFi decision here is therefore limited to passive scan/sniff behavior.

Firmware tests showed that reducing LR11xx WiFi scan effort materially reduced scan power:

- Earlier multi-result scans were typically around 7-14 uAh per scan in the tested environment.
- Requesting one result typically reduced successful scan cost to roughly 1.5-4 uAh in WiFi-only testing.
- Immediate same-cycle retries after randomized/mobile-only results raised the average again, around 5 uAh in the observed test run.
- Adaptive next-cycle retry capped at two results reduced the b46 WiFi-only average to about 3.31 uAh per scan over 182 scans.
- Staged primary-channel scanning and BLE-first combined mode reduced later observed WiFi scan costs further, down to about 0.99 uAh average in a b48 WiFi-only primary-channel run and about 0.16 uAh for WiFi scans reached after BLE failed in a combined BLE/WiFi/GNSS test.

iPhone hotspot testing showed that the LR11xx mobile AP origin flag did not reliably identify the nearby phone hotspot. Locally administered/randomized MAC detection was a more useful filter for mobile/private hotspot style results, but a randomized MAC still strongly suggests a nearby WiFi transmitter and therefore may still indicate that the wearer is on or near the vessel.

## Decision

Use BLE first, WiFi second, and GNSS last for the primary combined geolocation strategy.

Combined scan order:

- `BLE_WIFI_GNSS`: scan BLE first; if no accepted BLE result, scan WiFi; if WiFi does not produce an accepted or provisional on-board hint, continue to GNSS.
- `BLE_WIFI`: scan BLE first; if no accepted BLE result, scan WiFi.
- `WIFI_GNSS`: scan WiFi first; if no accepted or provisional WiFi result, continue to GNSS.
- `GNSS_WIFI`: run GNSS first, then WiFi only if GNSS does not produce a usable result.
- `WIFI_ONLY`: run only WiFi; no fallback is available if the capped WiFi scan fails.
- `BLE_ONLY` and `BLE_GNSS`: unaffected by WiFi policy.

This preserves existing explicitly configured scan modes, while making `BLE_WIFI_GNSS` the preferred low-power all-method order for vessel presence detection.

## BLE

BLE remains the first preferred on-vessel hint when it is enabled.

BLE behavior:

- Use approved BLE beacons as the most deterministic on-vessel signal.
- Treat BLE scans as low-power local snapshots before uplink, consistent with MDR-012.
- Use BLE first in combined modes because BLE beacons are expected to consume less scan power than WiFi and are less likely to be confused with a crew member's phone hotspot.
- Use BLE to cover vessel gaps where WiFi is weak or absent.
- Keep BLE beacon identity and DR hint behavior governed by MDR-012.

BLE deployment policy:

- Do not require complete BLE coverage for temporary or larger vessel installations when existing WiFi coverage can provide a useful presence hint.
- Prefer BLE for known critical dead zones, muster areas, deck transitions, and areas where WiFi coverage is unreliable.
- Treat missing BLE as inconclusive, not proof that the wearer is off vessel.

## WiFi

WiFi is used as a passive presence detector, not as a positioning source.

WiFi scan behavior:

- Request one LR11xx WiFi result by default.
- Support B/G/N signal types.
- Scan primary channels `1,6,11` plus the last accepted fixed/global AP channel first.
- If the primary channel set returns no raw result at all, immediately scan the remaining 2.4 GHz channels `2,3,4,5,7,8,9,10,12,13`, excluding any channel already covered by the primary set.
- Ignore channel 14.
- Filter known mobile AP origin results when the LR11xx marks them.
- Filter locally administered/randomized MAC addresses as likely phone/private hotspot results.
- Filter duplicate MAC addresses within a scan result set.
- Preserve detailed logs for scan stage, channel mask, raw count, accepted count, mobile/local-admin/duplicate/unknown-origin filtering, rejected MAC details, origin, RSSI validity, power, and timing.

Adaptive WiFi effort:

- If a fixed/global AP is accepted, treat WiFi as a successful on-board hint and step the next scan effort back down toward one result.
- If a one-result scan sees only a filtered mobile/random/local-admin AP, accept WiFi provisionally for that cycle because the wearer is still likely near a WiFi transmitter.
- Do not immediately retry in the same cycle after a provisional mobile/random-only hit.
- Program the next WiFi scan to request up to two results after a provisional mobile/random-only hit.
- If a capped scan still finds no acceptable fixed/global AP, do not accept WiFi for that cycle. Let the configured fallback continue, such as GNSS in `BLE_WIFI_GNSS` or `WIFI_GNSS`.
- If a capped scan finds an accepted fixed/global AP, step the next scan directly back to one result.
- Keep the adaptive cap at two results unless field data justifies a different limit.

WiFi payload policy:

- Do not change the payload format as part of this decision.
- Continue padding shorter WiFi results to the existing configured WiFi payload shape during testing.
- Defer the payload optimization that sends only the strongest/accepted WiFi reading until backend decoder and product behavior are agreed.

Phone hotspot handling:

- Do not rely solely on the LR11xx mobile AP origin flag.
- Treat locally administered MAC addresses as randomized/private candidates.
- A randomized/private candidate is not a strong fixed-vessel AP signal, but it is still a useful provisional on-board hint on the first low-effort scan.
- If the adaptive higher-effort scan cannot find any fixed/global AP, WiFi should fail for that cycle so the system can escalate.

## GNSS

GNSS remains the fallback for uncertain/off-vessel or alert-oriented operation.

GNSS behavior:

- Do not run routine GNSS before BLE/WiFi in the preferred all-method vessel-presence mode.
- Use GNSS after BLE and WiFi have failed to provide an accepted/provisional on-board hint.
- Continue using DeviceTimeReq, gateway/vessel position assistance, and charging/background GNSS maintenance to reduce TTFF, as described in MDR-017 and `docs/background-gnss-charging.md`.
- Do not weaken MOB/SOS behavior. Emergency GNSS rules remain separate from routine low-power vessel-presence scans.

## Test Evidence

Tests were run on temporary firmware builds b39 through b48 while exercising WiFi-only and combined BLE/WiFi/GNSS scan modes. These figures are field-log observations from the tested RF environment, not guaranteed hardware limits.

WiFi-only results:

- b39/b40-style multi-result WiFi scans commonly reported about 6-14 uAh per scan when collecting several APs.
- b42 used `max_results=1` and a 20 second test interval. Successful scans commonly fell to about 3-4 uAh, a large reduction from the earlier multi-result scans.
- b43 added immediate same-cycle retry when the first result looked mobile/randomized. It recovered fixed APs, but the retry path pushed average scan energy back up to roughly 5 uAh in the tested run.
- b45 moved the retry decision to the next cycle but initially stepped to three results. A longer WiFi-only log showed 322 scans averaging about 3.69 uAh; one-result scans averaged about 2.72 uAh, two-result scans about 4.68 uAh, and three-result scans about 6.07 uAh.
- b46 capped adaptive follow-up at two results and dropped directly back to one result after any fixed/global AP. A longer WiFi-only log showed 182 scans averaging about 3.31 uAh; one-result scans averaged about 2.93 uAh and two-result scans about 4.70 uAh.
- b47/b48 added staged primary-channel scanning. In a WiFi-only b48 primary-channel log, 138 scans averaged about 0.99 uAh, with one-result scans averaging about 0.77 uAh and two-result follow-up scans averaging about 1.61 uAh.

Combined BLE/WiFi/GNSS result:

- In b48 `BLE_WIFI_GNSS` testing, BLE ran first and suppressed WiFi whenever approved iBeacons were found.
- When BLE reported zero approved iBeacons, WiFi ran next and accepted a fixed/global AP.
- GNSS did not run as a fallback in those cycles because BLE or WiFi succeeded first.
- The observed WiFi scans reached after BLE failure averaged about 0.16 uAh in the provided snippet, because the fixed AP was found quickly on channel 1.

Phone/randomized AP handling:

- A nearby iPhone hotspot did not reliably appear as `LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP`.
- Locally administered MAC filtering did catch randomized/private-style addresses.
- b48 logs confirmed rejected local-admin MAC details are visible, including MAC, channel, type, RSSI, origin, and RSSI validity.
- In one b48 WiFi-only log, the rejected local-admin MAC `32 24 A9 B6 61 6B` appeared 49 times, always on channel 1.

Staged channel behavior:

- The LR11xx channel API accepts a mask, so it does not expose a direct order such as `11,6,1` versus `1,6,11` within one scan.
- Ordered behavior is therefore implemented by staged masks: primary set first, then remainder only when the primary set returns no raw result.
- In the observed b48 logs the primary mask `0x0421` was sufficient; no remainder scan was triggered because every primary scan returned at least one raw result.

## Reason for Decision

- BLE first gives the lowest-power deterministic vessel-local signal when beacons are present.
- WiFi second lets short-term or large-vessel installations reuse existing vessel WiFi coverage, reducing the number of BLE beacons required.
- GNSS last avoids spending high power and time when local RF can already indicate likely on-board presence.
- WiFi `max_results=1` produced a large observed power reduction compared with multi-result scans.
- Immediate same-cycle WiFi retries recovered fixed APs but increased average scan energy; adaptive next-cycle effort preserves most of the power saving while still learning from previous scans.
- Primary-channel staged scanning avoids spending time across all 2.4 GHz channels when common channels or the last good channel are sufficient.
- Treating randomized/mobile-only WiFi as provisional avoids false off-vessel escalation when a nearby phone hotspot is visible on the vessel.
- Capping adaptive WiFi effort prevents WiFi from consuming repeated power when the RF environment is dominated by mobile/randomized APs.
- Keeping payload shape unchanged reduces variables during power and behavior testing.

## Alternatives Considered

- WiFi first, then BLE, then GNSS.
- WiFi only as the main on-vessel detector.
- Immediate same-cycle WiFi retry whenever the first result is mobile/random/local-admin only.
- Always scan all WiFi channels in one pass.
- Try to control channel order inside a single LR11xx channel mask.
- Reject all randomized/local-admin WiFi results immediately.
- Use WiFi as a positioning input by sending multiple APs.
- Change the uplink payload immediately to send only the strongest WiFi reading.
- Use GNSS before local RF methods in combined modes.

## Known Costs

- BLE requires beacon installation, commissioning, and UUID management.
- WiFi coverage may not be uniform across decks, machinery spaces, or metal compartments.
- A phone hotspot can create a provisional WiFi success even if the wearer is near or overboard with the phone.
- Locally administered MAC filtering is heuristic; some legitimate infrastructure can use randomized or locally administered addresses.
- A one-result WiFi scan may miss a fixed AP when a stronger randomized/mobile AP is present.
- Adaptive WiFi state can make consecutive scans behave differently, which must be understood when comparing field logs.
- Keeping the old WiFi payload shape continues to spend uplink airtime on padding until the payload decision is made.

## Mitigations

- Put BLE first in the preferred combined mode so approved beacons win before WiFi is attempted.
- Limit WiFi adaptive scan effort to two results.
- Scan `1,6,11` plus last-good channel before scanning the remaining channels.
- Log adaptive WiFi state, including current and next result limits, so field traces show why scan effort changed.
- Log rejected local-admin and mobile AP MAC details so phone/randomized behavior can be audited.
- Only accept randomized/mobile-only WiFi provisionally when the current scan effort is below the cap.
- At the adaptive cap, require an accepted fixed/global AP for WiFi success; otherwise proceed to fallback.
- Fill WiFi coverage gaps with BLE beacons rather than trying to solve all vessel areas with WiFi.
- Keep GNSS fallback available for uncertainty and off-vessel confirmation.
- Revisit WiFi payload compaction after scan behavior is validated in field logs.

## Open Follow-Ups

- Decide whether the backend should receive a distinct provisional WiFi marker, instead of the current dummy/padded WiFi record.
- Decide whether WiFi payloads should be reduced to one strongest accepted result for routine traffic.
- Measure BLE scan power with the same logging precision used for WiFi.
- Field-test locked b48 behavior on vessels with real onboard WiFi, crew phones, and sparse/dead-zone areas.
- Test a case where primary WiFi channels return zero raw results, to verify immediate remainder-channel scanning in field logs.
- Test last-good channels outside `1,6,11`, such as channel 4, to verify the primary mask includes the last-good channel and excludes it from the remainder scan.
- Test fallback to GNSS by disabling approved BLE beacons and moving outside usable WiFi range.
- Decide whether fixed AP allowlists or vessel-specific SSID/BSSID commissioning are useful for higher assurance deployments.

## Related Notes

- [[MDR-012: BLE Hint and LinkCheck Based Crew Tag DR Strategy]]
- [[MDR-017 Vessel Assistance Position and Time Seeding for Crew Tag GNSS]]
- [[MDR-018: Crew Tag FPort and Uplink Policy]]
- [[MDR-019: Crew Tag Charger State Routing and POB Counting]]
- `docs/background-gnss-charging.md`
- `UPLINK_PAYLOADS.md`
- `apps/examples/11_lorawan_tracker/main_lorawan_tracker.c`
- `t1000_e/peripherals/src/wifi_scan.c`
- `t1000_e/peripherals/src/wifi_helpers.c`

#mob #mob/mdr #mob/rf #mob/crew-tags #wifi #ble
