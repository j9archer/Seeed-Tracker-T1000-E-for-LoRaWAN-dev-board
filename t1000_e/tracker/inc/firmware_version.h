/*!
 * @file      firmware_version.h
 *
 * @brief     Firmware version information for T1000-E tracker
 */

#ifndef FIRMWARE_VERSION_H
#define FIRMWARE_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- FIRMWARE VERSION --------------------------------------------------------
 */

#define FIRMWARE_VERSION_MAJOR      1
#define FIRMWARE_VERSION_MINOR      0
#define FIRMWARE_VERSION_PATCH      0
#define FIRMWARE_VERSION_BUILD      49

// Version string (e.g., "1.0.0-b49")
#define FIRMWARE_VERSION_STRING     "1.0.0-b49"

// Version features (changelog for this version)
#define FIRMWARE_VERSION_FEATURES   "Restore production scan interval"

/*
 * Version History:
 * 
 * v1.0.0-b49 (2026-05-30)
 *   - CHANGED: Removed WiFi test override so configured default uplink interval returns to 1 minute
 *   - REMOVED: Hardcoded temporary GNSS PAIR600 test position injection
 *
 * v1.0.0-b48 (2026-05-30)
 *   - ADDED: WiFi logs include MAC/channel/type/RSSI details for rejected local-admin and mobile AP results
 *
 * v1.0.0-b47 (2026-05-30)
 *   - CHANGED: WiFi scans primary channels 1/6/11 plus last fixed AP channel before scanning other channels
 *   - CHANGED: If the primary WiFi channel set returns zero raw results, the same scan cycle checks channels 2/3/5/7/8/9/10/12/13
 *
 * v1.0.0-b46 (2026-05-30)
 *   - CHANGED: WiFi provisional mobile/random-only hits raise the next scan to 2 results instead of 3
 *   - CHANGED: Any accepted fixed/global WiFi result drops the next scan directly back to 1 result
 *
 * v1.0.0-b45 (2026-05-29)
 *   - CHANGED: Randomized/mobile-only WiFi hits are accepted provisionally and raise the next scan effort
 *   - CHANGED: WiFi no longer performs immediate same-cycle retry; capped failed scans fall through to fallback methods
 *
 * v1.0.0-b44 (2026-05-29)
 *   - CHANGED: WiFi scan filters locally administered/randomized MAC addresses before accepting a result
 *   - CHANGED: WiFi retry also triggers when the fast result is a filtered local-admin MAC
 *
 * v1.0.0-b43 (2026-05-29)
 *   - CHANGED: WiFi scan retries with three raw results when the fast scan only finds mobile APs
 *   - ADDED: WiFi logs include raw result count, mobile AP filtering, origin, and RSSI validity
 *
 * v1.0.0-b42 (2026-05-29)
 *   - CHANGED: Test firmware forces tracker periodic interval to 20 seconds after config load
 *
 * v1.0.0-b41 (2026-05-29)
 *   - CHANGED: LR11xx WiFi scan requests one result while preserving the current three-record uplink shape
 *
 * v1.0.0-b40 (2026-05-29)
 *   - CHANGED: 5-second startup serial delay now runs before version logging for all scan strategies
 *
 * v1.0.0-b39 (2026-05-29)
 *   - FIXED: WiFi scan nAh/uAh estimate uses 64-bit math before division
 *
 * v1.0.0-b38 (2026-05-29)
 *   - CHANGED: Trial WiFi scan duration reduced from 3 seconds to 1 second
 *
 * v1.0.0-b37 (2026-05-29)
 *   - ADDED: WiFi scan result logs include LR11xx-reported scan power consumption in uAh
 *
 * v1.0.0-b35 (2026-05-28)
 *   - FIXED: Firmware never writes, clears, or replaces app_param DevEUI
 *   - CHANGED: Config app / AT DevEUI setter is rejected; DevEUI is factory-provisioned read-only identity
 *
 * v1.0.0-b34 (2026-05-28)
 *   - FIXED: LR11xx hardware UID is not accepted as a LoRaWAN DevEUI for deterministic ABP
 *   - CHANGED: Invalid LR11xx UID-like DevEUI is cleared so the SenseCAP DevEUI must be entered/restored
 *
 * v1.0.0-b33 (2026-05-28)
 *   - FIXED: ABP derivation falls back to the LR11xx chip UID when the modem secure-element DevEUI is empty
 *   - ADDED: Visible modem-reset startup log for DevEUI, DevAddr, NwkSKey, and AppSKey
 *
 * v1.0.0-b32 (2026-05-28)
 *   - FIXED: Derived ABP DevAddr/NwkSKey/AppSKey are prepared before BLE config app reads app_param
 *   - CHANGED: LR11xx factory UID is mirrored into app_param DevEUI before deterministic ABP derivation
 *
 * v1.0.0-b31 (2026-05-27)
 *   - ADDED: Routine on-charge/spare uplinks route on FPort 7 with event-state bit 0x04 set
 *   - ADDED: MOB/SOS FPort 6 payloads preserve alert routing and carry on-charge metadata where available
 *
 * v1.0.0-b30 (2026-05-27)
 *   - CHANGED: LinkCheck strong-margin reserve reduced from 15 dB to 10 dB before spending 5 dB per DR step
 *
 * v1.0.0-b29 (2026-05-26)
 *   - ADDED: DevEUI-derived startup jitter before the first routine status uplink to spread charger-bank wakeups
 *   - ADDED: DevEUI-phased periodic LinkCheck scheduling to avoid fleet-wide control-frame bunching
 *   - ADDED: Missing LinkCheckAns retry ladder probes lower DRs without changing the normal FPort 5 uplink DR
 *
 * v1.0.0-b28 (2026-05-26)
 *   - FIXED: Power-on status payload is not misclassified as SOS when battery byte contains bit 0x40
 *   - CHANGED: Power-on status uplink uses normal routine scheduling on FPort 5
 *
 * v1.0.0-b27 (2026-05-26)
 *   - FIXED: Missing LinkCheckAns is treated as inconclusive instead of weak RF, preventing false DR step-downs
 *   - CHANGED: A stable BLE Minor hint can restore its configured DR when LinkCheck has not actually refined it
 *
 * v1.0.0-b26 (2026-05-26)
 *   - CHANGED: Custom BLE routine uplinks use new Data IDs 0x27/0x28 with Major/Minor/RSSI records
 *   - CHANGED: BLE scanning returned to passive mode for production beacon detection
 *
 * v1.0.0-b25 (2026-05-19)
 *   - CHANGED: Derived ABP DevAddr allocation treats each NetID as one 128-address client/project block
 *   - CHANGED: Crew Tags use Group 2 addresses after the reserved Group 1 slice inside each project block
 *   - ADDED: Config macros for NetID start/end and Group 1 block size so gateway filtering can use /25 project ranges
 *
 * v1.0.0-b24 (2026-05-19)
 *   - CHANGED: ABP mode always replaces stored DevAddr/NwkSKey/AppSKey with deterministic derived values
 *   - CHANGED: Derived ABP values are persisted only when they differ from stored config to avoid repeated flash writes
 *
 * v1.0.0-b23 (2026-05-19)
 *   - ADDED: Deterministic ABP defaults for Crew Tags using factory DevEUI plus RemEX master key/NetID
 *   - CHANGED: ABP is the default activation mode while OTAA remains selectable through the config app
 *   - CHANGED: Missing DevEUI/AppKey values are left empty instead of falling back to build-time identity constants
 *
 * v1.0.0-b22 (2026-05-15)
 *   - CHANGED: BLE movement detection uses strongest approved UUID beacon even when Minor is not 1..5
 *   - CHANGED: Minor 1..5 remains required for direct BLE DR changes; unconfigured Minors only reset stability/LinkCheck timing
 *   - CHANGED: Minor 1..5 is interpreted as the proposed LoRaWAN DR, not an inverse RF-grade profile
 *
 * v1.0.0-b21 (2026-05-15)
 *   - FIXED: Strongest-beacon changes must repeat across scan sessions before resetting BLE DR baseline
 *   - CHANGED: BLE movement hysteresis now guards both conservative and less-conservative location changes
 *
 * v1.0.0-b20 (2026-05-15)
 *   - FIXED: Same BLE beacon/profile no longer pulls DR back down after LinkCheck has refined it upward
 *   - ADDED: BLE hint state tracks UUID/MAC identity so a new strongest beacon still resets the location hint
 *
 * v1.0.0-b19 (2026-05-15)
 *   - CHANGED: Strong LinkCheckAns now corrects the BLE hint DR immediately using measured margin
 *   - CHANGED: Uses a conservative 5 dB per DR step budget while preserving 15 dB reserve margin
 *
 * v1.0.0-b18 (2026-05-15)
 *   - CHANGED: PAIR080,7 Swimming navigation mode hook is compiled out after AG3335M_V2.6.0 rejected it with status=4
 *   - NOTE: Code remains behind AG3335_ENABLE_SWIMMING_NAV_MODE for future GNSS chip/firmware revisions
 *
 * v1.0.0-b17 (2026-05-15)
 *   - FIXED: Standard vessel FPort 5 uplinks now re-tag each queued send with the current crew DR
 *   - FIXED: Prevents background MAC tasks from leaving later vessel uplinks at a stale/lower DR after a BLE hint
 *
 * v1.0.0-b16 (2026-05-15)
 *   - CHANGED: LinkCheck logs now state that Basics Modem schedules a dedicated MAC-only FPort 0 task
 *   - ADDED: TODO to revisit LinkCheck scheduling after the gateway shim forwards FOpts MAC commands by default
 *
 * v1.0.0-b15 (2026-05-15)
 *   - FIXED: BLE DR hints now work with build-time whitelisted UUIDs even when the config app extra UUID is empty
 *   - ADDED: BLE log explains when DR hints are disabled because no approved UUID source is configured
 *
 * v1.0.0-b14 (2026-05-15)
 *   - CHANGED: Restored MDR-012 behavior: all selected DRs clamp to lr_DR_min/lr_DR_max
 *   - CHANGED: BLE Minor profile 5 and MOB/SOS minimum paths use the same configured SOS-low/minimum-safe DR
 *   - NOTE: This reverses the b13 experiment that allowed MOB/SOS below lr_DR_min
 *
 * v1.0.0-b13 (2026-05-15)
 *   - CHANGED: Vessel DR floor now respects lr_DR_min while MOB/SOS can use region_min + 1 for emergency penetration
 *   - CHANGED: BLE Minor profile 5 now selects the vessel floor; the lower emergency floor is reserved for MOB/SOS
 *   - FIXED: Forced next-uplink DR tagging now uses the same clamped DR that was applied to the modem profile
 *
 * v1.0.0-b12 (2026-05-15)
 *   - CHANGED: GNSS scan startup now tries PAIR080,7 Swimming navigation mode for MOB/PIW validation
 *   - NOTE: Some AG3335 firmware may reject mode 7 with PAIR001 status=4; PAIR081 query logs the result
 *
 * v1.0.0-b11 (2026-05-15)
 *   - ADDED: Build-time approved iBeacon UUID whitelist for production beacon families
 *   - CHANGED: Config app Group UUID is treated as one extra approved full UUID, not the only scanner filter
 *   - CHANGED: BLE UUID approval now requires exact 16-byte matches instead of prefix/substring matching
 *
 * v1.0.0-b10 (2026-05-15)
 *   - ADDED: BLE scan diagnostics under the B log filter: scan start/stop summaries, approved iBeacon details, rejected UUID samples
 *   - ADDED: DR hint logging showing the strongest approved beacon, Major, Minor, RSSI, UUID, and MAC
 *
 * v1.0.0-b9 (2026-05-15)
 *   - ADDED: Real-time serial log filters using single-key toggles: L=LORA, N=NMEA, G=GNSS, B=BLE, W=WIFI, ?=status
 *   - CHANGED: Semtech modem/RP traces and LoRaWAN TX/RX/LinkCheck events are tagged as LORA for RF-only debugging
 *   - CHANGED: GNSS lifecycle logs and MOB/PIW logs are tagged as GNSS; NMEA quality lines are tagged as NMEA
 *   - FIXED: Literal escaped line breaks in scheduler logs now print as real CR/LF line breaks
 *
 * v1.0.0-b8 (2026-05-15)
 *   - ADDED: Mission-oriented crew tag DR strategy with ADR disabled and NbTrans=1
 *   - ADDED: MOB/SOS DR policies: initial multi-DR bursts, max-DR early tracking, persistence/minimum fallback
 *   - ADDED: Per-task forced DR support so queued burst uplinks retain distinct datarates
 *   - ADDED: BLE beacon RF hints using approved iBeacon UUIDs and Minor values 1..5
 *   - ADDED: BLE hint hysteresis: conservative moves apply immediately, less-conservative moves require scan-session confirmation
 *   - CHANGED: Vessel RF validation uses LinkCheckReq margin/gateway count instead of confirmed health uplinks
 *   - LinkCheckReq runs after stable BLE profiles and periodically, with a longer interval when valid BLE hints are active
 *   - See MDR-012 for the BLE hint + LinkCheck DR decision record
 *
 * v1.0.0-b7 (2025-11-27)
 *   - INTEGRATED: marine_gnss replaces default GNSS in app_tracker_scan_process
 *   - REMOVED: app_get_adaptive_gnss_scan_duration() - superseded by marine_gnss
 *   - CHANGED: app_tracker_gnss_scan_begin() now calls mob_tracker_activate()
 *   - CHANGED: Scan process uses mob_tracker_process() for timing
 *   - Marine GNSS provides: MOB burst -> PIW phases with quality-driven exit
 *   - BLE beacon detection cancels marine_gnss tracking
 *
 * v1.0.0-b6 (2025-11-26)
 *   - RENAMED: mob_piw_tracker -> marine_gnss (clearer module purpose)
 *   - RENAMED: vessel_assistance -> gateway_assistance (supports any gateway)
 *   - ADDED: PAIR004 hot start for PIW phases (uses ephemeris if available)
 *   - ADDED: PAIR005 warm start for MOB burst (uses almanac, no ephemeris)
 *   - ADDED: GNSS power cycling for PAIR590/600 when not in GNSS mode
 *   - ADDED: Dynamic NMEA debug output for MOB burst and PIW Phase 1
 *   - ADDED: gnss_enable_nmea_debug() for quality verification
 *   - FIXED: gateway_assistance_send_time_to_gnss() now takes gnss_is_active param
 *   - Gateway assistance only powers GNSS when needed (BLE/WiFi mode)
 *
 * v1.0.0-b5 (2025-11-26)
 *   - ADDED: MOB (Man Overboard) burst mode - continuous GNSS for 5 min
 *   - ADDED: PIW (Person In Water) tracking mode - quality-driven GNSS scans
 *   - ADDED: gnss_fix_t struct with HDOP/HACC quality metrics
 *   - ADDED: gnss_scan_until_good() - early exit when fix quality acceptable
 *   - ADDED: Double uplinks every 30s with position + HDOP
 *   - ADDED: BLE scan interrupt - GNSS stops if beacon found
 *   - ADDED: Phase transitions: 30s->1min->2min scan intervals
 *   - MOB burst: 0-5 min, 30s double uplinks, loose accuracy
 *   - PIW phase 1: 5-30 min, 30s intervals, HDOP<3, HACC<15m
 *   - PIW phase 2: 30min-2hr, 60s intervals
 *   - PIW phase 3: after 2hr, 120s intervals
 *
 * v1.0.0-b4 (2025-11-26)
 *   - FIXED: Adaptive GNSS scan duration now uses MIN (user config as upper bound)
 *   - FIXED: PAIR590 UTC time injection moved to GNSS scan start (was sent when asleep)
 *   - ADDED: PAIR080 Swimming navigation mode (mode=7) for MOB/PIW use-case
 *   - ADDED: PAIR001 ACK response parsing with status logging
 *
 * v1.0.0-b3 (2025-11-21)
 *   - ADDED: PAIR590 - UTC time injection to AG3335 (SIM68D NMEA spec)
 *   - ADDED: PAIR600 - Reference position injection to AG3335 (SIM68D NMEA spec)
 *   - Vessel assistance now uses actual AG3335 position/time commands
 *   - Time accuracy <3 seconds recommended for optimal TTFF
 *   - Position accuracy: 50m horizontal, 100m vertical (conservative)
 * 
 * v1.0.0-b2 (2025-11-21)
 *   - FIXED: Removed non-existent PAIR062 command
 *   - FIXED: Implemented proper time synchronization via smtc_modem_set_time()
 *   - Vessel assistance provides time sync for improved TTFF
 *   - Adaptive GNSS scan duration based on assistance quality
 *   - Charging-based almanac maintenance (zero battery cost)
 *   - Verified against AG3335 command specification (LOCOSYS v1.1)
 * 
 * v1.0.0-b1 (2025-11-20)
 *   - Initial vessel position and time assistance implementation
 *   - 13-byte downlink payload on port 10
 *   - Adaptive GNSS scan duration (10-60s based on assistance quality)
 */

#ifdef __cplusplus
}
#endif

#endif // FIRMWARE_VERSION_H

/* --- EOF ------------------------------------------------------------------ */
