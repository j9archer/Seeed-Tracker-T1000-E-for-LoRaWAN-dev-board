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
#define FIRMWARE_VERSION_BUILD      7

// Version string (e.g., "1.0.0-b7")
#define FIRMWARE_VERSION_STRING     "1.0.0-b7"

// Version features (changelog for this version)
#define FIRMWARE_VERSION_FEATURES   "marine_gnss integrated into scan process"

/*
 * Version History:
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
