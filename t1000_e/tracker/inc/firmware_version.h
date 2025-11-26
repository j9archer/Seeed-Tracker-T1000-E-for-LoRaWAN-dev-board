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
#define FIRMWARE_VERSION_BUILD      3

// Version string (e.g., "1.0.0-b3")
#define FIRMWARE_VERSION_STRING     "1.0.0-b3"

// Version features (changelog for this version)
#define FIRMWARE_VERSION_FEATURES   "AG3335 PAIR590/PAIR600 assistance"

/*
 * Version History:
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
