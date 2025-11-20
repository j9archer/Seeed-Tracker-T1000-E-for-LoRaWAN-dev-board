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
#define FIRMWARE_VERSION_BUILD      1

// Version string (e.g., "1.0.0-b1")
#define FIRMWARE_VERSION_STRING     "1.0.0-b1"

// Version features (changelog for this version)
#define FIRMWARE_VERSION_FEATURES   "Vessel GNSS assistance + charging almanac"

/*
 * Version History:
 * 
 * v1.0.0-b1 (2025-11-20)
 *   - Added vessel position and time assistance for GNSS
 *   - Implemented 13-byte downlink payload on port 10
 *   - Added adaptive GNSS scan duration (10-60s based on assistance quality)
 *   - Implemented charging-based almanac maintenance (zero battery cost)
 *   - Position assistance via AG3335 PAIR062 commands
 *   - Own fix storage as fallback assistance
 *   - Expected TTFF: 10-25s with assistance vs 60s+ cold start
 */

#ifdef __cplusplus
}
#endif

#endif // FIRMWARE_VERSION_H

/* --- EOF ------------------------------------------------------------------ */
