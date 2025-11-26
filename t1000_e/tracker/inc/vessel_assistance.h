/*!
 * @file      vessel_assistance.h
 *
 * @brief     Vessel position and time assistance for GNSS
 *
 * Handles position and time updates from vessel relay gateway to improve
 * GNSS Time-To-First-Fix (TTFF) performance using AG3335 PAIR commands:
 * - PAIR590: UTC time reference (accuracy <3s recommended)
 * - PAIR600: Position reference (lat/lon/altitude with accuracy estimates)
 */

#ifndef VESSEL_ASSISTANCE_H
#define VESSEL_ASSISTANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define VESSEL_ASSISTANCE_PORT 10  // LoRaWAN port for vessel assistance messages

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Vessel position update message (9 bytes)
 * Note: Time sync handled separately via DeviceTimeReq MAC command
 */
typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0x01 = Position Update
    int32_t vessel_lat;    // Latitude * 10^7
    int32_t vessel_lon;    // Longitude * 10^7
} vessel_position_msg_t;

/*!
 * @brief Assistance quality levels
 */
typedef enum {
    ASSISTANCE_EXCELLENT,  // < 30 min old, high confidence
    ASSISTANCE_GOOD,       // 30-90 min old
    ASSISTANCE_FAIR,       // 90-180 min old
    ASSISTANCE_POOR        // > 180 min old or no data
} assistance_quality_t;

/*!
 * @brief Position and time cache structure
 */
typedef struct {
    float latitude;              // Vessel latitude in degrees
    float longitude;             // Vessel longitude in degrees
    uint32_t unix_time;          // Time when position received (from modem GPS time)
    uint32_t rtc_at_receipt;     // Local RTC when message received
    uint32_t time_uncertainty;   // Estimated time uncertainty in seconds
    bool valid;                  // Cache contains valid data
} position_time_cache_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize vessel assistance system
 */
void vessel_assistance_init(void);

/*!
 * @brief Handle vessel position downlink message
 *
 * @param [in] payload Downlink payload
 * @param [in] size Payload size in bytes
 *
 * @returns true if message was valid and processed
 */
bool vessel_assistance_handle_downlink(const uint8_t* payload, uint8_t size);

/*!
 * @brief Get current position assistance cache
 *
 * @returns Pointer to position/time cache (read-only)
 */
const position_time_cache_t* vessel_assistance_get_cache(void);

/*!
 * @brief Check if assistance data is available and useful
 *
 * @returns true if assistance should be used
 */
bool vessel_assistance_is_available(void);

/*!
 * @brief Get assistance quality level
 *
 * @returns Assistance quality enumeration
 */
assistance_quality_t vessel_assistance_get_quality(void);

/*!
 * @brief Get estimated current time based on cache and RTC
 *
 * @returns Estimated current Unix time in seconds
 */
uint32_t vessel_assistance_get_estimated_time(void);

/*!
 * @brief Get time uncertainty in seconds
 *
 * @returns Estimated time uncertainty in seconds
 */
uint32_t vessel_assistance_get_time_uncertainty(void);

/*!
 * @brief Get recommended GNSS scan duration based on assistance quality
 *
 * @returns Recommended scan duration in seconds
 */
uint32_t vessel_assistance_get_recommended_scan_duration(void);

/*!
 * @brief Check if device is currently charging
 *
 * @returns true if USB/charger connected, false otherwise
 */
bool vessel_assistance_is_charging(void);

/*!
 * @brief Check if almanac maintenance is needed
 *
 * @param [in] days_threshold Number of days without GNSS fix before maintenance needed
 * @returns true if almanac maintenance should be performed
 */
bool vessel_assistance_needs_almanac_maintenance(uint32_t days_threshold);

/*!
 * @brief Get extended scan duration for almanac download
 *
 * Provides longer scan duration (12.5 minutes) to allow almanac download
 * when device hasn't had a GNSS fix for an extended period
 *
 * @returns Extended scan duration in seconds (750s = 12.5 minutes)
 */
uint32_t vessel_assistance_get_almanac_scan_duration(void);

/*!
 * @brief Send time sync to AG3335 GNSS module NVRAM
 *
 * Called immediately when DeviceTimeAns received via modem time sync callback.
 * Writes UTC time to AG3335 NVRAM via PAIR590 command.
 *
 * @returns true if time was sent successfully
 */
bool vessel_assistance_send_time_to_gnss(void);

/*!
 * @brief Store own GNSS fix as fallback assistance data
 *
 * @param [in] lat Latitude from GNSS fix (scaled by 1e6)
 * @param [in] lon Longitude from GNSS fix (scaled by 1e6)
 */
void vessel_assistance_store_own_fix(int32_t lat, int32_t lon);

/*!
 * @brief Check if GNSS is ready for warm start
 *
 * Evaluates all GNSS Ready criteria:
 * - Time sync: GPS time error < 3 seconds (DeviceTimeReq valid)
 * - Fresh almanac: GNSS fix within 14 days
 * - Recent position: Valid position within 4 hours
 *
 * @returns true if all criteria met for warm start
 */
bool vessel_assistance_is_gnss_ready(void);

/*!
 * @brief Send warm start command to AG3335
 *
 * Issues PAIR005 command for warm start when GNSS ready conditions are met.
 * Warm start uses almanac data without requiring ephemeris download.
 *
 * @returns true if command was sent successfully
 */
bool vessel_assistance_send_warm_start(void);

/*!
 * @brief Send hardcoded test position to AG3335 (TEMPORARY for testing)
 *
 * Sends PAIR600 with hardcoded coordinates for initial testing:
 * - Latitude: 37.099775°N
 * - Longitude: 8.460805°W
 * - Altitude: 63m
 *
 * @note This is a temporary function for testing. Remove when server downlink implemented.
 *
 * @returns true if command was sent successfully
 */
bool vessel_assistance_send_test_position(void);

#ifdef __cplusplus
}
#endif

#endif // VESSEL_ASSISTANCE_H

/* --- EOF ------------------------------------------------------------------ */
