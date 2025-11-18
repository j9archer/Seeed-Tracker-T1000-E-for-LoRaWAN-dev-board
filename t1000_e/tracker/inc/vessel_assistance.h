/*!
 * @file      vessel_assistance.h
 *
 * @brief     Vessel position and time assistance for GNSS
 *
 * Handles position and time updates from vessel relay gateway to improve
 * GNSS Time-To-First-Fix (TTFF) performance.
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
 * @brief Vessel position update message (13 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0x01 = Position Update
    int32_t vessel_lat;    // Latitude * 10^7
    int32_t vessel_lon;    // Longitude * 10^7
    uint32_t unix_time;    // Unix timestamp (seconds since epoch)
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
    uint32_t unix_time;          // Time from vessel (seconds since epoch)
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
 * @brief Apply position assistance to GNSS module
 *
 * Sends assistance position to AG3335 if available and useful
 *
 * @returns true if assistance was applied
 */
bool vessel_assistance_apply_to_gnss(void);

/*!
 * @brief Store own GNSS fix as fallback assistance data
 *
 * @param [in] lat Latitude from GNSS fix (scaled by 1e6)
 * @param [in] lon Longitude from GNSS fix (scaled by 1e6)
 */
void vessel_assistance_store_own_fix(int32_t lat, int32_t lon);

#ifdef __cplusplus
}
#endif

#endif // VESSEL_ASSISTANCE_H

/* --- EOF ------------------------------------------------------------------ */
