/*!
 * @file      gateway_assistance.h
 *
 * @brief     Gateway position and time assistance for GNSS
 *
 * Handles position and time updates from relay gateway to improve
 * GNSS Time-To-First-Fix (TTFF) performance using AG3335 PAIR commands:
 * - PAIR004: Hot start (fast restart with valid ephemeris)
 * - PAIR005: Warm start (uses almanac, no ephemeris)
 * - PAIR590: UTC time reference (accuracy <3s recommended)
 * - PAIR600: Position reference (lat/lon/altitude with accuracy estimates)
 *
 * When device is NOT in GNSS mode (BLE or WiFi mode), PAIR590/600 commands
 * will power on the GNSS module, send commands, wait for ACK, and power off.
 */

#ifndef GATEWAY_ASSISTANCE_H
#define GATEWAY_ASSISTANCE_H

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

#define GATEWAY_ASSISTANCE_PORT 10  // LoRaWAN port for gateway assistance messages

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Gateway position update message (9 bytes)
 * Note: Time sync handled separately via DeviceTimeReq MAC command
 */
typedef struct __attribute__((packed)) {
    uint8_t msg_type;        // 0x01 = Position Update
    int32_t gateway_lat;     // Latitude * 10^7
    int32_t gateway_lon;     // Longitude * 10^7
} gateway_position_msg_t;

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
    float latitude;              // Gateway latitude in degrees
    float longitude;             // Gateway longitude in degrees
    uint32_t unix_time;          // Time when position received (from modem GPS time)
    uint32_t rtc_at_receipt;     // Local RTC when message received
    uint32_t time_uncertainty;   // Estimated time uncertainty in seconds
    uint32_t time_rtc_at_sync;   // Local RTC when time was synced
    bool valid;                  // Position cache contains valid data
    bool time_synced;            // Time has been synced (DeviceTimeReq success)
} position_time_cache_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize gateway assistance system
 */
void gateway_assistance_init(void);

/*!
 * @brief Handle gateway position downlink message
 *
 * @param [in] payload Downlink payload
 * @param [in] size Payload size in bytes
 *
 * @returns true if message was valid and processed
 */
bool gateway_assistance_handle_downlink(const uint8_t* payload, uint8_t size);

/*!
 * @brief Get current position assistance cache
 *
 * @returns Pointer to position/time cache (read-only)
 */
const position_time_cache_t* gateway_assistance_get_cache(void);

/*!
 * @brief Check if assistance data is available and useful
 *
 * @returns true if assistance should be used
 */
bool gateway_assistance_is_available(void);

/*!
 * @brief Get assistance quality level
 *
 * @returns Assistance quality enumeration
 */
assistance_quality_t gateway_assistance_get_quality(void);

/*!
 * @brief Get estimated current time based on cache and RTC
 *
 * @returns Estimated current Unix time in seconds
 */
uint32_t gateway_assistance_get_estimated_time(void);

/*!
 * @brief Get time uncertainty in seconds
 *
 * @returns Estimated time uncertainty in seconds
 */
uint32_t gateway_assistance_get_time_uncertainty(void);

/*!
 * @brief Get recommended GNSS scan duration based on assistance quality
 *
 * @returns Recommended scan duration in seconds
 */
uint32_t gateway_assistance_get_recommended_scan_duration(void);

/*!
 * @brief Check if device is currently charging
 *
 * @returns true if USB/charger connected, false otherwise
 */
bool gateway_assistance_is_charging(void);

/*!
 * @brief Check if almanac maintenance is needed
 *
 * @param [in] days_threshold Number of days without GNSS fix before maintenance needed
 * @returns true if almanac maintenance should be performed
 */
bool gateway_assistance_needs_almanac_maintenance(uint32_t days_threshold);

/*!
 * @brief Get extended scan duration for almanac download
 *
 * Provides longer scan duration (12.5 minutes) to allow almanac download
 * when device hasn't had a GNSS fix for an extended period
 *
 * @returns Extended scan duration in seconds (750s = 12.5 minutes)
 */
uint32_t gateway_assistance_get_almanac_scan_duration(void);

/*!
 * @brief Send time sync to AG3335 GNSS module NVRAM
 *
 * When called while NOT in GNSS mode, this will:
 * 1. Power on the AG3335 module
 * 2. Wait for module ready
 * 3. Send PAIR590 UTC time command
 * 4. Wait for ACK
 * 5. Power off the module
 *
 * When called during GNSS scan (module already powered), just sends command.
 *
 * @param [in] gnss_is_active true if GNSS scan is in progress (module powered)
 * @returns true if time was sent successfully
 */
bool gateway_assistance_send_time_to_gnss(bool gnss_is_active);

/*!
 * @brief Send position to AG3335 GNSS module NVRAM
 *
 * When called while NOT in GNSS mode, this will:
 * 1. Power on the AG3335 module
 * 2. Wait for module ready
 * 3. Send PAIR600 position command
 * 4. Wait for ACK
 * 5. Power off the module
 *
 * When called during GNSS scan (module already powered), just sends command.
 *
 * @param [in] gnss_is_active true if GNSS scan is in progress (module powered)
 * @returns true if position was sent successfully
 */
bool gateway_assistance_send_position_to_gnss(bool gnss_is_active);

/*!
 * @brief Store own GNSS fix as fallback assistance data
 *
 * @param [in] lat Latitude from GNSS fix (scaled by 1e6)
 * @param [in] lon Longitude from GNSS fix (scaled by 1e6)
 */
void gateway_assistance_store_own_fix(int32_t lat, int32_t lon);

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
bool gateway_assistance_is_gnss_ready(void);

/*!
 * @brief Send warm start command to AG3335 (PAIR005)
 *
 * Warm start uses almanac data without requiring ephemeris download.
 * Best for MOB burst mode where speed is priority.
 * Requires: time sync + almanac + approximate position.
 *
 * @returns true if command was sent successfully
 */
bool gateway_assistance_send_warm_start(void);

/*!
 * @brief Send hot start command to AG3335 (PAIR004)
 *
 * Hot start uses ephemeris if available for fastest TTFF.
 * Best for PIW phases where device had recent fix.
 * Falls back to warm start behavior if ephemeris not available.
 *
 * @returns true if command was sent successfully
 */
bool gateway_assistance_send_hot_start(void);

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
bool gateway_assistance_send_test_position(void);

/*!
 * @brief Periodic almanac maintenance check (call every 24 hours while charging)
 *
 * When device is on charge, this should be called periodically (every 24 hours)
 * to check almanac status and perform maintenance if needed.
 *
 * The function will:
 * 1. Power on GNSS module
 * 2. Refresh almanac status via PAIR550
 * 3. If maintenance needed, run 750-second almanac download scan
 * 4. Power off GNSS module
 *
 * @returns true if maintenance was performed, false if not needed
 */
bool gateway_assistance_periodic_almanac_maintenance(void);

/*!
 * @brief Check if it's time for periodic almanac status check
 *
 * Returns true if:
 * - Device is on charge AND
 * - Last almanac check was >24 hours ago
 *
 * @returns true if periodic check should be performed
 */
bool gateway_assistance_should_check_almanac(void);

/*
 * -----------------------------------------------------------------------------
 * --- BACKGROUND GNSS MODE (Charging/Docked) ---------------------------------
 * -----------------------------------------------------------------------------
 * When device is charging/docked, we run continuous GNSS in background to
 * maintain almanac and ephemeris data. This coexists with normal tracking.
 */

/*!
 * @brief Check charge state and manage background GNSS mode
 *
 * Call this periodically (e.g., in main loop or on alarm).
 * Detects charge state transitions and:
 * - On charge connect: Starts background GNSS scan
 * - On charge disconnect: Stops background GNSS scan
 *
 * When background mode is active, normal tracking scans can still run
 * and will use the already-active GNSS module.
 */
void gateway_assistance_check_charge_state(void);

/*!
 * @brief Check if background GNSS mode is active
 *
 * @returns true if GNSS is running in background (charging mode)
 */
bool gateway_assistance_is_background_gnss_active(void);

/*!
 * @brief Start background GNSS mode manually
 *
 * Starts continuous GNSS scanning for almanac/ephemeris maintenance.
 * Normally called automatically when charge is detected.
 */
void gateway_assistance_start_background_gnss(void);

/*!
 * @brief Stop background GNSS mode manually
 *
 * Stops continuous GNSS scanning.
 * Normally called automatically when charge is removed.
 */
void gateway_assistance_stop_background_gnss(void);

#ifdef __cplusplus
}
#endif

#endif // GATEWAY_ASSISTANCE_H

/* --- EOF ------------------------------------------------------------------ */
