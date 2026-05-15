
#ifndef __PERIPHERAL_AG3335_H__
#define __PERIPHERAL_AG3335_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief GNSS fix quality structure for MOB/PIW tracking
 */
typedef struct {
    int32_t  latitude;           // Latitude scaled by 1e6 (microdegrees)
    int32_t  longitude;          // Longitude scaled by 1e6 (microdegrees)
    int32_t  speed;              // Speed scaled by 1e6 (knots * 1e6)
    float    hdop;               // Horizontal Dilution of Precision (lower = better)
    float    hacc;               // Horizontal accuracy estimate in meters (from GST)
    uint8_t  fix_quality;        // GGA fix quality (0=invalid, 1=GPS, 2=DGPS, etc.)
    uint8_t  satellites;         // Number of satellites tracked
    bool     valid;              // Overall fix validity
} gnss_fix_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Init gnss
 */
void gnss_init( void );

/*!
 * @brief Start gnss scan
 */
bool gnss_scan_start( void );

/*!
 * @brief Stop gnss scan
 */
void gnss_scan_stop( void );

/*!
 * @brief Get gnss fix status
 * 
 * @return Fix status
 */
bool gnss_get_fix_status( void );

/*!
 * @brief Get gnss fix status
 * 
 * @param [out] lat Pointer to buffer to be saved for latitude
 * @param [out] lon Pointer to buffer to be saved for longitude
 */
void gnss_get_position( int32_t *lat, int32_t *lon );

/*!
 * @brief Parse gnss nmea data
 * 
 * @param [in] nmea Pointer to buffer to be parsed
 */
void gnss_parse_handler( char *nmea );

/*!
 * @brief Get current fix with quality metrics
 * 
 * Retrieves position along with HDOP and horizontal accuracy estimate.
 * Use this for quality-driven scanning decisions.
 * 
 * @param [out] fix Pointer to gnss_fix_t structure to populate
 * @returns true if valid fix available
 */
bool gnss_get_quality_fix( gnss_fix_t *fix );

/*!
 * @brief Quality-driven GNSS scan with early exit
 * 
 * Runs GNSS scan with 1 Hz quality checks. Exits early when fix meets
 * quality thresholds or timeout is reached.
 * 
 * @param [in]  max_ms                Maximum scan duration in milliseconds
 * @param [in]  max_hdop              Maximum acceptable HDOP (e.g., 3.0)
 * @param [in]  max_hacc              Maximum acceptable horizontal accuracy in meters (e.g., 15.0)
 * @param [out] fix                   Pointer to gnss_fix_t to store result
 * @param [in]  skip_power_management If true, don't call gnss_scan_start/stop (for background mode)
 * @returns true if good fix obtained, false if timeout or no fix
 */
bool gnss_scan_until_good( uint32_t max_ms, float max_hdop, float max_hacc, gnss_fix_t *fix, bool skip_power_management );

/*!
 * @brief Check if BLE beacon was found (for scan interruption)
 * 
 * Used by quality-driven scan to check if BLE scan found a beacon,
 * allowing early GNSS termination.
 * 
 * @returns true if BLE beacon found
 */
bool gnss_check_ble_interrupt( void );

/*!
 * @brief Set BLE interrupt flag (called from BLE scan callback)
 * 
 * @param [in] found true if BLE beacon found
 */
void gnss_set_ble_found( bool found );

/*!
 * @brief Enable/disable NMEA debug output
 * 
 * When enabled, raw NMEA sentences (GGA, GST, RMC) are printed
 * to debug console for quality verification. Useful during MOB
 * burst and PIW Phase 1 modes.
 * 
 * @param [in] enable true to enable debug output
 */
void gnss_enable_nmea_debug( bool enable );

/*!
 * @brief Check if almanac is valid for reliable GNSS operation
 * 
 * Based on PAIR550 status query. Returns true if at least 4 GPS
 * satellites have valid almanac data for the 14-day horizon.
 * 
 * @returns true if almanac is valid for warm/hot start
 */
bool gnss_almanac_is_valid( void );

/*!
 * @brief Check if almanac maintenance is needed
 * 
 * Returns true if almanac data is stale or insufficient for
 * reliable GNSS operation. Consider running a 750-second scan
 * to download fresh almanac.
 * 
 * @returns true if maintenance scan is recommended
 */
bool gnss_almanac_needs_maintenance( void );

/*!
 * @brief Get count of satellites with valid almanac
 * 
 * Returns total count across GPS, GLONASS, and BeiDou constellations
 * based on last PAIR550 query.
 * 
 * @returns Number of satellites with valid almanac (0 if unknown)
 */
uint8_t gnss_almanac_get_valid_sv_count( void );

/*!
 * @brief Get RTC timestamp of last almanac status check
 * 
 * @returns RTC time in seconds when PAIR550 was last queried
 */
uint32_t gnss_almanac_get_last_check_time( void );

/*!
 * @brief Refresh almanac status by querying GNSS module
 * 
 * Call this when GNSS is powered and UART is initialized
 * to get updated almanac status.
 */
void gnss_almanac_refresh_status( void );

/*!
 * @brief Send a PAIR command to the AG3335 GNSS module
 * 
 * Automatically calculates checksum and formats the command.
 * Sends once and waits for response.
 * 
 * @param [in] command Command string without checksum (e.g., "$PAIR081")
 * @returns true (always, actual ACK is parsed asynchronously)
 */
bool gnss_send_command( const char* command );

#ifdef __cplusplus
}
#endif

#endif
