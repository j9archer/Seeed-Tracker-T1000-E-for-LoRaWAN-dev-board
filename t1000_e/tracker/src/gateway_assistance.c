/*!
 * @file      gateway_assistance.c
 *
 * @brief     Gateway position and time assistance for GNSS implementation
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "gateway_assistance.h"
#include "smtc_hal.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_config.h"
#include "smtc_modem_api.h"
#include "ag3335.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define POSITION_MAX_AGE_EXCELLENT_MIN  30   // Minutes
#define POSITION_MAX_AGE_GOOD_MIN       90   // Minutes  
#define POSITION_MAX_AGE_FAIR_MIN       180  // Minutes

#define GNSS_SCAN_DURATION_EXCELLENT    10   // Seconds
#define GNSS_SCAN_DURATION_GOOD         15   // Seconds
#define GNSS_SCAN_DURATION_FAIR         25   // Seconds
#define GNSS_SCAN_DURATION_COLD         60   // Seconds

// GNSS power up timing
#define GNSS_POWER_UP_DELAY_MS          500  // Time to wait after power on
#define GNSS_CMD_ACK_TIMEOUT_MS         200  // Time to wait for ACK

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static position_time_cache_t position_cache = {
    .valid = false
};

// Background GNSS mode state (for charging/docked mode)
static bool background_gnss_active = false;
static bool last_charge_state = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint8_t calculate_nmea_checksum(const char* sentence);
static void gnss_power_on_for_command(void);
static void gnss_power_off_after_command(void);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void gateway_assistance_init(void)
{
    memset(&position_cache, 0, sizeof(position_cache));
    position_cache.valid = false;
    position_cache.time_synced = false;
    
    HAL_DBG_TRACE_INFO("Gateway assistance system initialized\n");
}

bool gateway_assistance_handle_downlink(const uint8_t* payload, uint8_t size)
{
    if (payload == NULL || size < sizeof(gateway_position_msg_t)) {
        return false;
    }
    
    const gateway_position_msg_t* msg = (const gateway_position_msg_t*)payload;
    
    // Verify message type
    if (msg->msg_type != 0x01) {
        HAL_DBG_TRACE_WARNING("Unknown gateway message type: 0x%02X\n", msg->msg_type);
        return false;
    }
    
    // Extract position from downlink
    position_cache.latitude = msg->gateway_lat / 10000000.0f;
    position_cache.longitude = msg->gateway_lon / 10000000.0f;
    
    // Get current GPS time from modem (set via DeviceTimeReq)
    uint32_t gps_time_s = 0;
    uint32_t gps_frac_s = 0;
    if (smtc_modem_get_time(&gps_time_s, &gps_frac_s) == SMTC_MODEM_RC_OK) {
        // Convert GPS time to Unix time (GPS epoch Jan 6, 1980 = Unix 315964800)
        position_cache.unix_time = gps_time_s + 315964800 - 18; // subtract leap seconds
        position_cache.time_uncertainty = 2; // DeviceTimeReq typically ±1-2 seconds
        position_cache.time_rtc_at_sync = hal_rtc_get_time_s();
        position_cache.time_synced = true;
    } else {
        // Fallback to RTC if modem time not available
        position_cache.unix_time = hal_rtc_get_time_s();
        position_cache.time_uncertainty = 3600; // Large uncertainty without time sync
        // Don't mark as synced - this is just RTC time
    }
    
    position_cache.rtc_at_receipt = hal_rtc_get_time_s();
    position_cache.valid = true;
    
    HAL_DBG_TRACE_INFO("Gateway position received: %.6f, %.6f @ %u (GPS time)\n",
                       position_cache.latitude,
                       position_cache.longitude,
                       position_cache.unix_time);
    
    // Send PAIR600 command to AG3335 NVRAM (with power cycling if needed)
    gateway_assistance_send_position_to_gnss(false);
    
    return true;
}

const position_time_cache_t* gateway_assistance_get_cache(void)
{
    return &position_cache;
}

bool gateway_assistance_is_available(void)
{
    if (!position_cache.valid) {
        return false;
    }
    
    // Check if assistance is still useful (< 3 hours old)
    assistance_quality_t quality = gateway_assistance_get_quality();
    return quality != ASSISTANCE_POOR;
}

assistance_quality_t gateway_assistance_get_quality(void)
{
    if (!position_cache.valid) {
        return ASSISTANCE_POOR;
    }
    
    uint32_t current_rtc = hal_rtc_get_time_s();
    uint32_t age_seconds = current_rtc - position_cache.rtc_at_receipt;
    uint32_t age_minutes = age_seconds / 60;
    
    if (age_minutes < POSITION_MAX_AGE_EXCELLENT_MIN) {
        return ASSISTANCE_EXCELLENT;
    } else if (age_minutes < POSITION_MAX_AGE_GOOD_MIN) {
        return ASSISTANCE_GOOD;
    } else if (age_minutes < POSITION_MAX_AGE_FAIR_MIN) {
        return ASSISTANCE_FAIR;
    } else {
        return ASSISTANCE_POOR;
    }
}

uint32_t gateway_assistance_get_estimated_time(void)
{
    if (!position_cache.time_synced) {
        return hal_rtc_get_time_s();
    }
    
    uint32_t current_rtc = hal_rtc_get_time_s();
    uint32_t elapsed = current_rtc - position_cache.time_rtc_at_sync;
    
    return position_cache.unix_time + elapsed;
}

uint32_t gateway_assistance_get_time_uncertainty(void)
{
    // Check if time has been synced (from DeviceTimeReq or gateway downlink)
    if (!position_cache.time_synced) {
        return 3600; // 1 hour if no time sync
    }
    
    uint32_t current_rtc = hal_rtc_get_time_s();
    uint32_t elapsed_hours = (current_rtc - position_cache.time_rtc_at_sync) / 3600;
    
    // Base uncertainty + RTC drift (assume 3 seconds per day = 0.125 sec/hour)
    uint32_t drift_uncertainty = (elapsed_hours * 125) / 1000;
    
    return position_cache.time_uncertainty + drift_uncertainty;
}

uint32_t gateway_assistance_get_recommended_scan_duration(void)
{
    assistance_quality_t quality = gateway_assistance_get_quality();
    
    switch (quality) {
        case ASSISTANCE_EXCELLENT:
            return GNSS_SCAN_DURATION_EXCELLENT;
        case ASSISTANCE_GOOD:
            return GNSS_SCAN_DURATION_GOOD;
        case ASSISTANCE_FAIR:
            return GNSS_SCAN_DURATION_FAIR;
        default:
            return GNSS_SCAN_DURATION_COLD;
    }
}

bool gateway_assistance_is_charging(void)
{
    // Check if USB/charger is connected
    return (hal_gpio_get_value(CHARGER_ADC_DET) != 0);
}

bool gateway_assistance_needs_almanac_maintenance(uint32_t days_threshold)
{
    // Primary check: Use PAIR550 almanac status from GNSS module
    // This is the authoritative source - the module knows its own almanac state
    if (gnss_almanac_needs_maintenance()) {
        uint8_t sv_count = gnss_almanac_get_valid_sv_count();
        HAL_DBG_TRACE_INFO("Almanac maintenance needed - PAIR550 shows %u valid SVs\n", sv_count);
        return true;
    }
    
    // Secondary check: Time since last position update
    // Even with valid almanac, if we haven't had a fix in a long time,
    // the ephemeris will be stale and a maintenance scan could help
    if (position_cache.valid) {
        uint32_t current_time = hal_rtc_get_time_s();
        uint32_t time_since_fix = current_time - position_cache.rtc_at_receipt;
        uint32_t days_since_fix = time_since_fix / 86400;
        
        if (days_since_fix >= days_threshold) {
            HAL_DBG_TRACE_INFO("Almanac OK but ephemeris likely stale - %lu days since last fix\n", days_since_fix);
            return true;
        }
    }
    
    return false;
}

uint32_t gateway_assistance_get_almanac_scan_duration(void)
{
    // Extended duration for almanac download
    // 750 seconds (12.5 minutes) allows full almanac download from one satellite
    return 750;
}

bool gateway_assistance_send_time_to_gnss(bool gnss_is_active)
{
    // Send UTC time to AG3335 NVRAM via PAIR590
    // Format: $PAIR590,YYYY,MM,DD,hh,mm,ss*CS
    
    // Get current GPS time from modem (just updated by DeviceTimeAns)
    uint32_t gps_time_s = 0;
    uint32_t gps_frac_s = 0;
    
    if (smtc_modem_get_time(&gps_time_s, &gps_frac_s) != SMTC_MODEM_RC_OK) {
        HAL_DBG_TRACE_WARNING("Failed to get modem time for PAIR590\n");
        return false;
    }
    
    // Convert GPS time to Unix time (GPS epoch Jan 6, 1980 = Unix 315964800)
    // Current leap seconds between GPS and UTC = 18 seconds
    uint32_t unix_time = gps_time_s + 315964800 - 18;
    
    // Update position cache with new time - mark as synced!
    position_cache.unix_time = unix_time;
    position_cache.time_uncertainty = 2; // DeviceTimeReq typically ±1-2 seconds
    position_cache.time_rtc_at_sync = hal_rtc_get_time_s();
    position_cache.time_synced = true;
    
    HAL_DBG_TRACE_INFO("Time synced from modem - uncertainty now %lu s\n", 
                       position_cache.time_uncertainty);
    
    struct tm timeinfo;
    time_t unix_time_t = (time_t)unix_time;
    gmtime_r(&unix_time_t, &timeinfo);
    
    char time_cmd[80];
    snprintf(time_cmd, sizeof(time_cmd), "$PAIR590,%04d,%02d,%02d,%02d,%02d,%02d",
             timeinfo.tm_year + 1900,
             timeinfo.tm_mon + 1,
             timeinfo.tm_mday,
             timeinfo.tm_hour,
             timeinfo.tm_min,
             timeinfo.tm_sec);
    
    // Power cycle if GNSS not active
    if (!gnss_is_active) {
        HAL_DBG_TRACE_INFO("GNSS not active - powering on for PAIR590\n");
        gnss_power_on_for_command();
    }
    
    bool result = gnss_send_command(time_cmd);
    
    if (result) {
        HAL_DBG_TRACE_INFO("UTC time written to AG3335 NVRAM (PAIR590): %04d-%02d-%02d %02d:%02d:%02d\n",
                           timeinfo.tm_year + 1900,
                           timeinfo.tm_mon + 1,
                           timeinfo.tm_mday,
                           timeinfo.tm_hour,
                           timeinfo.tm_min,
                           timeinfo.tm_sec);
    } else {
        HAL_DBG_TRACE_WARNING("Failed to send PAIR590\n");
    }
    
    // Power off if we powered it on
    if (!gnss_is_active) {
        gnss_power_off_after_command();
    }
    
    return result;
}

bool gateway_assistance_send_position_to_gnss(bool gnss_is_active)
{
    if (!position_cache.valid) {
        HAL_DBG_TRACE_WARNING("No position data to send to GNSS\n");
        return false;
    }
    
    // Format: $PAIR600,lat,lon,height,accMaj,accMin,bear,accVert*CS
    // Using conservative 50m horizontal / 100m vertical accuracy
    char pos_cmd[120];
    snprintf(pos_cmd, sizeof(pos_cmd), 
             "$PAIR600,%.6f,%.6f,0.0,50.0,50.0,0.0,100.0",
             position_cache.latitude,
             position_cache.longitude);
    
    // Power cycle if GNSS not active
    if (!gnss_is_active) {
        HAL_DBG_TRACE_INFO("GNSS not active - powering on for PAIR600\n");
        gnss_power_on_for_command();
    }
    
    bool result = gnss_send_command(pos_cmd);
    
    if (result) {
        HAL_DBG_TRACE_INFO("Position written to AG3335 NVRAM (PAIR600): %.6f, %.6f\n",
                           position_cache.latitude,
                           position_cache.longitude);
    } else {
        HAL_DBG_TRACE_WARNING("Failed to send PAIR600\n");
    }
    
    // Power off if we powered it on
    if (!gnss_is_active) {
        gnss_power_off_after_command();
    }
    
    return result;
}

void gateway_assistance_store_own_fix(int32_t lat, int32_t lon)
{
    // Only update if no gateway data, or gateway data is very old
    if (position_cache.valid) {
        assistance_quality_t quality = gateway_assistance_get_quality();
        if (quality != ASSISTANCE_POOR) {
            // Gateway data is still good, don't override
            return;
        }
    }
    
    // Store own GNSS fix as fallback assistance
    position_cache.latitude = lat / 1000000.0f;
    position_cache.longitude = lon / 1000000.0f;
    position_cache.unix_time = hal_rtc_get_time_s();
    position_cache.rtc_at_receipt = hal_rtc_get_time_s();
    position_cache.time_uncertainty = gateway_assistance_get_time_uncertainty();
    position_cache.valid = true;
    
    HAL_DBG_TRACE_INFO("Stored own GNSS fix as assistance: %.6f, %.6f\n",
                       position_cache.latitude,
                       position_cache.longitude);
}

bool gateway_assistance_is_gnss_ready(void)
{
    // Check 1: Time sync - GPS time error < 3 seconds
    uint32_t time_uncertainty = gateway_assistance_get_time_uncertainty();
    if (time_uncertainty >= 3) {
        HAL_DBG_TRACE_INFO("GNSS not ready - time uncertainty %lu s (need <3s)\n", time_uncertainty);
        return false;
    }
    
    // Check 2: Valid almanac from PAIR550 status check
    if (!gnss_almanac_is_valid()) {
        uint8_t sv_count = gnss_almanac_get_valid_sv_count();
        HAL_DBG_TRACE_INFO("GNSS not ready - almanac invalid (%u SVs, need >=4 GPS)\n", sv_count);
        return false;
    }
    
    // Check 3: Recent position - valid position within 4 hours
    if (!position_cache.valid) {
        HAL_DBG_TRACE_INFO("GNSS not ready - no position data available\n");
        return false;
    }
    
    uint32_t current_rtc = hal_rtc_get_time_s();
    uint32_t age_seconds = current_rtc - position_cache.rtc_at_receipt;
    uint32_t age_hours = age_seconds / 3600;
    
    if (age_hours >= 4) {
        HAL_DBG_TRACE_INFO("GNSS not ready - position age %lu hours (need <4h)\n", age_hours);
        return false;
    }
    
    // All criteria met - GNSS ready for warm start
    uint8_t sv_count = gnss_almanac_get_valid_sv_count();
    HAL_DBG_TRACE_INFO("GNSS READY - time OK, almanac valid (%u SVs), position <4h old\n", sv_count);
    return true;
}

bool gateway_assistance_send_warm_start(void)
{
    // PAIR005: Warm start command
    // Uses almanac data without requiring ephemeris download
    // Significantly reduces TTFF when conditions are met
    // Best for MOB burst mode where speed is priority
    const char* warm_start_cmd = "$PAIR005";
    
    HAL_DBG_TRACE_INFO("Sending GNSS warm start command (PAIR005)\n");
    
    return gnss_send_command(warm_start_cmd);
}

bool gateway_assistance_send_hot_start(void)
{
    // PAIR004: Hot start command
    // Uses ephemeris if available for fastest TTFF
    // Falls back to warm start behavior if ephemeris not available
    // Best for PIW phases where device had recent fix
    const char* hot_start_cmd = "$PAIR004";
    
    HAL_DBG_TRACE_INFO("Sending GNSS hot start command (PAIR004)\n");
    
    return gnss_send_command(hot_start_cmd);
}

bool gateway_assistance_send_test_position(void)
{
    // TEMPORARY TEST POSITION - Algarve, Portugal
    // Latitude: 37.099775°N
    // Longitude: 8.460805°W (negative for West)
    // Altitude: 63m
    // Accuracy: 50m horizontal, 100m vertical
    
    char pos_cmd[96];
    snprintf(pos_cmd, sizeof(pos_cmd), 
             "$PAIR600,37.099775,-8.460805,63.0,50.0,50.0,0.0,100.0");
    
    HAL_DBG_TRACE_INFO("[TEST] Sending hardcoded position to AG3335 NVRAM\n");
    HAL_DBG_TRACE_INFO("[TEST] Position: 37.099775°N, 8.460805°W, altitude 63m\n");
    
    return gnss_send_command(pos_cmd);
}

bool gateway_assistance_should_check_almanac(void)
{
    // Only check while charging to preserve battery
    if (!gateway_assistance_is_charging()) {
        return false;
    }
    
    // Check if last almanac status check was more than 24 hours ago
    uint32_t last_check = gnss_almanac_get_last_check_time();
    uint32_t current_time = hal_rtc_get_time_s();
    
    // If never checked (last_check == 0) or >24 hours since last check
    uint32_t hours_since_check = (current_time - last_check) / 3600;
    
    if (last_check == 0 || hours_since_check >= 24) {
        HAL_DBG_TRACE_INFO("Almanac check due - %lu hours since last check (charging=%d)\n", 
                           hours_since_check, 1);
        return true;
    }
    
    return false;
}

bool gateway_assistance_periodic_almanac_maintenance(void)
{
    HAL_DBG_TRACE_INFO("=== Periodic Almanac Maintenance Check ===\n");
    
    // Power on GNSS module for status check
    gnss_power_on_for_command();
    
    // Refresh almanac status via PAIR550
    gnss_almanac_refresh_status();
    
    // Check if maintenance is needed
    bool maintenance_needed = gnss_almanac_needs_maintenance();
    
    if (maintenance_needed) {
        HAL_DBG_TRACE_INFO("Almanac maintenance required - starting 750s scan\n");
        
        // Keep module powered and run a full scan for almanac download
        // Note: We're already powered from gnss_power_on_for_command()
        // so we need to properly transition to scan mode
        
        gnss_fix_t fix;
        uint32_t scan_duration = gateway_assistance_get_almanac_scan_duration();
        
        // Run extended scan with relaxed quality requirements
        // We just want to keep the module on long enough to download almanac
        // Note: skip_power_management=false since this is not background mode
        bool got_fix = gnss_scan_until_good(scan_duration * 1000, 99.0f, 999.0f, &fix, false);
        
        if (got_fix && fix.valid) {
            HAL_DBG_TRACE_INFO("Almanac maintenance complete - got fix: %.6f, %.6f\n",
                               fix.latitude / 1000000.0, fix.longitude / 1000000.0);
            
            // Store the fix for future assistance
            gateway_assistance_store_own_fix(fix.latitude, fix.longitude);
        } else {
            HAL_DBG_TRACE_INFO("Almanac maintenance scan complete (no fix, but almanac should be updated)\n");
        }
        
        // Refresh status after scan to confirm almanac improvement
        gnss_almanac_refresh_status();
        
        gnss_power_off_after_command();
        return true;
    } else {
        uint8_t sv_count = gnss_almanac_get_valid_sv_count();
        HAL_DBG_TRACE_INFO("Almanac healthy - %u SVs valid for 14-day horizon, no maintenance needed\n", sv_count);
        
        gnss_power_off_after_command();
        return false;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint8_t calculate_nmea_checksum(const char* sentence)
{
    uint8_t checksum = 0;
    
    // Skip '$' and calculate XOR of all characters until '*' or end
    for (const char* p = sentence + 1; *p && *p != '*'; p++) {
        checksum ^= *p;
    }
    
    return checksum;
}

// NOTE: send_ag3335_command() has been moved to gnss_send_command() in ag3335.c
// All callers now use gnss_send_command() from ag3335.h

static void gnss_power_on_for_command(void)
{
    // Power on AG3335 module for NVRAM command
    // Same sequence as gnss_scan_start() but we don't init the full module
    
    // Enable VRTC (backup power)
    hal_gpio_set_value(AG3335_VRTC_EN, 1);
    hal_mcu_wait_ms(10);
    
    // Enable main power
    hal_gpio_set_value(AG3335_POWER_EN, 1);
    hal_mcu_wait_ms(50);
    
    // Release reset
    hal_gpio_set_value(AG3335_RESET, 1);
    
    // Wait for module to be ready
    hal_mcu_wait_ms(GNSS_POWER_UP_DELAY_MS);
    
    // Initialize UART for communication
    hal_uart_0_init();
    
    HAL_DBG_TRACE_INFO("AG3335 powered on for NVRAM command\n");
}

static void gnss_power_off_after_command(void)
{
    // Wait for any pending UART transmission
    hal_mcu_wait_ms(100);
    
    // Deinitialize UART
    hal_uart_0_deinit();
    
    // Assert reset
    hal_gpio_set_value(AG3335_RESET, 0);
    hal_mcu_wait_ms(10);
    
    // Power off main power (keep VRTC for NVRAM retention)
    hal_gpio_set_value(AG3335_POWER_EN, 0);
    
    HAL_DBG_TRACE_INFO("AG3335 powered off after NVRAM command\n");
}

/*
 * -----------------------------------------------------------------------------
 * --- BACKGROUND GNSS MODE (Charging/Docked) ---------------------------------
 * -----------------------------------------------------------------------------
 */

void gateway_assistance_check_charge_state(void)
{
    bool current_charge_state = gateway_assistance_is_charging();
    
    // Detect charge state transitions
    if (current_charge_state && !last_charge_state) {
        // Just connected to charger - start background GNSS
        HAL_DBG_TRACE_INFO("=== CHARGE CONNECTED - Starting background GNSS ===\n");
        gateway_assistance_start_background_gnss();
    }
    else if (!current_charge_state && last_charge_state) {
        // Just disconnected from charger - stop background GNSS
        HAL_DBG_TRACE_INFO("=== CHARGE DISCONNECTED - Stopping background GNSS ===\n");
        gateway_assistance_stop_background_gnss();
    }
    
    last_charge_state = current_charge_state;
}

bool gateway_assistance_is_background_gnss_active(void)
{
    return background_gnss_active;
}

void gateway_assistance_start_background_gnss(void)
{
    if (background_gnss_active) {
        HAL_DBG_TRACE_INFO("Background GNSS already active\n");
        return;
    }
    
    HAL_DBG_TRACE_INFO("Starting background GNSS mode (almanac/ephemeris maintenance)\n");
    
    // Start GNSS scan - this powers on the module and locks sleep
    gnss_scan_start();
    
    // Enable NMEA debug for background mode monitoring
    gnss_enable_nmea_debug(true);
    
    background_gnss_active = true;
    
    HAL_DBG_TRACE_INFO("Background GNSS mode ACTIVE - module running continuously\n");
}

void gateway_assistance_stop_background_gnss(void)
{
    if (!background_gnss_active) {
        HAL_DBG_TRACE_INFO("Background GNSS not active\n");
        return;
    }
    
    HAL_DBG_TRACE_INFO("Stopping background GNSS mode\n");
    
    // Disable NMEA debug
    gnss_enable_nmea_debug(false);
    
    // Stop GNSS scan - this powers off the module
    gnss_scan_stop();
    
    background_gnss_active = false;
    
    HAL_DBG_TRACE_INFO("Background GNSS mode STOPPED\n");
}

/* --- EOF ------------------------------------------------------------------ */
