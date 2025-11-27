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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint8_t calculate_nmea_checksum(const char* sentence);
static bool send_ag3335_command(const char* command);
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
    } else {
        // Fallback to RTC if modem time not available
        position_cache.unix_time = hal_rtc_get_time_s();
        position_cache.time_uncertainty = 3600; // Large uncertainty without time sync
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
    if (!position_cache.valid) {
        return hal_rtc_get_time_s();
    }
    
    uint32_t current_rtc = hal_rtc_get_time_s();
    uint32_t elapsed = current_rtc - position_cache.rtc_at_receipt;
    
    return position_cache.unix_time + elapsed;
}

uint32_t gateway_assistance_get_time_uncertainty(void)
{
    if (!position_cache.valid) {
        return 3600; // 1 hour if no data
    }
    
    uint32_t current_rtc = hal_rtc_get_time_s();
    uint32_t elapsed_hours = (current_rtc - position_cache.rtc_at_receipt) / 3600;
    
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
    if (!position_cache.valid) {
        // No fix ever received - almanac is likely stale, maintenance needed
        HAL_DBG_TRACE_INFO("Almanac maintenance needed - no prior fix\n");
        return true;
    }
    
    uint32_t current_time = hal_rtc_get_time_s();
    uint32_t time_since_fix = current_time - position_cache.rtc_at_receipt;
    uint32_t days_since_fix = time_since_fix / 86400;
    
    if (days_since_fix >= days_threshold) {
        HAL_DBG_TRACE_INFO("Almanac maintenance needed - %lu days since last fix\n", days_since_fix);
        return true;
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
    
    // Update position cache with new time
    position_cache.unix_time = unix_time;
    position_cache.time_uncertainty = 2; // DeviceTimeReq typically ±1-2 seconds
    
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
    
    bool result = send_ag3335_command(time_cmd);
    
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
    
    bool result = send_ag3335_command(pos_cmd);
    
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
    
    // Check 2: Fresh almanac - GNSS fix within 14 days
    if (gateway_assistance_needs_almanac_maintenance(14)) {
        HAL_DBG_TRACE_INFO("GNSS not ready - almanac maintenance needed (>14 days since fix)\n");
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
    HAL_DBG_TRACE_INFO("GNSS READY - time OK, almanac fresh, position <4h old\n");
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
    
    return send_ag3335_command(warm_start_cmd);
}

bool gateway_assistance_send_hot_start(void)
{
    // PAIR004: Hot start command
    // Uses ephemeris if available for fastest TTFF
    // Falls back to warm start behavior if ephemeris not available
    // Best for PIW phases where device had recent fix
    const char* hot_start_cmd = "$PAIR004";
    
    HAL_DBG_TRACE_INFO("Sending GNSS hot start command (PAIR004)\n");
    
    return send_ag3335_command(hot_start_cmd);
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
    
    return send_ag3335_command(pos_cmd);
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

static bool send_ag3335_command(const char* command)
{
    char full_command[128];
    uint8_t checksum = calculate_nmea_checksum(command);
    
    // AG3335 GNSS Module Assistance Capabilities:
    // - PAIR004: Hot start (uses available ephemeris/almanac)
    // - PAIR005: Warm start (no ephemeris, uses almanac)
    // - PAIR006: Cold start (no assistance data)
    // - PAIR010: Request aiding data from network
    // - PAIR496-509: EPOC orbit prediction
    // - PAIR590: UTC time reference (writes to NVRAM)
    // - PAIR600: Reference position (writes to NVRAM)
    //
    // Expected ACK format: $PAIR001,<cmd_num>,0*CS\r\n (0 = success)
    // Example: $PAIR001,590,0*37 for PAIR590 success
    //
    // Format for NMEA commands: $PAIR_CMD,params*checksum\r\n
    snprintf(full_command, sizeof(full_command), "%s*%02X\r\n", command, checksum);
    
    HAL_DBG_TRACE_INFO("Sending AG3335 command: %s*%02X (x3 for reliability)\n", command, checksum);
    
    // Send command multiple times for reliability
    for (uint8_t i = 0; i < 3; i++) {
        hal_uart_0_tx((uint8_t*)full_command, strlen(full_command));
        hal_mcu_wait_ms(GNSS_CMD_ACK_TIMEOUT_MS);
    }
    
    return true;
}

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

/* --- EOF ------------------------------------------------------------------ */
