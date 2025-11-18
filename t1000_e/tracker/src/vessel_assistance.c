/*!
 * @file      vessel_assistance.c
 *
 * @brief     Vessel position and time assistance for GNSS implementation
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "vessel_assistance.h"
#include "smtc_hal.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_config.h"
#include "ag3335.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void vessel_assistance_init(void)
{
    memset(&position_cache, 0, sizeof(position_cache));
    position_cache.valid = false;
    
    HAL_DBG_TRACE_INFO("Vessel assistance system initialized\n");
}

bool vessel_assistance_handle_downlink(const uint8_t* payload, uint8_t size)
{
    if (payload == NULL || size < sizeof(vessel_position_msg_t)) {
        return false;
    }
    
    const vessel_position_msg_t* msg = (const vessel_position_msg_t*)payload;
    
    // Verify message type
    if (msg->msg_type != 0x01) {
        HAL_DBG_TRACE_WARNING("Unknown vessel message type: 0x%02X\n", msg->msg_type);
        return false;
    }
    
    // Extract position and time
    position_cache.latitude = msg->vessel_lat / 10000000.0f;
    position_cache.longitude = msg->vessel_lon / 10000000.0f;
    position_cache.unix_time = msg->unix_time;
    position_cache.rtc_at_receipt = hal_rtc_get_time_s();
    position_cache.time_uncertainty = 60; // Assume ±60 seconds from potential queuing
    position_cache.valid = true;
    
    // Update system RTC with vessel time
    hal_rtc_set_time_s(msg->unix_time);
    
    HAL_DBG_TRACE_INFO("Vessel position received: %.6f, %.6f @ %u\n",
                       position_cache.latitude,
                       position_cache.longitude,
                       position_cache.unix_time);
    
    return true;
}

const position_time_cache_t* vessel_assistance_get_cache(void)
{
    return &position_cache;
}

bool vessel_assistance_is_available(void)
{
    if (!position_cache.valid) {
        return false;
    }
    
    // Check if assistance is still useful (< 3 hours old)
    assistance_quality_t quality = vessel_assistance_get_quality();
    return quality != ASSISTANCE_POOR;
}

assistance_quality_t vessel_assistance_get_quality(void)
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

uint32_t vessel_assistance_get_estimated_time(void)
{
    if (!position_cache.valid) {
        return hal_rtc_get_time_s();
    }
    
    uint32_t current_rtc = hal_rtc_get_time_s();
    uint32_t elapsed = current_rtc - position_cache.rtc_at_receipt;
    
    return position_cache.unix_time + elapsed;
}

uint32_t vessel_assistance_get_time_uncertainty(void)
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

uint32_t vessel_assistance_get_recommended_scan_duration(void)
{
    assistance_quality_t quality = vessel_assistance_get_quality();
    
    switch (quality) {
        case ASSISTANCE_QUALITY_EXCELLENT:
            return GNSS_SCAN_DURATION_EXCELLENT;
        case ASSISTANCE_QUALITY_GOOD:
            return GNSS_SCAN_DURATION_GOOD;
        case ASSISTANCE_QUALITY_FAIR:
            return GNSS_SCAN_DURATION_FAIR;
        default:
            return GNSS_SCAN_DURATION_COLD;
    }
}

bool vessel_assistance_is_charging(void)
{
    // Check if USB/charger is connected
    return (hal_gpio_get_value(CHARGER_ADC_DET) != 0);
}

bool vessel_assistance_needs_almanac_maintenance(uint32_t days_threshold)
{
    if (!g_position_cache.valid) {
        return false;  // No reference time available
    }
    
    uint32_t current_time = hal_rtc_get_time_s();
    uint32_t time_since_fix = current_time - g_position_cache.rtc_at_receipt;
    uint32_t days_since_fix = time_since_fix / 86400;
    
    return (days_since_fix >= days_threshold);
}

uint32_t vessel_assistance_get_almanac_scan_duration(void)
{
    // Extended duration for almanac download
    // 750 seconds (12.5 minutes) allows full almanac download from one satellite
    return 750;
}

bool vessel_assistance_apply_to_gnss(void)
{
    if (!vessel_assistance_is_available()) {
        HAL_DBG_TRACE_INFO("No vessel assistance available for GNSS\n");
        return false;
    }
    
    assistance_quality_t quality = vessel_assistance_get_quality();
    const char* quality_str = (quality == ASSISTANCE_EXCELLENT) ? "EXCELLENT" :
                             (quality == ASSISTANCE_GOOD) ? "GOOD" : "FAIR";
    
    HAL_DBG_TRACE_INFO("Applying vessel assistance (%s): %.6f, %.6f\n",
                       quality_str,
                       position_cache.latitude,
                       position_cache.longitude);
    
    // Send assistance position to AG3335 via PAIR062 command
    char command[80];
    snprintf(command, sizeof(command), "$PAIR062,%.6f,%.6f",
             position_cache.latitude, position_cache.longitude);
    
    if (send_ag3335_command(command)) {
        HAL_DBG_TRACE_INFO("Position assistance sent to AG3335\n");
        return true;
    } else {
        HAL_DBG_TRACE_WARNING("Failed to send position assistance\n");
        return false;
    }
}

void vessel_assistance_store_own_fix(int32_t lat, int32_t lon)
{
    // Only update if no vessel data, or vessel data is very old
    if (position_cache.valid) {
        assistance_quality_t quality = vessel_assistance_get_quality();
        if (quality != ASSISTANCE_POOR) {
            // Vessel data is still good, don't override
            return;
        }
    }
    
    // Store own GNSS fix as fallback assistance
    position_cache.latitude = lat / 1000000.0f;
    position_cache.longitude = lon / 1000000.0f;
    position_cache.unix_time = hal_rtc_get_time_s();
    position_cache.rtc_at_receipt = hal_rtc_get_time_s();
    position_cache.time_uncertainty = vessel_assistance_get_time_uncertainty();
    position_cache.valid = true;
    
    HAL_DBG_TRACE_INFO("Stored own GNSS fix as assistance: %.6f, %.6f\n",
                       position_cache.latitude,
                       position_cache.longitude);
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
    char full_command[96];
    uint8_t checksum = calculate_nmea_checksum(command);
    
    // Format: $PAIR062,lat,lon*checksum\r\n
    snprintf(full_command, sizeof(full_command), "%s*%02X\r\n", command, checksum);
    
    // Send command multiple times for reliability
    for (uint8_t i = 0; i < 3; i++) {
        hal_uart_0_tx((uint8_t*)full_command, strlen(full_command));
        hal_mcu_wait_ms(100);
    }
    
    return true;
}

bool vessel_assistance_is_charging(void)
{
    // Check if USB/charger is connected
    return (hal_gpio_get_value(CHARGER_ADC_DET) != 0);
}

bool vessel_assistance_needs_almanac_maintenance(uint32_t days_threshold)
{
    if (!g_position_cache.valid) {
        return false;  // No reference time available
    }
    
    uint32_t current_time = hal_rtc_get_time_s();
    uint32_t time_since_fix = current_time - g_position_cache.rtc_at_receipt;
    uint32_t days_since_fix = time_since_fix / 86400;
    
    return (days_since_fix >= days_threshold);
}

uint32_t vessel_assistance_get_almanac_scan_duration(void)
{
    // Extended duration for almanac download
    // 750 seconds (12.5 minutes) allows full almanac download from one satellite
    return 750;
}

/* --- EOF ------------------------------------------------------------------ */
