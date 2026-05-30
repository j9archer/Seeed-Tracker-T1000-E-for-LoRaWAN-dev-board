/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <string.h>
#include "lr11xx_wifi.h"
#include "lr11xx_system.h"
#include "wifi_helpers.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define TIMESTAMP_AP_PHONE_FILTERING 18000
#define WIFI_CORRELATION_UA ( 12000 )
#define WIFI_CAPTURE_UA ( 12000 )
#define WIFI_DEMODULATION_UA ( 4000 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

static wifi_settings_t settings = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void smtc_wifi_log_basic_result( const char* label, uint8_t index,
                                        const lr11xx_wifi_basic_complete_result_t* result,
                                        lr11xx_wifi_channel_t channel,
                                        lr11xx_wifi_signal_type_result_t type,
                                        lr11xx_wifi_mac_origin_t origin,
                                        bool rssi_validity )
{
    HAL_DBG_TRACE_PRINTF( "%s %u: ", label, index );
    for( uint8_t mac_index = 0; mac_index < LR11XX_WIFI_MAC_ADDRESS_LENGTH; mac_index++ )
    {
        HAL_DBG_TRACE_PRINTF( "%02X ", result->mac_address[mac_index] );
    }
    HAL_DBG_TRACE_PRINTF( "Channel: %u, Type: %u, RSSI: %d, Origin: %u, RSSI valid: %u\r\n",
                          channel, type, result->rssi, origin, rssi_validity ? 1 : 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void smtc_wifi_settings_init( const wifi_settings_t* wifi_settings )
{
    /* Set current context Wi-Fi settings */
    memcpy( &settings, wifi_settings, sizeof settings );
}

bool radio_configure_for_scan( const void* radio_context )
{
    lr11xx_status_t           status;
    lr11xx_system_errors_t    errors;
    lr11xx_system_lfclk_cfg_t lf_clock_cfg = LR11XX_SYSTEM_LFCLK_EXT;

    // Clear potential old errors
    status = lr11xx_system_clear_errors( radio_context );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Fail to clear error\n" );
        return false;
    }

    // Configure lf clock
    status = lr11xx_system_cfg_lfclk( radio_context, lf_clock_cfg, true );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Fail to config lfclk\n" );
        return false;
    }

    // Get errors
    status = lr11xx_system_get_errors( radio_context, &errors );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Fail to get lr11xx error\n" );
        return false;
    }

    // In case clock config is XTAL check if there is no LF_XOSC_START error
    if( ( lf_clock_cfg == LR11XX_SYSTEM_LFCLK_XTAL ) &&
        ( ( errors & LR11XX_SYSTEM_ERRORS_LF_XOSC_START_MASK ) == LR11XX_SYSTEM_ERRORS_LF_XOSC_START_MASK ) )
    {
        // lr11xx specification is telling to reset the radio to fix this error
        return false;
    }

    return true;
}

bool smtc_wifi_start_scan( const void* radio_context )
{
    lr11xx_status_t status;

    HAL_DBG_TRACE_INFO( "start Wi-Fi scan\n" );

    if( radio_configure_for_scan( radio_context ) == true )
    {
        status = lr11xx_wifi_cfg_timestamp_ap_phone( radio_context, TIMESTAMP_AP_PHONE_FILTERING );
        if( status != LR11XX_STATUS_OK )
        {
            HAL_DBG_TRACE_ERROR( "Failed to configure timestamp ap phone\n" );
            return false;
        }

        status = lr11xx_wifi_scan_time_limit( radio_context, settings.types, settings.channels,
                                              LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT, settings.max_results,
                                              settings.timeout_per_channel, settings.timeout_per_scan );
        if( status != LR11XX_STATUS_OK )
        {
            HAL_DBG_TRACE_ERROR( "Failed to start Wi-Fi scan\n" );
            return false;
        }
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "Failed to configure LR11XX for Wi-Fi scan\n" );
        return false;
    }

    return true;
}

void smtc_wifi_scan_ended( void )
{
    /* Disable the Wi-Fi path */
}

bool smtc_wifi_get_results( const void* radio_context, wifi_scan_all_result_t* wifi_results )
{
    lr11xx_wifi_basic_complete_result_t wifi_results_mac_addr[WIFI_MAX_RESULTS] = { 0 };
    uint8_t                             nb_results;
    uint8_t                             max_nb_results;
    uint8_t                             result_index = 0;
    lr11xx_status_t                     status       = LR11XX_STATUS_OK;

    status = lr11xx_wifi_get_nb_results( radio_context, &nb_results );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get Wi-Fi scan number of results\n" );
        return false;
    }

    /* check if the array is big enough to hold all results */
    max_nb_results = sizeof( wifi_results_mac_addr ) / sizeof( wifi_results_mac_addr[0] );
    if( nb_results > max_nb_results )
    {
        HAL_DBG_TRACE_ERROR( "Wi-Fi scan result size exceeds %u (%u)\n", max_nb_results, nb_results );
        return false;
    }

    wifi_results->raw_results = nb_results;

    status = lr11xx_wifi_read_basic_complete_results( radio_context, 0, nb_results, wifi_results_mac_addr );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to read Wi-Fi scan results\n" );
        return false;
    }

    /* add scan to results */
    for( uint8_t index = 0; index < nb_results; index++ )
    {
        const lr11xx_wifi_basic_complete_result_t* local_basic_result = &wifi_results_mac_addr[index];
        lr11xx_wifi_channel_t                      channel;
        bool                                       rssi_validity;
        lr11xx_wifi_mac_origin_t                   mac_origin_estimation;
        lr11xx_wifi_signal_type_result_t           signal_type;

        lr11xx_wifi_parse_channel_info( local_basic_result->channel_info_byte, &channel, &rssi_validity,
                                        &mac_origin_estimation );
        signal_type = lr11xx_wifi_extract_signal_type_from_data_rate_info( local_basic_result->data_rate_info_byte );

        HAL_DBG_TRACE_PRINTF( "WiFi raw result %u: origin=%u, rssi_valid=%u\r\n",
                              index, mac_origin_estimation, rssi_validity ? 1 : 0 );

        if( mac_origin_estimation == LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP )
        {
            smtc_wifi_log_basic_result( "WiFi reject mobile AP", index, local_basic_result, channel, signal_type,
                                        mac_origin_estimation, rssi_validity );
            wifi_results->mobile_ap_results++;
            continue;
        }

        if( ( local_basic_result->mac_address[0] & 0x02 ) != 0 )
        {
            smtc_wifi_log_basic_result( "WiFi reject local-admin", index, local_basic_result, channel, signal_type,
                                        mac_origin_estimation, rssi_validity );
            wifi_results->local_admin_results++;
            continue;
        }

        bool duplicate = false;
        for( uint8_t accepted_index = 0; accepted_index < result_index; accepted_index++ )
        {
            if( memcmp( wifi_results->results[accepted_index].mac_address, local_basic_result->mac_address,
                        LR11XX_WIFI_MAC_ADDRESS_LENGTH ) == 0 )
            {
                duplicate = true;
                break;
            }
        }

        if( duplicate )
        {
            wifi_results->duplicate_results++;
            continue;
        }

        if( result_index >= WIFI_MAX_RESULTS )
        {
            break;
        }

        wifi_results->results[result_index].channel = channel;

        wifi_results->results[result_index].type = signal_type;

        memcpy( wifi_results->results[result_index].mac_address, local_basic_result->mac_address,
                LR11XX_WIFI_MAC_ADDRESS_LENGTH );

        wifi_results->results[result_index].rssi          = local_basic_result->rssi;
        wifi_results->results[result_index].rssi_validity = rssi_validity;
        wifi_results->results[result_index].origin        = mac_origin_estimation;
        if( mac_origin_estimation == LR11XX_WIFI_ORIGIN_UNKNOWN )
        {
            wifi_results->unknown_origin_results++;
        }
        wifi_results->nbr_results++;
        result_index++;
    }

    return true;
}

bool smtc_wifi_get_power_consumption( const void* radio_context, uint32_t* power_consumption_uah )
{
    lr11xx_status_t                  status;
    lr11xx_wifi_cumulative_timings_t timing;
    lr11xx_system_reg_mode_t         reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC;

    status = lr11xx_wifi_read_cumulative_timing( radio_context, &timing );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get wifi timings\n" );
        return false;
    }

    *power_consumption_uah = ( uint32_t ) lr11xx_wifi_get_consumption( reg_mode, timing );

    /* Accumulate timings until there is a significant amount of energy consumed */
    if( *power_consumption_uah > 0 )
    {
        status = lr11xx_wifi_reset_cumulative_timing( radio_context );
        if( status != LR11XX_STATUS_OK )
        {
            HAL_DBG_TRACE_ERROR( "Failed to reset wifi timings\n" );
            return false;
        }
    }

    return true;
}

bool smtc_wifi_get_power_consumption_details( const void* radio_context, wifi_scan_all_result_t* result )
{
    lr11xx_status_t                  status;
    lr11xx_wifi_cumulative_timings_t timing;
    lr11xx_system_reg_mode_t         reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC;
    uint64_t                         active_uA_us;
    uint32_t                         active_us;

    if( result == NULL )
    {
        return false;
    }

    status = lr11xx_wifi_read_cumulative_timing( radio_context, &timing );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get wifi timings\n" );
        return false;
    }

    result->rx_detection_us   = timing.rx_detection_us;
    result->rx_correlation_us = timing.rx_correlation_us;
    result->rx_capture_us     = timing.rx_capture_us;
    result->demodulation_us   = timing.demodulation_us;

    active_uA_us = ( ( uint64_t ) timing.rx_capture_us * WIFI_CAPTURE_UA ) +
                   ( ( uint64_t ) timing.demodulation_us * WIFI_DEMODULATION_UA ) +
                   ( ( uint64_t ) timing.rx_correlation_us * WIFI_CORRELATION_UA );
    active_us = timing.rx_capture_us + timing.demodulation_us + timing.rx_correlation_us;
    result->power_consumption_nah = ( uint32_t )( active_uA_us / ( 3600000 - ( active_us / 1000 ) ) );
    result->power_consumption_uah = result->power_consumption_nah / 1000;

    if( reg_mode == LR11XX_SYSTEM_REG_MODE_LDO )
    {
        result->power_consumption_nah *= 2;
    }

    status = lr11xx_wifi_reset_cumulative_timing( radio_context );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to reset wifi timings\n" );
        return false;
    }

    return true;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
