
#include "ralf.h"
#include "lr11xx_system.h"
#include "smtc_modem_hal.h"
#include "wifi_helpers.h"
#include "smtc_hal_dbg_trace.h"
#include "wifi_scan.h"

/**
 * @brief Size in bytes to store the RSSI of a detected WiFi Access-Point
 */
#define WIFI_AP_RSSI_SIZE ( 1 )

/**
 * @brief Size in bytes of a WiFi Access-Point address
 */
#define WIFI_AP_ADDRESS_SIZE ( 6 )

#define WIFI_CHANNEL_MASK_1_13 ( 0x1FFF )
#define WIFI_CHANNEL_MASK_1_6_11 \
    ( LR11XX_WIFI_CHANNEL_1_MASK | LR11XX_WIFI_CHANNEL_6_MASK | LR11XX_WIFI_CHANNEL_11_MASK )

/*!
 * @brief Results of the current Wi-Fi scan
 */
wifi_scan_all_result_t wifi_results;

/*!
 * @brief The buffer containing results to be sent over the air
 */
uint8_t wifi_result_buffer[4+( WIFI_AP_RSSI_SIZE + WIFI_AP_ADDRESS_SIZE ) * WIFI_MAX_RESULTS + 4];

static uint8_t wifi_scan_current_max_results = WIFI_SCAN_TARGET_RESULTS;
static uint8_t wifi_scan_next_max_results    = WIFI_SCAN_TARGET_RESULTS;
static uint8_t wifi_scan_last_good_channel   = 0;
static bool    wifi_scan_next_remainder      = false;
static bool    wifi_scan_current_remainder   = false;

static lr11xx_wifi_channel_mask_t wifi_scan_channel_mask_for_channel( uint8_t channel )
{
    if( ( channel < 1 ) || ( channel > 13 ) )
    {
        return 0;
    }

    return ( lr11xx_wifi_channel_mask_t )( 1U << ( channel - 1 ) );
}

static lr11xx_wifi_channel_mask_t wifi_scan_primary_channel_mask( void )
{
    return WIFI_CHANNEL_MASK_1_6_11 | wifi_scan_channel_mask_for_channel( wifi_scan_last_good_channel );
}

static lr11xx_wifi_channel_mask_t wifi_scan_remainder_channel_mask( void )
{
    return WIFI_CHANNEL_MASK_1_13 & ( lr11xx_wifi_channel_mask_t )( ~wifi_scan_primary_channel_mask( ) );
}

bool wifi_scan_start( ralf_t* modem_radio )
{
    return wifi_scan_start_with_max_results( modem_radio, wifi_scan_next_max_results );
}

bool wifi_scan_start_with_max_results( ralf_t* modem_radio, uint8_t max_results )
{
    wifi_settings_t wifi_settings = { 0 };
    lr11xx_wifi_channel_mask_t channel_mask;

    if( max_results < WIFI_SCAN_TARGET_RESULTS )
    {
        max_results = WIFI_SCAN_TARGET_RESULTS;
    }
    if( max_results > WIFI_SCAN_ADAPTIVE_MAX_RESULTS )
    {
        max_results = WIFI_SCAN_ADAPTIVE_MAX_RESULTS;
    }
    wifi_scan_current_max_results = max_results;
    wifi_scan_current_remainder   = wifi_scan_next_remainder;
    wifi_scan_next_remainder      = false;
    channel_mask = wifi_scan_current_remainder ? wifi_scan_remainder_channel_mask( ) : wifi_scan_primary_channel_mask( );
    if( channel_mask == 0 )
    {
        channel_mask = WIFI_CHANNEL_MASK_1_13;
    }

    /* Init settings */
    wifi_settings.channels            = channel_mask;
    wifi_settings.types               = LR11XX_WIFI_TYPE_SCAN_B_G_N;
    wifi_settings.max_results         = max_results;
    wifi_settings.timeout_per_channel = WIFI_TIMEOUT_PER_CHANNEL_DEFAULT;
    wifi_settings.timeout_per_scan    = WIFI_TIMEOUT_PER_SCAN_DEFAULT;
    smtc_wifi_settings_init( &wifi_settings );
    
    /* Start WIFI scan */
    HAL_DBG_TRACE_PRINTF( "WiFi scan START (max_results=%u, channels=0x%04X, stage=%s, last_good=%u)\r\n",
                          max_results, channel_mask, wifi_scan_current_remainder ? "remainder" : "primary",
                          wifi_scan_last_good_channel );
    if( smtc_wifi_start_scan( modem_radio->ral.context ) != true )
    {
        HAL_DBG_TRACE_PRINTF( "RP_TASK_WIFI - failed to start scan, abort task\n" );
        return false;
    }
    return true;
}

bool wifi_scan_should_scan_remainder_channels( void )
{
    return ( wifi_scan_current_remainder == false ) && ( wifi_results.raw_results == 0 );
}

void wifi_scan_prepare_remainder_channels( void )
{
    wifi_scan_next_remainder = true;
}

bool wifi_get_results( ralf_t* modem_radio, uint8_t* result, uint8_t *size )
{
    bool scan_results_rc;

    /* Reset previous results */
    memset( &wifi_results, 0, sizeof( wifi_results ));

    /* Wi-Fi scan completed, get and display the results */
    scan_results_rc = smtc_wifi_get_results( modem_radio->ral.context, &wifi_results );

    /* Get scan power consumption */
    smtc_wifi_get_power_consumption_details( modem_radio->ral.context, &wifi_results );

    if( scan_results_rc == true )
    {
        if( wifi_results.nbr_results )
        {
            uint8_t wifi_buffer_size = 0;

            /* Concatenate all results in send buffer */
            for( uint8_t i = 0; i < wifi_results.nbr_results; i++ )
            {
                /* Copy Access Point MAC address in result buffer */
                memcpy( &wifi_result_buffer[wifi_buffer_size], wifi_results.results[i].mac_address, WIFI_AP_ADDRESS_SIZE );
                wifi_buffer_size += WIFI_AP_ADDRESS_SIZE;

                /* Copy Access Point RSSI address in result buffer (if requested) */
                wifi_result_buffer[wifi_buffer_size] = wifi_results.results[i].rssi;
                wifi_buffer_size += WIFI_AP_RSSI_SIZE;
            }

            // if( wifi_results.nbr_results < WIFI_MAX_RESULTS )
            // {
            //     for( uint8_t i = 0; i < ( WIFI_MAX_RESULTS - wifi_results.nbr_results ); i++  )
            //     {
            //         memset( &wifi_result_buffer[wifi_buffer_size], 0xff, ( WIFI_AP_ADDRESS_SIZE + WIFI_AP_RSSI_SIZE ));
            //         wifi_buffer_size += ( WIFI_AP_ADDRESS_SIZE + WIFI_AP_RSSI_SIZE );
            //     }
            // }
            
            if( result ) memcpy( result, wifi_result_buffer, wifi_buffer_size );
            if( size ) memcpy( size, &wifi_buffer_size, 1 );

            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "RP_TASK_WIFI - unkown error on get results\n" );
        return false;
    }
}

void wifi_display_results( void )
{
    HAL_DBG_TRACE_PRINTF( "Number of results: %u accepted / %u raw\r\n",
                          wifi_results.nbr_results, wifi_results.raw_results );
    HAL_DBG_TRACE_PRINTF( "WiFi scan filtered: mobile=%u, local_admin=%u, whitelist=%u, duplicate=%u, unknown_origin=%u\r\n",
                          wifi_results.mobile_ap_results, wifi_results.local_admin_results,
                          wifi_results.whitelisted_results, wifi_results.duplicate_results,
                          wifi_results.unknown_origin_results );
    HAL_DBG_TRACE_PRINTF( "WiFi scan power: %lu uAh (%lu nAh)\r\n",
                          wifi_results.power_consumption_uah, wifi_results.power_consumption_nah );
    HAL_DBG_TRACE_PRINTF( "WiFi scan timings: detect=%lu us, correlate=%lu us, capture=%lu us, demod=%lu us\r\n",
                          wifi_results.rx_detection_us, wifi_results.rx_correlation_us,
                          wifi_results.rx_capture_us, wifi_results.demodulation_us );
    for( uint8_t i = 0; i < wifi_results.nbr_results; i++ )
    {
        for( uint8_t j = 0; j < WIFI_AP_ADDRESS_SIZE; j++ )
        {
            HAL_DBG_TRACE_PRINTF( "%02X ", wifi_results.results[i].mac_address[j] );
        }
        HAL_DBG_TRACE_PRINTF( "Channel: %d, ", wifi_results.results[i].channel );
        HAL_DBG_TRACE_PRINTF( "Type: %d, ", wifi_results.results[i].type );
        HAL_DBG_TRACE_PRINTF( "RSSI: %d, ", wifi_results.results[i].rssi );
        HAL_DBG_TRACE_PRINTF( "Origin: %u, ", wifi_results.results[i].origin );
        HAL_DBG_TRACE_PRINTF( "RSSI valid: %u\r\n", wifi_results.results[i].rssi_validity ? 1 : 0 );
    }
    HAL_DBG_TRACE_PRINTF( "\n" );
}

bool wifi_scan_update_adaptive_state( void )
{
    bool filtered_only = ( wifi_results.nbr_results == 0 ) &&
                         ( ( wifi_results.mobile_ap_results > 0 ) || ( wifi_results.local_admin_results > 0 ) );
    bool provisional_accept = false;

    if( wifi_results.nbr_results > 0 )
    {
        if( ( wifi_results.results[0].channel >= 1 ) && ( wifi_results.results[0].channel <= 13 ) )
        {
            wifi_scan_last_good_channel = wifi_results.results[0].channel;
        }
        wifi_scan_next_max_results = WIFI_SCAN_TARGET_RESULTS;
    }
    else if( filtered_only )
    {
        if( wifi_scan_current_max_results < WIFI_SCAN_ADAPTIVE_MAX_RESULTS )
        {
            provisional_accept = true;
            wifi_scan_next_max_results = WIFI_SCAN_ADAPTIVE_MAX_RESULTS;
        }
        else
        {
            wifi_scan_next_max_results = WIFI_SCAN_ADAPTIVE_MAX_RESULTS;
        }
    }
    else
    {
        wifi_scan_next_max_results = WIFI_SCAN_TARGET_RESULTS;
    }

    HAL_DBG_TRACE_PRINTF( "WiFi adaptive: current_max=%u, next_max=%u, provisional=%u, last_good=%u\r\n",
                          wifi_scan_current_max_results, wifi_scan_next_max_results,
                          provisional_accept ? 1 : 0, wifi_scan_last_good_channel );

    return provisional_accept;
}

uint8_t wifi_scan_get_next_max_results( void )
{
    return wifi_scan_next_max_results;
}

void wifi_scan_stop( ralf_t* modem_radio )
{
    lr11xx_system_sleep_cfg_t radio_sleep_cfg;
    radio_sleep_cfg.is_warm_start  = true;
    radio_sleep_cfg.is_rtc_timeout = false;
    HAL_DBG_TRACE_PRINTF( "WiFi scan STOP -> radio SLEEP\r\n" );
    if( lr11xx_system_cfg_lfclk( modem_radio->ral.context, LR11XX_SYSTEM_LFCLK_RC, true ) != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to set LF clock\n" );
    }
    if( lr11xx_system_set_sleep( modem_radio->ral.context, radio_sleep_cfg, 0 ) != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to set the radio to sleep\n" );
    }
}
