/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main_lorawan_tracker.h"
#include "lorawan_key_config.h"
#include "smtc_board.h"
#include "smtc_hal.h"
#include "apps_modem_common.h"
#include "apps_modem_event.h"
#include "smtc_modem_api.h"
#include "smtc_modem_middleware_advanced_api.h"
#include "smtc_modem_hal.h"
#include "device_management_defs.h"
#include "smtc_board_ralf.h"
#include "apps_utilities.h"
#include "smtc_modem_utilities.h"
#include "lr11xx_system.h"
#include "remex_abp_derive.h"

#include "app_board.h"
#include "app_ble_all.h"
#include "app_config_param.h"
#include "app_at_fds_datas.h"
#include "app_at_command.h"
#include "app_lora_packet.h"
#include "app_user_timer.h"
#include "app_button.h"
#include "app_led.h"
#include "app_beep.h"
#include "app_timer.h"
#include "crew_dr_strategy_config.h"
#include "main_lorawan_tracker_api.h"
#include "wifi_scan.h"
#include "gateway_assistance.h"
#include "marine_gnss.h"
#include "crew_payload_schema.h"
#include "ag3335.h"
#include "firmware_version.h"
#include "log_filter.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define CREW_DR_PROFILE_LEN         16
#define CREW_HP_CHANNEL_ENABLE      false
#define CREW_HP_CHANNEL_FREQ_HZ     869525000
#define CREW_HP_CHANNEL_DR          3
#define CREW_BLE_UPLINK_RECORD_LEN  5
#define CREW_SOS_CONTEXT_SUBTYPE    0x23
#define CREW_SOS_CONTEXT_SCHEMA     0x10
#define STARTUP_SERIAL_DELAY_MS     5000

/* Fleet de-synchronisation windows. All are deterministic per DevEUI, so a tag keeps
 * the same phase across reboots while the fleet is spread across the available window.
 * Alert/MOB traffic bypasses these delays; they apply only to routine/control traffic. */
#define CREW_STARTUP_FIRST_UPLINK_JITTER_S      120
#define CREW_STABLE_LINKCHECK_PHASE_WINDOW_S    120
#define CREW_LINKCHECK_RETRY_FIRST_DELAY_S      30
#define CREW_LINKCHECK_RETRY_FIRST_JITTER_S     30
#define CREW_LINKCHECK_RETRY_UPLINK_INTERVAL    10

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    uint8_t region_min;
    uint8_t region_max;
    uint8_t normal;
    uint8_t persistence;
    uint8_t minimum;
    uint8_t sos_low;
    uint8_t vessel_current;
} crew_dr_config_t;

typedef enum
{
    CREW_DR_PROFILE_NONE = 0,
    CREW_DR_PROFILE_MIN = 1,
    CREW_DR_PROFILE_MAX = 5,
} crew_dr_beacon_profile_t;

typedef struct
{
    bool active;
    uint8_t profile;
    uint8_t pending_upgrade_profile;
    uint8_t pending_upgrade_count;
    uint8_t pending_movement_profile;
    uint8_t pending_movement_count;
    uint16_t pending_movement_major;
    uint16_t pending_movement_minor;
    int8_t pending_movement_rssi;
    uint8_t pending_movement_uuid[16];
    uint8_t pending_movement_mac[8];
    uint16_t major;
    uint16_t minor;
    int8_t rssi;
    uint8_t uuid[16];
    uint8_t mac[8];
    uint32_t stable_since_s;
    uint32_t last_forced_linkcheck_s;
    bool linkcheck_sent_for_profile;
    bool linkcheck_refined_dr;
} crew_ble_hint_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * Override the default LoRaWAN RX1 delay (seconds).
 * If non-zero, the LoRaWAN stack will use this value as the initial RX1 delay
 * instead of its regional default. The network may still update it later via
 * RxTimingSetupReq.
 */
uint8_t g_rx1_delay_override_s = 4; // set RX1 delay to 4 seconds

/*!
 * @brief Stack identifier
 */
static uint8_t stack_id = 0;

/*!
 * @brief Modem radio
 */
static ralf_t* modem_radio;

/*!
 * @brief User application data
 */
static uint8_t app_data_buffer[LORAWAN_APP_DATA_MAX_SIZE];
static uint8_t app_data_len = 0;

static uint8_t adr_custom_list_eu868_default[16] = { 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static uint8_t adr_custom_list_us915_default[16] = { 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static uint8_t adr_custom_list_au915_default[16] = { 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static uint8_t adr_custom_list_as923_default[16] = { 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static uint8_t adr_custom_list_kr920_default[16] = { 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static uint8_t adr_custom_list_in865_default[16] = { 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static uint8_t adr_custom_list_ru864_default[16] = { 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7

static uint8_t tracker_scan_status = 0;
static uint32_t tracker_scan_begin = 0;
static uint32_t last_time_sync_s = 0;  // Track last DeviceTimeReq for periodic resync
static bool device_time_req_pending = false;
static bool sos_gnss_prestarted = false;
static bool app_gnss_initialized = false;
static bool gnss_proof_scan_active = false;
static bool gnss_proof_pending = false;
static bool gnss_proof_quality_ok = false;
static gnss_fix_t gnss_proof_fix = { 0 };

uint8_t tracker_scan_type = 0;

uint32_t gnss_scan_duration = 30;            // in second
uint32_t wifi_scan_duration = 1;            // in second
uint32_t ble_scan_duration = 3;             // in second
uint32_t tracker_periodic_interval = 60;    // in second

uint8_t wifi_scan_max = 3;
uint8_t ble_scan_max = 3;

uint8_t tracker_acc_en = 0;

bool adr_user_enable = true;
uint8_t adr_user_dr_min = 0;
uint8_t adr_user_dr_max = 0;
uint8_t adr_custom_list_region[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static crew_dr_config_t crew_dr = { 0 };
static bool crew_dr_ready = false;
static uint8_t crew_linkcheck_good_streak = 0;
static uint16_t crew_uplinks_since_linkcheck = 0;
static uint32_t crew_vessel_uplink_count = 0;
static bool crew_linkcheck_pending = false;
static crew_ble_hint_state_t crew_ble_hint = { 0 };
static bool crew_linkcheck_retry_active = false;
static uint8_t crew_linkcheck_retry_probe_dr = 0;
static uint8_t crew_linkcheck_retry_miss_count = 0;
static uint32_t crew_linkcheck_retry_not_before_s = 0;
static uint32_t crew_linkcheck_retry_after_uplink = 0;
static uint32_t mob_phase3_uplink_count = 0;
/* Extended uplink tasks keep only a payload pointer, so burst payloads need stable storage until the modem runs them. */
static uint8_t crew_burst_ext_payload[2][LORAWAN_APP_DATA_MAX_SIZE];
static uint8_t crew_burst_ext_len[2] = { 0, 0 };

uint8_t packet_policy = RETRY_STATE_1N;

bool duty_cycle_enable = true;

uint8_t tracker_test_mode = 0;

uint8_t tracker_gps_scan_len = 0;
uint8_t tracker_gps_scan_data[64] = { 0 };

uint8_t tracker_wifi_scan_len = 0;
uint8_t tracker_wifi_scan_data[64] = { 0 };

uint8_t tracker_ble_scan_len = 0;
uint8_t tracker_ble_scan_data[64] = { 0 };

uint8_t tracker_scan_temp_len = 0;
uint8_t tracker_scan_data_temp[64] = { 0 };

bool scan_result = false;
int8_t scan_result_num = 0;

uint8_t event_state = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief   Send an application frame on LoRaWAN port defined by LORAWAN_APP_PORT
 *
 * @param [in] buffer     Buffer containing the LoRaWAN buffer
 * @param [in] length     Payload length
 * @param [in] confirmed  Send a confirmed or unconfirmed uplink [false : unconfirmed / true : confirmed]
 */
static void send_frame( const uint8_t* buffer, const uint8_t length, const bool confirmed );

/*!
 * @brief Parse the received downlink
 *
 * @remark Demonstrates how a TLV-encoded command sequence received by downlink can control the state of an LED. It can
 * easily be extended to handle other commands received on the same port or another port.
 *
 * @param [in] port LoRaWAN port
 * @param [in] payload Payload Buffer
 * @param [in] size Payload size
 */
static void parse_downlink_frame( uint8_t port, const uint8_t* payload, uint8_t size );

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void on_modem_reset( uint16_t reset_count );

/*!
 * @brief Network Joined event callback
 */
static void on_modem_network_joined( void );

/*!
 * @brief Alarm event callback
 */
static void on_modem_alarm( void );

/*!
 * @brief Tx done event callback
 *
 * @param [in] status tx done status @ref smtc_modem_event_txdone_status_t
 */
static void on_modem_tx_done( smtc_modem_event_txdone_status_t status );

/*!
 * @brief Time sync event callback
 *
 * @param [in] status Time sync status
 */
static void on_modem_time_updated( smtc_modem_event_time_status_t status );

/*!
 * @brief Downlink data event callback.
 *
 * @param [in] rssi       RSSI in signed value in dBm + 64
 * @param [in] snr        SNR signed value in 0.25 dB steps
 * @param [in] rx_window  RX window
 * @param [in] port       LoRaWAN port
 * @param [in] payload    Received buffer pointer
 * @param [in] size       Received buffer size
 */
static void on_modem_down_data( int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                                const uint8_t* payload, uint8_t size );

/*!
 * @brief 
 */
static void app_tracker_scan_process( void );
static bool app_send_frame_on_port( uint8_t port, const uint8_t* buffer, const uint8_t length, bool tx_confirmed,
                                    bool emergency );
static bool app_send_frame_on_port_ext( uint8_t port, const uint8_t* buffer, const uint8_t length, bool tx_confirmed,
                                        bool emergency, uint8_t extended_id );
static bool crew_send_dr_burst_on_port( uint8_t port, const uint8_t* buffer, const uint8_t length, bool tx_confirmed,
                                        bool emergency );
static void crew_dr_configure_for_region( smtc_modem_region_t region );
static bool crew_dr_apply_fixed( uint8_t dr );
static bool crew_dr_prepare_next_uplink( uint8_t dr );
static uint8_t crew_dr_for_mob_policy( app_mob_dr_policy_t policy );
static uint8_t crew_dr_for_ble_profile( uint8_t profile );
static void crew_dr_handle_ble_hint_scan( void );
static void crew_dr_handle_link_status( smtc_modem_event_link_check_status_t status, uint8_t margin, uint8_t gw_cnt );
static void crew_dr_after_vessel_uplink( bool send_ok );
static bool crew_dr_request_linkcheck( const char* reason );
static bool crew_dr_request_linkcheck_at_dr( const char* reason, uint8_t probe_dr );
static void crew_linkcheck_retry_reset( void );
static void crew_linkcheck_retry_schedule_missing_ans( void );
static bool crew_linkcheck_retry_due( uint32_t now_s );
static uint8_t app_tracker_live_event_state_get( void );
static uint32_t crew_dev_eui_hash( uint32_t purpose );
static uint32_t crew_dev_eui_jitter_s( uint32_t purpose, uint32_t window_s );
static uint16_t crew_dev_eui_phase_uplinks( uint32_t purpose, uint16_t interval );
static void crew_wait_startup_first_uplink_jitter( void );
static void crew_extended_uplink_done( void );
static void remex_prepare_abp_config_for_ble_app( void );
static bool app_tracker_is_sos_event( void );
static bool app_tracker_payload_is_sos_context( const uint8_t* buffer, uint8_t length );
static bool app_tracker_send_compact_presence( uint8_t outgoing_event_state, int8_t battery, bool confirm );
static bool app_tracker_send_gnss_proof( uint8_t outgoing_event_state, int8_t battery, bool confirm );
static bool app_tracker_send_sos_context( uint8_t outgoing_event_state, int8_t battery, bool confirm );
static void app_tracker_sos_gnss_prestart( void );
static uint32_t app_tracker_gnss_next_delay( void );
static void app_tracker_u16_le( uint8_t* buffer, uint16_t value );
static void app_tracker_i32_le( uint8_t* buffer, int32_t value );

/*!
 * @}
 */

/*
 * ----------------------------------------------------------------------------- 
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    static apps_modem_event_callback_t smtc_event_callback = {
        .adr_mobile_to_static  = NULL,
        .alarm                 = on_modem_alarm,
        .almanac_update        = NULL,
        .down_data             = on_modem_down_data,
        .join_fail             = NULL,
        .joined                = on_modem_network_joined,
        .link_status           = crew_dr_handle_link_status,
        .mute                  = NULL,
        .new_link_adr          = NULL,
        .reset                 = on_modem_reset,
        .set_conf              = NULL,
        .stream_done           = NULL,
        .time_updated_alc_sync = on_modem_time_updated,
        .tx_done               = on_modem_tx_done,
        .upload_done           = NULL,
    };

    /* Initialise the ralf_t object corresponding to the board */
    modem_radio = smtc_board_initialise_and_get_ralf( );

    /* Init board and peripherals */
    hal_mcu_init( );
    fds_init_write( );
    smtc_board_init_periph( );
    remex_prepare_abp_config_for_ble_app( );
    app_lora_packet_params_load( );

    ret_code_t err_code;
    err_code = app_timer_init( );
    APP_ERROR_CHECK( err_code );

    hal_mcu_wait_ms( 500 ); // wait for stable

    app_user_timers_init( );
    app_user_button_init( );
    app_ble_all_init( );
    app_led_init( );
    app_beep_init( );
    
    /* Initialize gateway assistance system */
    gateway_assistance_init( );
    
    /* Initialize marine GNSS tracking */
    mob_tracker_init( );
    
    HAL_DBG_TRACE_INFO( "Waiting 5s for serial connection...\n" );
    hal_mcu_wait_ms( STARTUP_SERIAL_DELAY_MS );

    /* Print firmware version */
    HAL_DBG_TRACE_INFO( "========================================\n" );
    HAL_DBG_TRACE_INFO( "T1000-E Tracker Firmware v%s\n", FIRMWARE_VERSION_STRING );
    HAL_DBG_TRACE_INFO( "Features: %s\n", FIRMWARE_VERSION_FEATURES );
    HAL_DBG_TRACE_INFO( "========================================\n" );

    app_beep_boot_up( );

    if( tracker_scan_type == TRACKER_SCAN_GNSS_ONLY 
        || tracker_scan_type == TRACKER_SCAN_WIFI_GNSS 
        || tracker_scan_type == TRACKER_SCAN_GNSS_WIFI 
        || tracker_scan_type == TRACKER_SCAN_BLE_GNSS 
        || tracker_scan_type == TRACKER_SCAN_BLE_WIFI_GNSS )
    {
        gnss_init( );
        app_gnss_initialized = true;
        gnss_scan_start( );
        hal_mcu_wait_ms( 3000 );
        gnss_scan_stop( );
    }

    if( tracker_acc_en )
    {
        hal_gpio_init_out( ACC_POWER, HAL_GPIO_SET );
        qma6100p_init( );
    }

APP_MAIN:
    /* Init the Lora Basics Modem event callbacks */
    apps_modem_event_init( &smtc_event_callback );

    /* Init the modem and use smtc_event_process as event callback, please note that the callback will be called
     * immediately after the first call to modem_run_engine because of the reset detection */
    smtc_modem_init( modem_radio, &apps_modem_event_process );

    HAL_DBG_TRACE_MSG( "\n" );
    HAL_DBG_TRACE_INFO( "###### ===== T1000-E Tracker %s (%s %s) ==== ######\n\n", 
                        FIRMWARE_VERSION_STRING, __DATE__, __TIME__ );

    /* LoRa Basics Modem Version */
    apps_modem_common_display_lbm_version( );

    /* Configure the partial low power mode */
    hal_mcu_partial_sleep_enable( APP_PARTIAL_SLEEP );

    while( 1 )
    {
        /* Execute modem runtime, this function must be called again in sleep_time_ms milliseconds or sooner. */
        uint32_t sleep_time_ms = smtc_modem_run_engine( );
        /* go in low power */
        hal_mcu_set_sleep_for_ms( sleep_time_ms );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool remex_buffer_is_all_zero( const uint8_t* buffer, uint8_t len )
{
    for( uint8_t i = 0; i < len; i++ )
    {
        if( buffer[i] != 0 )
        {
            return false;
        }
    }
    return true;
}

static bool remex_deveui_is_lr11xx_uid( const uint8_t dev_eui[8] )
{
    return ( dev_eui[0] == 0x00 ) && ( dev_eui[1] == 0x16 ) && ( dev_eui[2] == 0xC0 );
}

static void remex_prepare_abp_config_for_ble_app( void )
{
    uint8_t dev_eui[8]    = { 0 };
    uint8_t nwk_s_key[16] = { 0 };
    uint8_t app_s_key[16] = { 0 };
    uint32_t dev_addr     = 0;
    bool config_changed   = false;

    /*
     * The BLE config app can read app_param before the LoRa Basics Modem reset
     * callback runs. Use only the immutable LoRaWAN DevEUI already stored in
     * app_param by factory provisioning. The LR11xx UID is a hardware identifier,
     * not the SenseCAP LoRaWAN DevEUI, and must never be copied into DevEUI.
     */
    memcpy( dev_eui, app_param.lora_info.DevEui, sizeof( dev_eui ) );

    if( remex_deveui_is_lr11xx_uid( dev_eui ) )
    {
        uint8_t chip_uid[8] = { 0 };
        HAL_DBG_TRACE_ERROR( "Stored DevEUI looks like LR11xx UID; refusing to derive ABP keys\n" );
        if( lr11xx_system_read_uid( modem_radio->ral.context, chip_uid ) == LR11XX_STATUS_OK )
        {
            HAL_DBG_TRACE_ARRAY( "LR11xx UID rejected", chip_uid, sizeof( chip_uid ) );
        }
        memset( dev_eui, 0, sizeof( dev_eui ) );
    }

    if( app_param.lora_info.ActivationType != ACTIVATION_TYPE_ABP )
    {
        return;
    }

    if( remex_buffer_is_all_zero( dev_eui, sizeof( dev_eui ) ) ||
        remex_abp_derive_session( dev_eui, &dev_addr, nwk_s_key, app_s_key ) == false )
    {
        HAL_DBG_TRACE_ERROR( "Failed to prepare RemEX ABP defaults for config app\n" );
        return;
    }

    if( app_param.lora_info.DevAddr != dev_addr )
    {
        app_param.lora_info.DevAddr = dev_addr;
        config_changed = true;
    }

    if( memcmp( app_param.lora_info.NwkSKey, nwk_s_key, sizeof( app_param.lora_info.NwkSKey ) ) != 0 )
    {
        memcpy( app_param.lora_info.NwkSKey, nwk_s_key, sizeof( app_param.lora_info.NwkSKey ) );
        config_changed = true;
    }

    if( memcmp( app_param.lora_info.AppSKey, app_s_key, sizeof( app_param.lora_info.AppSKey ) ) != 0 )
    {
        memcpy( app_param.lora_info.AppSKey, app_s_key, sizeof( app_param.lora_info.AppSKey ) );
        config_changed = true;
    }

    HAL_DBG_TRACE_INFO( "RemEX ABP defaults prepared for config app:\n" );
    HAL_DBG_TRACE_ARRAY( "DevEUI", dev_eui, sizeof( dev_eui ) );
    HAL_DBG_TRACE_INFO( "DevAddr: %02X%02X%02X%02X\n",
                        ( unsigned )( ( dev_addr >> 24 ) & 0xFF ),
                        ( unsigned )( ( dev_addr >> 16 ) & 0xFF ),
                        ( unsigned )( ( dev_addr >> 8 ) & 0xFF ),
                        ( unsigned )( dev_addr & 0xFF ) );
    HAL_DBG_TRACE_ARRAY( "NwkSKey", nwk_s_key, sizeof( nwk_s_key ) );
    HAL_DBG_TRACE_ARRAY( "AppSKey", app_s_key, sizeof( app_s_key ) );

    if( config_changed && !write_current_param_config( ) )
    {
        HAL_DBG_TRACE_ERROR( "Failed to persist RemEX ABP defaults for config app\n" );
    }
}

/*!
 * @brief LoRa Basics Modem event callbacks called by smtc_event_process function
 */

static void on_modem_reset( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "Application parameters:\n" );
    HAL_DBG_TRACE_INFO( "  - LoRaWAN uplink Fport = %d\n", LORAWAN_APP_PORT );
    HAL_DBG_TRACE_INFO( "  - Confirmed uplink     = %s\n", ( LORAWAN_CONFIRMED_MSG_ON == true ) ? "Yes" : "No" );

    apps_modem_common_configure_lorawan_params( stack_id );

    uint8_t ativation_mode;
    ativation_mode = smtc_modem_get_activation_mode( stack_id );
    if( ativation_mode == 0 ) // OTAA
    {
        app_led_breathe_start( );
        ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( stack_id ) );
    }
    else // ABP, manual run joined init
    {
        on_modem_network_joined( );
    }
}

static void custom_lora_adr_compute( uint8_t min, uint8_t max, uint8_t *buf )
{
    uint8_t temp = max - min + 1;
    uint8_t num = 16 / temp;
    uint8_t remain = 16 % temp;
    uint8_t offset = 0;

    for( uint8_t i = 0; i < temp; i++ )
    {
        for( uint8_t j = 0; j < num; j++ )
        {
            buf[i * num + j + offset] = min + i;
        }

        if(( 16 % temp != 0 ) && ( i < remain ))
        {
            buf[( i + 1 ) * num + offset] = min + i;
            offset += 1;
        }
    }
}

static bool crew_get_region_dr_bounds( smtc_modem_region_t region, uint8_t *min_dr, uint8_t *max_dr )
{
    switch( region )
    {
        case SMTC_MODEM_REGION_EU_868:
            *min_dr = LORAWAN_EU868_DR_MIN;
            *max_dr = LORAWAN_EU868_DR_MAX;
        break;

        case SMTC_MODEM_REGION_US_915:
            *min_dr = LORAWAN_US915_DR_MIN;
            *max_dr = LORAWAN_US915_DR_MAX;
        break;

        case SMTC_MODEM_REGION_AU_915:
            *min_dr = LORAWAN_AU915_DR_MIN;
            *max_dr = LORAWAN_AU915_DR_MAX;
        break;

        case SMTC_MODEM_REGION_AS_923_GRP1:
        case SMTC_MODEM_REGION_AS_923_GRP2:
        case SMTC_MODEM_REGION_AS_923_GRP3:
        case SMTC_MODEM_REGION_AS_923_GRP4:
        case SMTC_MODEM_REGION_AS_923_HELIUM_1:
        case SMTC_MODEM_REGION_AS_923_HELIUM_2:
        case SMTC_MODEM_REGION_AS_923_HELIUM_3:
        case SMTC_MODEM_REGION_AS_923_HELIUM_4:
        case SMTC_MODEM_REGION_AS_923_HELIUM_1B:
            *min_dr = LORAWAN_AS923_DR_MIN;
            *max_dr = LORAWAN_AS923_DR_MAX;
        break;

        case SMTC_MODEM_REGION_KR_920:
            *min_dr = LORAWAN_KR920_DR_MIN;
            *max_dr = LORAWAN_KR920_DR_MAX;
        break;

        case SMTC_MODEM_REGION_IN_865:
            *min_dr = LORAWAN_IN865_DR_MIN;
            *max_dr = LORAWAN_IN865_DR_MAX;
        break;

        case SMTC_MODEM_REGION_RU_864:
            *min_dr = LORAWAN_RU864_DR_MIN;
            *max_dr = LORAWAN_RU864_DR_MAX;
        break;

        default:
            return false;
    }

    return true;
}

static uint8_t crew_clamp_dr( uint8_t dr, uint8_t min_dr, uint8_t max_dr )
{
    if( dr < min_dr )
    {
        return min_dr;
    }
    if( dr > max_dr )
    {
        return max_dr;
    }
    return dr;
}

static void crew_dr_configure_for_region( smtc_modem_region_t region )
{
    uint8_t region_min = 0;
    uint8_t region_max = 0;

    if( crew_get_region_dr_bounds( region, &region_min, &region_max ) == false )
    {
        HAL_DBG_TRACE_WARNING( "Crew DR strategy disabled: unsupported region\n" );
        crew_dr_ready = false;
        return;
    }

    bool user_range_valid = ( adr_user_dr_min >= region_min ) && ( adr_user_dr_max <= region_max ) &&
                            ( adr_user_dr_min <= adr_user_dr_max );

    /*
     * Crew tags run with a mission-oriented DR policy instead of ADR.
     * All selected DRs are clamped to the configured lr_DR_min/lr_DR_max window per MDR-012.
     * If the BT app has not provided a valid lr_DR_min/lr_DR_max window, use the region bounds.
     */
    crew_dr.region_min = region_min;
    crew_dr.region_max = region_max;
    crew_dr.minimum = user_range_valid ? adr_user_dr_min : region_min;
    crew_dr.normal = user_range_valid ? adr_user_dr_max : region_max;
    crew_dr.persistence = ( crew_dr.normal > 2 ) ? ( crew_dr.normal - 2 ) : crew_dr.minimum;
    crew_dr.persistence = crew_clamp_dr( crew_dr.persistence, crew_dr.minimum, crew_dr.normal );
    crew_dr.sos_low = crew_dr.minimum;
    crew_dr.vessel_current = crew_dr.normal;
    crew_linkcheck_good_streak = 0;
    crew_uplinks_since_linkcheck = 0;
    crew_linkcheck_pending = false;
    memset( &crew_ble_hint, 0, sizeof( crew_ble_hint ));
    mob_phase3_uplink_count = 0;
    crew_dr_ready = true;

    LOG_LORA( "Crew DR strategy: minimum=%u normal=%u persistence=%u sos_low=%u\n",
              crew_dr.minimum, crew_dr.normal, crew_dr.persistence, crew_dr.sos_low );
}

static bool crew_dr_apply_fixed( uint8_t dr )
{
    uint8_t fixed_dr[CREW_DR_PROFILE_LEN];

    if( crew_dr_ready == false )
    {
        return false;
    }

    dr = crew_clamp_dr( dr, crew_dr.minimum, crew_dr.normal );
    memset( fixed_dr, dr, sizeof( fixed_dr ));

    if( smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, fixed_dr ) != SMTC_MODEM_RC_OK )
    {
        LOG_LORA( "WARN: Failed to apply fixed DR%u\n", dr );
        return false;
    }

    return true;
}

static bool crew_dr_prepare_next_uplink( uint8_t dr )
{
    uint8_t applied_dr = crew_clamp_dr( dr, crew_dr.minimum, crew_dr.normal );

    /*
     * The ADR profile sets the live modem DR. The per-task tag below preserves that DR
     * for queued SEND_TASK_EXTENDED packets, otherwise later burst entries could overwrite it.
     */
    if( crew_dr_apply_fixed( applied_dr ) == false )
    {
        return false;
    }

    if( smtc_modem_set_next_uplink_datarate( stack_id, applied_dr ) != SMTC_MODEM_RC_OK )
    {
        LOG_LORA( "WARN: Failed to tag next uplink for DR%u\n", applied_dr );
        return false;
    }

    return true;
}

static uint32_t crew_dev_eui_hash( uint32_t purpose )
{
    uint8_t dev_eui[SMTC_MODEM_EUI_LENGTH] = { 0 };
    uint32_t hash = 2166136261u ^ purpose;

    if( smtc_modem_get_deveui( stack_id, dev_eui ) != SMTC_MODEM_RC_OK )
    {
        /*
         * If DevEUI is not readable yet, still return a deterministic value for this build.
         * This path should be rare after join/configuration, but avoids falling back to zero
         * and re-synchronising the fleet.
         */
        memcpy( dev_eui, "REMXJITR", SMTC_MODEM_EUI_LENGTH );
    }

    for( uint8_t i = 0; i < SMTC_MODEM_EUI_LENGTH; i++ )
    {
        hash ^= dev_eui[i];
        hash *= 16777619u;
    }

    hash ^= purpose >> 16;
    hash *= 16777619u;
    return hash;
}

static uint32_t crew_dev_eui_jitter_s( uint32_t purpose, uint32_t window_s )
{
    return ( window_s == 0 ) ? 0 : ( crew_dev_eui_hash( purpose ) % ( window_s + 1 ));
}

static uint16_t crew_dev_eui_phase_uplinks( uint32_t purpose, uint16_t interval )
{
    return ( interval == 0 ) ? 0 : ( uint16_t )( crew_dev_eui_hash( purpose ) % interval );
}

static void crew_wait_startup_first_uplink_jitter( void )
{
    uint32_t jitter_s = crew_dev_eui_jitter_s( 0x53544152u, CREW_STARTUP_FIRST_UPLINK_JITTER_S ); /* "STAR" */

    if( jitter_s == 0 )
    {
        LOG_LORA( "Crew startup jitter: 0 s (DevEUI phase, first routine uplink)\n" );
        return;
    }

    /*
     * Startup jitter handles the charger-bank case: many flat tags can wake and join together when
     * power is restored. Spread only the first routine status uplink; button/MOB alert paths do not
     * pass through this wait and must stay immediate.
     */
    LOG_LORA( "Crew startup jitter: wait %lu s before first routine status uplink (DevEUI phase)\n", jitter_s );
    hal_mcu_wait_ms( jitter_s * 1000 );
}

static uint8_t crew_dr_for_mob_policy( app_mob_dr_policy_t policy )
{
    if( crew_dr_ready == false )
    {
        return 0;
    }

    switch( policy )
    {
        case APP_MOB_DR_MAX:
            return crew_dr.normal;

        case APP_MOB_DR_MINIMUM:
            return crew_dr.sos_low;

        case APP_MOB_DR_PHASE3_ALTERNATING:
            /* Phase 3 keeps 90% of traffic at persistence DR and probes minimum DR every 10th uplink. */
            mob_phase3_uplink_count++;
            return ( ( mob_phase3_uplink_count % 10 ) == 0 ) ? crew_dr.sos_low : crew_dr.persistence;

        case APP_MOB_DR_PERSISTENCE:
        default:
            return crew_dr.persistence;
    }
}

static uint8_t crew_dr_for_ble_profile( uint8_t profile )
{
    if(( profile >= CREW_DR_PROFILE_MIN ) && ( profile <= CREW_DR_PROFILE_MAX ))
    {
        return crew_clamp_dr( profile, crew_dr.minimum, crew_dr.normal );
    }

    return crew_dr.vessel_current;
}

static bool crew_dr_accept_ble_profile( const ble_beacon_hint_t *hint, uint8_t profile )
{
    if( crew_dr_ready == false )
    {
        return false;
    }

    uint8_t new_dr = crew_dr_for_ble_profile( profile );
    bool beacon_changed = ( crew_ble_hint.active == false ) || ( crew_ble_hint.profile != profile ) ||
                          ( crew_ble_hint.major != hint->major ) || ( crew_ble_hint.minor != hint->minor ) ||
                          ( memcmp( crew_ble_hint.uuid, hint->uuid, sizeof( crew_ble_hint.uuid )) != 0 ) ||
                          ( memcmp( crew_ble_hint.mac, hint->mac, sizeof( crew_ble_hint.mac )) != 0 );

    crew_ble_hint.active = true;
    crew_ble_hint.profile = profile;
    crew_ble_hint.pending_upgrade_profile = CREW_DR_PROFILE_NONE;
    crew_ble_hint.pending_upgrade_count = 0;
    crew_ble_hint.pending_movement_profile = CREW_DR_PROFILE_NONE;
    crew_ble_hint.pending_movement_count = 0;
    crew_ble_hint.major = hint->major;
    crew_ble_hint.minor = hint->minor;
    crew_ble_hint.rssi = hint->rssi;
    memcpy( crew_ble_hint.uuid, hint->uuid, sizeof( crew_ble_hint.uuid ));
    memcpy( crew_ble_hint.mac, hint->mac, sizeof( crew_ble_hint.mac ));

    if( beacon_changed )
    {
        crew_ble_hint.stable_since_s = hal_rtc_get_time_s( );
        crew_ble_hint.linkcheck_sent_for_profile = false;
        crew_ble_hint.linkcheck_refined_dr = false;
        crew_linkcheck_good_streak = 0;
        crew_linkcheck_retry_reset( );
    }

    /*
     * BLE hints set the vessel baseline quickly when the wearer enters a new RF zone.
     * LinkCheck may later fine tune the baseline, but the local hint is the immediate movement signal.
     */
    if( beacon_changed || (( crew_ble_hint.linkcheck_refined_dr == false ) && ( new_dr != crew_dr.vessel_current )))
    {
        crew_dr.vessel_current = new_dr;
        crew_dr_apply_fixed( crew_dr.vessel_current );
        LOG_LORA( "Crew BLE hint accepted: major=%u minor=%u rssi=%d DR%u\n",
                  hint->major, hint->minor, hint->rssi, crew_dr.vessel_current );
    }
    else if(( crew_ble_hint.linkcheck_refined_dr ) && ( new_dr < crew_dr.vessel_current ))
    {
        LOG_LORA( "Crew BLE hint held: major=%u minor=%u suggests DR%u, LinkCheck keeps DR%u\n",
                  hint->major, hint->minor, new_dr, crew_dr.vessel_current );
    }

    return true;
}

static bool crew_ble_hint_matches_current( const ble_beacon_hint_t *hint, uint8_t profile )
{
    return ( crew_ble_hint.active ) && ( crew_ble_hint.profile == profile ) &&
           ( crew_ble_hint.major == hint->major ) && ( crew_ble_hint.minor == hint->minor ) &&
           ( memcmp( crew_ble_hint.uuid, hint->uuid, sizeof( crew_ble_hint.uuid )) == 0 ) &&
           ( memcmp( crew_ble_hint.mac, hint->mac, sizeof( crew_ble_hint.mac )) == 0 );
}

static bool crew_ble_hint_minor_is_dr_profile( uint16_t minor )
{
    return ( minor >= CREW_DR_PROFILE_MIN ) && ( minor <= CREW_DR_PROFILE_MAX );
}

static bool crew_ble_hint_confirm_movement( const ble_beacon_hint_t *hint, uint8_t profile )
{
    bool same_pending = ( crew_ble_hint.pending_movement_profile == profile ) &&
                        ( crew_ble_hint.pending_movement_major == hint->major ) &&
                        ( crew_ble_hint.pending_movement_minor == hint->minor ) &&
                        ( memcmp( crew_ble_hint.pending_movement_uuid, hint->uuid,
                                  sizeof( crew_ble_hint.pending_movement_uuid )) == 0 ) &&
                        ( memcmp( crew_ble_hint.pending_movement_mac, hint->mac,
                                  sizeof( crew_ble_hint.pending_movement_mac )) == 0 );

    /*
     * The T1000-E only samples BLE once before each uplink, so movement hysteresis is based on
     * repeated scan sessions, not advertisement counts inside one scan. A one-off strongest-beacon
     * flip is treated as RSSI noise and must repeat before it can reset the BLE DR baseline.
     */
    if( same_pending )
    {
        if( crew_ble_hint.pending_movement_count < CREW_DR_BLE_HINT_MOVEMENT_CONFIRM_SCANS )
        {
            crew_ble_hint.pending_movement_count++;
        }
    }
    else
    {
        crew_ble_hint.pending_movement_profile = profile;
        crew_ble_hint.pending_movement_count = 1;
        crew_ble_hint.pending_movement_major = hint->major;
        crew_ble_hint.pending_movement_minor = hint->minor;
        crew_ble_hint.pending_movement_rssi = hint->rssi;
        memcpy( crew_ble_hint.pending_movement_uuid, hint->uuid, sizeof( crew_ble_hint.pending_movement_uuid ));
        memcpy( crew_ble_hint.pending_movement_mac, hint->mac, sizeof( crew_ble_hint.pending_movement_mac ));
    }

    if( crew_ble_hint.pending_movement_count >= CREW_DR_BLE_HINT_MOVEMENT_CONFIRM_SCANS )
    {
        return true;
    }

    LOG_LORA( "Crew BLE movement pending: major=%u minor=%u rssi=%d count=%u/%u\n",
              hint->major, hint->minor, hint->rssi, crew_ble_hint.pending_movement_count,
              CREW_DR_BLE_HINT_MOVEMENT_CONFIRM_SCANS );
    return false;
}

static void crew_ble_accept_movement_only( const ble_beacon_hint_t *hint, uint8_t profile )
{
    /*
     * Approved UUIDs are useful movement/location signals even when Minor has not been commissioned
     * as a DR hint. Update stability state so LinkCheck can run after the wearer stops moving, but
     * leave the current DR unchanged until a valid Minor 1..5 hint or LinkCheckAns changes it.
     */
    crew_ble_hint.active = true;
    crew_ble_hint.profile = profile;
    crew_ble_hint.pending_upgrade_profile = CREW_DR_PROFILE_NONE;
    crew_ble_hint.pending_upgrade_count = 0;
    crew_ble_hint.pending_movement_profile = CREW_DR_PROFILE_NONE;
    crew_ble_hint.pending_movement_count = 0;
    crew_ble_hint.major = hint->major;
    crew_ble_hint.minor = hint->minor;
    crew_ble_hint.rssi = hint->rssi;
    memcpy( crew_ble_hint.uuid, hint->uuid, sizeof( crew_ble_hint.uuid ));
    memcpy( crew_ble_hint.mac, hint->mac, sizeof( crew_ble_hint.mac ));
    crew_ble_hint.stable_since_s = hal_rtc_get_time_s( );
    crew_ble_hint.linkcheck_sent_for_profile = false;
    crew_ble_hint.linkcheck_refined_dr = false;
    crew_linkcheck_good_streak = 0;

    LOG_LORA( "Crew BLE movement accepted: major=%u minor=%u rssi=%d, keep DR%u\n",
              hint->major, hint->minor, hint->rssi, crew_dr.vessel_current );
}

static void crew_dr_handle_ble_hint_scan( void )
{
#if CREW_DR_BLE_HINT_ENABLE
    ble_beacon_hint_t movement_hint;
    ble_beacon_hint_t dr_hint;

    if( crew_dr_ready == false )
    {
        return;
    }

    if( ble_get_strongest_approved_beacon( &movement_hint ) == false )
    {
        if( crew_ble_hint.active )
        {
            LOG_LORA( "Crew BLE movement hint lost; continuing at DR%u until LinkCheck updates baseline\n",
                      crew_dr.vessel_current );
        }
        memset( &crew_ble_hint, 0, sizeof( crew_ble_hint ));
        return;
    }

    bool has_dr_profile = crew_ble_hint_minor_is_dr_profile( movement_hint.minor );
    uint8_t movement_profile = has_dr_profile ? ( uint8_t ) movement_hint.minor : CREW_DR_PROFILE_NONE;

    if(( crew_ble_hint.active ) && ( crew_ble_hint_matches_current( &movement_hint, movement_profile ) == false ))
    {
        if( crew_ble_hint_confirm_movement( &movement_hint, movement_profile ) == false )
        {
            return;
        }

        if( has_dr_profile == false )
        {
            crew_ble_accept_movement_only( &movement_hint, movement_profile );
            return;
        }
    }

    if( has_dr_profile == false )
    {
        /*
         * Same approved beacon/location but no DR Minor. Keep it active for stability timing so a
         * post-movement LinkCheck can refine DR, without changing DR directly from BLE.
         */
        if( crew_ble_hint.active == false )
        {
            crew_ble_accept_movement_only( &movement_hint, movement_profile );
        }
        return;
    }

    dr_hint = movement_hint;
    uint8_t new_profile = ( uint8_t ) dr_hint.minor;
    if(( crew_ble_hint.active == false ) ||
       ( crew_dr_for_ble_profile( new_profile ) <= crew_dr_for_ble_profile( crew_ble_hint.profile )))
    {
        /*
         * Lower requested DR is more conservative. Apply it after movement hysteresis has decided
         * the strongest-beacon change is likely real movement through the vessel.
         */
        crew_dr_accept_ble_profile( &dr_hint, new_profile );
        return;
    }

    /*
     * Higher requested DR is delayed across scan sessions. The T1000-E only scans before an uplink,
     * so scan-session confirmation is more meaningful than counting individual BLE advertisements.
     */
    if( crew_ble_hint.pending_upgrade_profile == new_profile )
    {
        crew_ble_hint.pending_upgrade_count++;
    }
    else
    {
        crew_ble_hint.pending_upgrade_profile = new_profile;
        crew_ble_hint.pending_upgrade_count = 1;
    }

    if( crew_ble_hint.pending_upgrade_count >= CREW_DR_BLE_HINT_UPGRADE_CONFIRM_SCANS )
    {
        crew_dr_accept_ble_profile( &dr_hint, new_profile );
    }
    else
    {
        LOG_LORA( "Crew BLE hint higher-DR pending: minor=%u DR%u count=%u/%u\n", new_profile,
                  crew_dr_for_ble_profile( new_profile ), crew_ble_hint.pending_upgrade_count,
                  CREW_DR_BLE_HINT_UPGRADE_CONFIRM_SCANS );
    }
#endif
}

static bool crew_dr_request_linkcheck( const char* reason )
{
    return crew_dr_request_linkcheck_at_dr( reason, crew_dr.vessel_current );
}

static bool crew_dr_request_linkcheck_at_dr( const char* reason, uint8_t probe_dr )
{
#if CREW_DR_LINKCHECK_ENABLE
    if(( crew_dr_ready == false ) || crew_linkcheck_pending )
    {
        return false;
    }

    probe_dr = crew_clamp_dr( probe_dr, crew_dr.minimum, crew_dr.normal );

    /*
     * LinkCheck probe DR is intentionally separate from the normal vessel uplink DR.
     * A missing LinkCheckAns can mean gateway/LNS/shim congestion, not poor RF. Probe
     * retries may step down to improve the control-frame chance of success, while
     * app_send_frame() continues to re-apply crew_dr.vessel_current for FPort 5 traffic.
     */
    crew_dr_prepare_next_uplink( probe_dr );

    /*
     * The Basics Modem implements LinkCheckReq as a dedicated LINK_CHECK_REQ_TASK.
     * That task sends the MAC command on the LoRaWAN network port (FPort 0), rather than
     * piggybacking it inside the tracker payload on FPort 5. This keeps LinkCheck traffic
     * clear of the current gateway shim filtering path.
     *
     * TODO: Update the gateway shim to forward MAC commands carried in FOpts by default,
     * then we can revisit whether LinkCheck needs to remain a dedicated MAC-only uplink.
     */
    if( smtc_modem_lorawan_request_link_check( stack_id ) != SMTC_MODEM_RC_OK )
    {
        LOG_LORA( "WARN: Crew LinkCheck request failed (%s)\n", reason );
        return false;
    }

    crew_linkcheck_pending = true;
    crew_uplinks_since_linkcheck = 0;
    LOG_LORA( "Crew LinkCheck requested (%s, probe DR%u, app DR%u, dedicated MAC-only FPort 0 task)\n",
              reason, probe_dr, crew_dr.vessel_current );
    return true;
#else
    UNUSED( reason );
    UNUSED( probe_dr );
    return false;
#endif
}

static void crew_linkcheck_retry_reset( void )
{
    crew_linkcheck_retry_active = false;
    crew_linkcheck_retry_probe_dr = 0;
    crew_linkcheck_retry_miss_count = 0;
    crew_linkcheck_retry_not_before_s = 0;
    crew_linkcheck_retry_after_uplink = 0;
}

static void crew_linkcheck_retry_schedule_missing_ans( void )
{
#if CREW_DR_LINKCHECK_ENABLE
    if(( crew_ble_hint.active == false ) || ( crew_dr_ready == false ))
    {
        crew_linkcheck_retry_reset( );
        return;
    }

    crew_linkcheck_retry_active = true;
    crew_linkcheck_retry_miss_count++;

    if(( crew_linkcheck_retry_probe_dr == 0 ) || ( crew_linkcheck_retry_probe_dr > crew_dr.vessel_current ))
    {
        crew_linkcheck_retry_probe_dr = crew_dr.vessel_current;
    }

    if( crew_linkcheck_retry_probe_dr > crew_dr.minimum )
    {
        crew_linkcheck_retry_probe_dr--;
    }

    if( crew_linkcheck_retry_miss_count == 1 )
    {
        /*
         * First retry is time-spread: it happens soon enough to recover from a single lost
         * LinkCheckAns, but adds DevEUI jitter so a group of tags does not create a downlink burst.
         */
        uint32_t jitter_s = crew_dev_eui_jitter_s( 0x4c435231u, CREW_LINKCHECK_RETRY_FIRST_JITTER_S ); /* "LCR1" */
        crew_linkcheck_retry_not_before_s = hal_rtc_get_time_s( ) + CREW_LINKCHECK_RETRY_FIRST_DELAY_S + jitter_s;
        crew_linkcheck_retry_after_uplink = crew_vessel_uplink_count;
    }
    else
    {
        /*
         * Later retries are uplink-phased instead of timer-only. This avoids a fleet-wide
         * synchronized control retry when tags share the same reporting interval.
         */
        uint16_t phase = crew_dev_eui_phase_uplinks( 0x4c435255u, CREW_LINKCHECK_RETRY_UPLINK_INTERVAL ); /* "LCRU" */
        crew_linkcheck_retry_not_before_s = 0;
        crew_linkcheck_retry_after_uplink = crew_vessel_uplink_count + CREW_LINKCHECK_RETRY_UPLINK_INTERVAL + phase;
    }

    LOG_LORA( "Crew LinkCheck retry scheduled: miss=%u probe DR%u app DR%u not_before=%lu after_uplink=%lu\n",
              crew_linkcheck_retry_miss_count, crew_linkcheck_retry_probe_dr, crew_dr.vessel_current,
              crew_linkcheck_retry_not_before_s, crew_linkcheck_retry_after_uplink );
#endif
}

static bool crew_linkcheck_retry_due( uint32_t now_s )
{
#if CREW_DR_LINKCHECK_ENABLE
    if(( crew_linkcheck_retry_active == false ) || crew_linkcheck_pending )
    {
        return false;
    }

    if(( crew_linkcheck_retry_not_before_s != 0 ) && ( now_s < crew_linkcheck_retry_not_before_s ))
    {
        return false;
    }

    return crew_vessel_uplink_count >= crew_linkcheck_retry_after_uplink;
#else
    UNUSED( now_s );
    return false;
#endif
}

static void crew_dr_handle_link_status( smtc_modem_event_link_check_status_t status, uint8_t margin, uint8_t gw_cnt )
{
    if( crew_dr_ready == false )
    {
        return;
    }

    crew_linkcheck_pending = false;

    /*
     * LinkCheck replaces confirmed health uplinks for RF validation. It gives demod margin and gateway count,
     * but still reflects the LNS that owns the LoRaWAN session.
     */
    if(( status != SMTC_MODEM_EVENT_LINK_CHECK_RECEIVED ) || ( gw_cnt == 0 ))
    {
        crew_linkcheck_good_streak = 0;
        LOG_LORA( "Crew LinkCheck inconclusive: status=%u margin=%u gw=%u, keep DR%u\n",
                  status, margin, gw_cnt, crew_dr.vessel_current );
        crew_linkcheck_retry_schedule_missing_ans( );
        return;
    }

    crew_linkcheck_retry_reset( );

    if( margin <= CREW_DR_LINKCHECK_LOW_MARGIN_DB )
    {
        crew_linkcheck_good_streak = 0;
        if( crew_dr.vessel_current > crew_dr.minimum )
        {
            crew_dr.vessel_current--;
            LOG_LORA( "Crew LinkCheck weak: margin=%u gw=%u, step down to DR%u\n",
                      margin, gw_cnt, crew_dr.vessel_current );
            crew_dr_apply_fixed( crew_dr.vessel_current );
            if( crew_ble_hint.active )
            {
                crew_ble_hint.linkcheck_refined_dr = true;
            }
        }
        return;
    }

    if( margin >= CREW_DR_LINKCHECK_HIGH_MARGIN_DB )
    {
        /*
         * BLE Minor is only the installer's initial RF hint. Once the tag is stable enough to run
         * LinkCheckReq, let the measured demodulation margin correct that hint immediately.
         *
         * Keep CREW_DR_LINKCHECK_HIGH_MARGIN_DB as reserve, then spend the surplus margin in
         * conservative CREW_DR_LINKCHECK_MARGIN_PER_DR_DB chunks. Example: with a 10 dB reserve,
         * DR3 and 21 dB margin has 11 dB surplus, so it can move three DR steps, clamped to DR5.
         */
        if( crew_dr.vessel_current < crew_dr.normal )
        {
            uint8_t previous_dr = crew_dr.vessel_current;
            uint8_t spare_margin_db = margin - CREW_DR_LINKCHECK_HIGH_MARGIN_DB;
            uint8_t step_count = 1 + ( spare_margin_db / CREW_DR_LINKCHECK_MARGIN_PER_DR_DB );
            uint8_t target_dr = crew_dr.vessel_current + step_count;

            crew_dr.vessel_current = crew_clamp_dr( target_dr, crew_dr.minimum, crew_dr.normal );
            crew_linkcheck_good_streak = 0;
            LOG_LORA( "Crew LinkCheck strong: margin=%u gw=%u, correct DR%u -> DR%u\n",
                      margin, gw_cnt, previous_dr, crew_dr.vessel_current );
            crew_dr_apply_fixed( crew_dr.vessel_current );
            if( crew_ble_hint.active )
            {
                crew_ble_hint.linkcheck_refined_dr = true;
            }
        }
        else
        {
            LOG_LORA( "Crew LinkCheck strong: margin=%u gw=%u already at DR%u\n",
                      margin, gw_cnt, crew_dr.vessel_current );
        }
    }
    else
    {
        crew_linkcheck_good_streak = 0;
        LOG_LORA( "Crew LinkCheck stable: margin=%u gw=%u DR%u\n", margin, gw_cnt, crew_dr.vessel_current );
    }
}

static void crew_dr_after_vessel_uplink( bool send_ok )
{
#if CREW_DR_LINKCHECK_ENABLE
    if(( crew_dr_ready == false ) || ( send_ok == false ) || (( event_state & TRACKER_STATE_BIT7_SOS ) != 0 ))
    {
        return;
    }

    crew_uplinks_since_linkcheck++;
    crew_vessel_uplink_count++;
    uint32_t now_s = hal_rtc_get_time_s( );

    if( crew_linkcheck_retry_due( now_s ))
    {
        if( crew_dr_request_linkcheck_at_dr( "retry after missing LinkCheckAns", crew_linkcheck_retry_probe_dr ))
        {
            return;
        }
    }

    if( crew_ble_hint.active )
    {
        /*
         * The tag only scans BLE before an uplink, then sleeps. Treat stability as the accepted DR profile
         * staying stable across time, not continuous beacon visibility.
         */
        uint32_t stable_phase_s = crew_dev_eui_jitter_s( 0x4c435354u, CREW_STABLE_LINKCHECK_PHASE_WINDOW_S ); /* "LCST" */
        bool stable_enough = ( now_s - crew_ble_hint.stable_since_s ) >=
                             ( CREW_DR_BLE_HINT_STABLE_LINKCHECK_S + stable_phase_s );
        bool cooldown_ok = ( crew_ble_hint.last_forced_linkcheck_s == 0 ) ||
                           (( now_s - crew_ble_hint.last_forced_linkcheck_s ) >=
                            CREW_DR_BLE_HINT_LINKCHECK_COOLDOWN_S );

        if( stable_enough && cooldown_ok && ( crew_ble_hint.linkcheck_sent_for_profile == false ))
        {
            if( crew_dr_request_linkcheck( "stable BLE hint" ))
            {
                crew_ble_hint.last_forced_linkcheck_s = now_s;
                crew_ble_hint.linkcheck_sent_for_profile = true;
            }
            return;
        }
    }

    uint16_t interval = crew_ble_hint.active ? CREW_DR_LINKCHECK_INTERVAL_WITH_HINT :
                                               CREW_DR_LINKCHECK_INTERVAL_NO_HINT;
    uint16_t phase = crew_dev_eui_phase_uplinks( crew_ble_hint.active ? 0x4c435748u : 0x4c434e48u, interval );
    if(( crew_uplinks_since_linkcheck >= interval ) && (( crew_vessel_uplink_count % interval ) == phase ))
    {
        /*
         * Periodic LinkCheck uses an uplink-count phase, not "every tag on its 60th uplink".
         * This keeps routine network validation distributed when a charger bank wakes many tags together.
         */
        crew_dr_request_linkcheck( crew_ble_hint.active ? "periodic with BLE hint" : "periodic no BLE hint" );
    }
#else
    UNUSED( send_ok );
#endif
}

static void on_modem_network_joined( void )
{
    smtc_modem_region_t region;
    ASSERT_SMTC_MODEM_RC( smtc_modem_get_region( stack_id, &region ));

    if( app_led_state != APP_LED_BLE_CFG )
    {
        uint8_t ativation_mode;
        ativation_mode = smtc_modem_get_activation_mode( stack_id );
        if( ativation_mode == 0 ) // OTAA
        {
            app_beep_joined( );
            app_led_breathe_stop( );
            app_led_lora_joined( );
        }
    }

    if( adr_user_enable == false )
    {
        /* Set the ADR profile based on selected region */
        switch( region )
        {
            case SMTC_MODEM_REGION_EU_868:
                HAL_DBG_TRACE_INFO( "Set ADR profile for EU868\n" );
                if( adr_user_dr_min >= LORAWAN_EU868_DR_MIN && adr_user_dr_max <= LORAWAN_EU868_DR_MAX )
                {
                    if( adr_user_dr_min == adr_user_dr_max )
                    {
                        memset( adr_custom_list_region, adr_user_dr_min, 16 );
                    }
                    else if( adr_user_dr_min > adr_user_dr_max )
                    {
                        memcpy( adr_custom_list_region, adr_custom_list_eu868_default, 16 );
                    }
                    else
                    {
                        custom_lora_adr_compute( adr_user_dr_min, adr_user_dr_max, adr_custom_list_region );
                    }
                }
                else
                {
                    memcpy( adr_custom_list_region, adr_custom_list_eu868_default, 16 );
                }
            break;

            case SMTC_MODEM_REGION_US_915:
                HAL_DBG_TRACE_INFO( "Set ADR profile for US915\n" );
                if( adr_user_dr_min >= LORAWAN_US915_DR_MIN && adr_user_dr_max <= LORAWAN_US915_DR_MAX )
                {
                    if( adr_user_dr_min == adr_user_dr_max )
                    {
                        memset( adr_custom_list_region, adr_user_dr_min, 16 );
                    }
                    else if( adr_user_dr_min > adr_user_dr_max )
                    {
                        memcpy( adr_custom_list_region, adr_custom_list_us915_default, 16 );
                    }
                    else
                    {
                        custom_lora_adr_compute( adr_user_dr_min, adr_user_dr_max, adr_custom_list_region );
                    }
                }
                else
                {
                    memcpy( adr_custom_list_region, adr_custom_list_us915_default, 16 );
                }
            break;

            case SMTC_MODEM_REGION_AU_915:
                HAL_DBG_TRACE_INFO( "Set ADR profile for AU915\n" );
                if( adr_user_dr_min >= LORAWAN_AU915_DR_MIN && adr_user_dr_max <= LORAWAN_AU915_DR_MAX )
                {
                    if( adr_user_dr_min == adr_user_dr_max )
                    {
                        memset( adr_custom_list_region, adr_user_dr_min, 16 );
                    }
                    else if( adr_user_dr_min > adr_user_dr_max )
                    {
                        memcpy( adr_custom_list_region, adr_custom_list_au915_default, 16 );
                    }
                    else
                    {
                        custom_lora_adr_compute( adr_user_dr_min, adr_user_dr_max, adr_custom_list_region );
                    }
                }
                else
                {
                    memcpy( adr_custom_list_region, adr_custom_list_au915_default, 16 );
                }
            break;

            case SMTC_MODEM_REGION_AS_923_GRP1:
            case SMTC_MODEM_REGION_AS_923_GRP2:
            case SMTC_MODEM_REGION_AS_923_GRP3:
            case SMTC_MODEM_REGION_AS_923_GRP4:
            case SMTC_MODEM_REGION_AS_923_HELIUM_1:
            case SMTC_MODEM_REGION_AS_923_HELIUM_2:
            case SMTC_MODEM_REGION_AS_923_HELIUM_3:
            case SMTC_MODEM_REGION_AS_923_HELIUM_4:
            case SMTC_MODEM_REGION_AS_923_HELIUM_1B:
                HAL_DBG_TRACE_INFO( "Set ADR profile for AS923\n" );
                if( adr_user_dr_min >= LORAWAN_AS923_DR_MIN && adr_user_dr_max <= LORAWAN_AS923_DR_MAX )
                {
                    if( adr_user_dr_min == adr_user_dr_max )
                    {
                        memset( adr_custom_list_region, adr_user_dr_min, 16 );
                    }
                    else if( adr_user_dr_min > adr_user_dr_max )
                    {
                        memcpy( adr_custom_list_region, adr_custom_list_as923_default, 16 );
                    }
                    else
                    {
                        custom_lora_adr_compute( adr_user_dr_min, adr_user_dr_max, adr_custom_list_region );
                    }
                }
                else
                {
                    memcpy( adr_custom_list_region, adr_custom_list_as923_default, 16 );
                }
            break;

            case SMTC_MODEM_REGION_KR_920:
                HAL_DBG_TRACE_INFO( "Set ADR profile for KR920\n" );
                if( adr_user_dr_min >= LORAWAN_KR920_DR_MIN && adr_user_dr_max <= LORAWAN_KR920_DR_MAX )
                {
                    if( adr_user_dr_min == adr_user_dr_max )
                    {
                        memset( adr_custom_list_region, adr_user_dr_min, 16 );
                    }
                    else if( adr_user_dr_min > adr_user_dr_max )
                    {
                        memcpy( adr_custom_list_region, adr_custom_list_kr920_default, 16 );
                    }
                    else
                    {
                        custom_lora_adr_compute( adr_user_dr_min, adr_user_dr_max, adr_custom_list_region );
                    }
                }
                else
                {
                    memcpy( adr_custom_list_region, adr_custom_list_kr920_default, 16 );
                }
            break;

            case SMTC_MODEM_REGION_IN_865:
                HAL_DBG_TRACE_INFO( "Set ADR profile for IN865\n" );
                if( adr_user_dr_min >= LORAWAN_IN865_DR_MIN && adr_user_dr_max <= LORAWAN_IN865_DR_MAX )
                {
                    if( adr_user_dr_min == adr_user_dr_max )
                    {
                        memset( adr_custom_list_region, adr_user_dr_min, 16 );
                    }
                    else if( adr_user_dr_min > adr_user_dr_max )
                    {
                        memcpy( adr_custom_list_region, adr_custom_list_in865_default, 16 );
                    }
                    else
                    {
                        custom_lora_adr_compute( adr_user_dr_min, adr_user_dr_max, adr_custom_list_region );
                    }
                }
                else
                {
                    memcpy( adr_custom_list_region, adr_custom_list_in865_default, 16 );
                }
            break;

            case SMTC_MODEM_REGION_RU_864:
                HAL_DBG_TRACE_INFO( "Set ADR profile for RU864\n" );
                if( adr_user_dr_min >= LORAWAN_RU864_DR_MIN && adr_user_dr_max <= LORAWAN_RU864_DR_MAX )
                {
                    if( adr_user_dr_min == adr_user_dr_max )
                    {
                        memset( adr_custom_list_region, adr_user_dr_min, 16 );
                    }
                    else if( adr_user_dr_min > adr_user_dr_max )
                    {
                        memcpy( adr_custom_list_region, adr_custom_list_ru864_default, 16 );
                    }
                    else
                    {
                        custom_lora_adr_compute( adr_user_dr_min, adr_user_dr_max, adr_custom_list_region );
                    }
                }
                else
                {
                    memcpy( adr_custom_list_region, adr_custom_list_ru864_default, 16 );
                }
            break;

            default:
                HAL_DBG_TRACE_ERROR( "Region not supported in this example, could not set custom ADR profile\n" );
            break;
        }

        HAL_DBG_TRACE_PRINTF( "User ADR list: " ); // just for test
        for( uint8_t i = 0; i < 16; i++ )
        {
            HAL_DBG_TRACE_PRINTF( "%d ", adr_custom_list_region[i] ); // just for test
        }
        HAL_DBG_TRACE_PRINTF( "\r\n" ); // just for test

        ASSERT_SMTC_MODEM_RC( smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_region ));
        // ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans_region ));
    }

    crew_dr_configure_for_region( region );
    if( crew_dr_ready )
    {
        crew_dr_apply_fixed( crew_dr.vessel_current );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, 1 ));
    }

    switch( region )
    {
        case SMTC_MODEM_REGION_EU_868:
        case SMTC_MODEM_REGION_RU_864:
            if( duty_cycle_enable == false )
            {
                // Intentionally left blank: public API for disabling regional duty-cycle is not available here.
                // If needed later, handle via modem configuration or network-side settings.
            }
        break;

        default:
        break;
    }

    app_led_bat_new_detect( 3000 );

    /*
     * Use the startup jitter window for docked GNSS/almanac maintenance before
     * the first routine uplink. Steady-state maintenance still runs between
     * TXDONE and the next routine charger uplink.
     */
    gateway_assistance_check_charge_state();

    crew_wait_startup_first_uplink_jitter( );
    if( gateway_assistance_is_background_gnss_active( ))
    {
        /*
         * MDR-022: stop or pause GNSS before LoRa TX so AG3335 and LR11xx
         * high-current paths do not overlap. The first power-on uplink is sent
         * before normal TXDONE maintenance scheduling has had a chance to run.
         */
        LOG_LORA( "Startup uplink: pausing background GNSS before first LoRa TX\n" );
        gateway_assistance_stop_background_gnss( );
    }
    app_lora_packet_power_on_uplink( );

    /* Start DeviceTimeReq time sync service and trigger request. This is best-effort:
     * the first status uplink may already be queued, so BUSY/FAIL must not destabilize startup.
     */
    smtc_modem_return_code_t time_rc = smtc_modem_time_start_sync_service( stack_id, SMTC_MODEM_TIME_MAC_SYNC );
    if(( time_rc == SMTC_MODEM_RC_OK ) || ( time_rc == SMTC_MODEM_RC_FAIL ))
    {
        time_rc = smtc_modem_time_trigger_sync_request( stack_id );
    }
    if( time_rc == SMTC_MODEM_RC_OK )
    {
        last_time_sync_s = hal_rtc_get_time_s();  // Track when we requested sync
        device_time_req_pending = true;
        HAL_DBG_TRACE_INFO( "DeviceTimeReq triggered after join\n" );
    }
    else
    {
        HAL_DBG_TRACE_INFO( "DeviceTimeReq deferred after join (modem rc=%u)\n", time_rc );
    }

    ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( 15 ) );
}

static void on_modem_alarm( void )
{
    smtc_modem_status_mask_t modem_status;
    ASSERT_SMTC_MODEM_RC( smtc_modem_get_status( stack_id, &modem_status ));
    modem_status_to_string( modem_status );
    
    /*
     * If background GNSS is already running from the previous TXDONE, refresh
     * its charger/almanac state before routine scanning. Do not start it here:
     * maintenance is scheduled after uplinks so indoor or stale-almanac tags
     * still send their normal status every interval.
     */
    if( gateway_assistance_is_background_gnss_active() )
    {
        gateway_assistance_check_charge_state();
    }
    
    /* Periodic time sync every 2 hours (7200 seconds) */
    uint32_t current_time_s = hal_rtc_get_time_s();
    if( last_time_sync_s > 0 && (current_time_s - last_time_sync_s) >= 7200 )
    {
        HAL_DBG_TRACE_INFO( "Triggering periodic DeviceTimeReq (2 hour interval)\n" );
        smtc_modem_return_code_t time_rc = smtc_modem_time_trigger_sync_request( stack_id );
        if( time_rc == SMTC_MODEM_RC_OK )
        {
            device_time_req_pending = true;
            last_time_sync_s = current_time_s;
        }
        else
        {
            HAL_DBG_TRACE_INFO( "Periodic DeviceTimeReq deferred (modem rc=%u)\n", time_rc );
        }
    }
    
    app_tracker_scan_process( );
}

static void on_modem_tx_done( smtc_modem_event_txdone_status_t status )
{
    static uint32_t uplink_count = 0;
    LOG_LORA( "Uplink count: %d\n", ++uplink_count );

    if( status == SMTC_MODEM_EVENT_TXDONE_CONFIRMED )
    {
        if( event_state == TRACKER_STATE_BIT8_USER ) // alarm confirm
        {
            app_beep_pos_s( );
        }
    }
    event_state = 0;

    /*
     * Docked almanac maintenance runs between uplinks: start or continue it
     * after LoRa TX has completed, then app_send_frame() pauses it just before
     * the next routine charger uplink. This preserves status cadence even when
     * the charger is indoors and PAIR550 remains stale.
     */
    if( device_time_req_pending )
    {
        HAL_DBG_TRACE_INFO( "Charging GNSS maintenance deferred: DeviceTimeReq pending\n" );
    }
    else
    {
        gateway_assistance_check_charge_state();
    }
}

static void on_modem_time_updated( smtc_modem_event_time_status_t status )
{
    uint32_t gps_time_s = 0;
    uint32_t gps_frac_s = 0;
    
    /* Get current RTC time */
    uint32_t rtc_ms = hal_rtc_get_time_ms();
    uint32_t rtc_s = hal_rtc_get_time_s();
    
    if( smtc_modem_get_time( &gps_time_s, &gps_frac_s ) == SMTC_MODEM_RC_OK )
    {
        /* Convert GPS time to Unix time for display (GPS epoch Jan 6, 1980 = Unix 315964800) */
        /* Current leap seconds between GPS and UTC = 18 seconds */
        uint32_t unix_time = gps_time_s + 315964800 - 18;
        
        /* Calculate difference between modem GPS time and local RTC */
        /* RTC runs in seconds since boot, GPS is absolute time */
        int32_t time_correction_s = (int32_t)gps_time_s - (int32_t)rtc_s;
        
        HAL_DBG_TRACE_INFO( "==== TIME SYNC UPDATE ====\n" );
        HAL_DBG_TRACE_INFO( "Status: %s\n", 
                           status == SMTC_MODEM_EVENT_TIME_VALID ? "VALID" :
                           status == SMTC_MODEM_EVENT_TIME_VALID_BUT_NOT_SYNC ? "VALID_BUT_NOT_SYNC" :
                           "NOT_VALID" );
        HAL_DBG_TRACE_INFO( "Network GPS time: %lu.%03lu s\n", gps_time_s, gps_frac_s / 1000 );
        HAL_DBG_TRACE_INFO( "Network Unix time: %lu (UTC)\n", unix_time );
        HAL_DBG_TRACE_INFO( "Local RTC: %lu s (%lu ms)\n", rtc_s, rtc_ms );
        HAL_DBG_TRACE_INFO( "Time correction: %ld s\n", time_correction_s );
        HAL_DBG_TRACE_INFO( "Modem time is now synchronized with network\n" );
        HAL_DBG_TRACE_INFO( "========================\n" );
        
        device_time_req_pending = false;

        /*
         * A pending DeviceTimeReq may have deferred charger GNSS maintenance to
         * avoid a MAC-only LoRa TX while AG3335 is active. Start maintenance now
         * that the time exchange has completed, then write PAIR590 if active.
         */
        if( gateway_assistance_is_charging() && !gateway_assistance_is_background_gnss_active() )
        {
            gateway_assistance_check_charge_state();
        }

        /* Send time to GNSS if background mode is active (module already powered) */
        if( gateway_assistance_is_background_gnss_active() )
        {
            HAL_DBG_TRACE_INFO( "Background GNSS active - sending time update (PAIR590)\n" );
            gateway_assistance_send_time_to_gnss( true );  // true = GNSS already active
        }
        
        /* Update last sync timestamp for periodic resync */
        last_time_sync_s = rtc_s;
    }
    else
    {
        device_time_req_pending = false;
        HAL_DBG_TRACE_WARNING( "Time sync event but failed to get time\n" );
    }
}

static void on_modem_down_data( int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                                const uint8_t* payload, uint8_t size )
{
    LOG_LORA( "Downlink received:\n" );
    LOG_LORA( "  - LoRaWAN Fport = %d\n", port );
    LOG_LORA( "  - Payload size  = %d\n", size );
    LOG_LORA( "  - RSSI          = %d dBm\n", rssi - 64 );
    LOG_LORA( "  - SNR           = %d dB\n", snr >> 2 );

    switch( rx_window )
    {
        case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1:
        {
            LOG_LORA( "  - Rx window     = %s\n", xstr( SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1 ) );
            break;
        }
        case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2:
        {
            LOG_LORA( "  - Rx window     = %s\n", xstr( SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2 ) );
            break;
        }
        case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC:
        {
            LOG_LORA( "  - Rx window     = %s\n", xstr( SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC ) );
            break;
        }
    }

    if( size != 0 )
    {
        HAL_DBG_TRACE_ARRAY( "Payload", payload, size );

        if( port == LORAWAN_APP_PORT )
        {
            app_lora_packet_downlink_decode( (uint8_t*) payload, size );
        }
        else if( port == GATEWAY_ASSISTANCE_PORT )
        {
            // Handle vessel position and time update
            gateway_assistance_handle_downlink( payload, size );
        }
    }
}

static void app_tracker_ble_scan_begin( void )
{
    tracker_ble_scan_len = 0;
    memset( tracker_ble_scan_data, 0, sizeof( tracker_ble_scan_data ));
    ble_scan_start( );
}

static void app_tracker_ble_scan_end( void )
{
    ble_scan_stop( );
    ble_get_results( tracker_ble_scan_data, &tracker_ble_scan_len );
    crew_dr_handle_ble_hint_scan( );
    ble_display_results( );
    uint8_t len_max = ble_scan_max * CREW_BLE_UPLINK_RECORD_LEN;
    if( tracker_ble_scan_len > len_max ) tracker_ble_scan_len = len_max;
    if( tracker_ble_scan_len ) scan_result_num ++;
    if( scan_result_num > 3 ) scan_result_num = 3;
    if( tracker_test_mode == 0 && tracker_ble_scan_len ) scan_result = true;    
}

static void app_tracker_wifi_scan_begin( void )
{
    tracker_wifi_scan_len = 0;
    memset( tracker_wifi_scan_data, 0, sizeof( tracker_wifi_scan_data ));
    wifi_scan_start( modem_radio );
}

static bool app_tracker_wifi_scan_end( void )
{
    wifi_scan_stop( modem_radio );
    wifi_get_results( modem_radio, tracker_wifi_scan_data, &tracker_wifi_scan_len );
    wifi_display_results( );
    if( wifi_scan_should_scan_remainder_channels( ) )
    {
        LOG_WIFI( "wifi primary channels empty - scan remaining channels\n\n" );
        wifi_scan_prepare_remainder_channels( );
        app_tracker_wifi_scan_begin( );
        return false;
    }

    if( wifi_scan_update_adaptive_state( ) )
    {
        LOG_WIFI( "mobile/random AP only - provisional WiFi accept; next scan max_results=%u\n\n",
                  wifi_scan_get_next_max_results( ) );
        memset( tracker_wifi_scan_data, 0xFF, 7 );
        tracker_wifi_scan_len = 7;
    }

    uint8_t len_max = wifi_scan_max * 7;
    if( tracker_wifi_scan_len > len_max ) tracker_wifi_scan_len = len_max;
    if( tracker_wifi_scan_len ) scan_result_num ++;
    if( scan_result_num > 3 ) scan_result_num = 3;
    if( tracker_test_mode == 0 && tracker_wifi_scan_len ) scan_result = true;
    return true;
}

// REMOVED: app_get_adaptive_gnss_scan_duration()
// Replaced by marine_gnss quality-driven scanning which manages its own timing

static bool app_tracker_gnss_scan_begin( void )
{
    tracker_gps_scan_len = 0;
    memset( tracker_gps_scan_data, 0, sizeof( tracker_gps_scan_data ));

    if(( event_state & TRACKER_STATE_BIT7_SOS ) == 0 )
    {
        LOG_GNSS( "GNSS proof begin - local RF failed, checking vessel geofence position\n\n" );
        gnss_proof_scan_active = gnss_scan_start( );
        gnss_proof_pending = false;
        gnss_proof_quality_ok = false;
        memset( &gnss_proof_fix, 0, sizeof( gnss_proof_fix ));
        return gnss_proof_scan_active;
    }
    
    // Activate marine_gnss quality-driven scanning
    // This handles MOB burst mode -> PIW phases with internal timing
    // marine_gnss manages GNSS power, warm/hot start, BLE checks, and uplinks
    if( !mob_tracker_is_active( ))
    {
        mob_tracker_activate( );
        LOG_GNSS( "Marine GNSS activated - quality-driven scan mode\n" );
    }
    else
    {
        LOG_GNSS( "Marine GNSS already active - continuing\n" );
    }

    return true;
}

static void app_tracker_gnss_scan_end( void )
{
    if( gnss_proof_scan_active )
    {
        gnss_fix_t fix;
        bool got_fix = false;

        memset( &fix, 0, sizeof( fix ));
        got_fix = gnss_get_quality_fix( &fix );
        gnss_scan_stop( );
        gnss_proof_scan_active = false;

        if( got_fix && fix.valid )
        {
            gnss_proof_fix = fix;
            gnss_proof_quality_ok = ( fix.hdop <= PIW_GNSS_MAX_HDOP ) && ( fix.hacc <= PIW_GNSS_MAX_HACC_M );
            gnss_proof_pending = true;
            tracker_gps_scan_len = 1;
            scan_result = true;
            gateway_assistance_store_own_fix( fix.latitude, fix.longitude );
            LOG_GNSS( "GNSS proof fix: lat=%ld lon=%ld HDOP=%.1f HACC=%.1f sats=%u quality=%s\n",
                      fix.latitude,
                      fix.longitude,
                      fix.hdop,
                      fix.hacc,
                      fix.satellites,
                      gnss_proof_quality_ok ? "OK" : "MARGINAL" );
        }
        else
        {
            tracker_gps_scan_len = 0;
            LOG_GNSS( "GNSS proof: no valid fix\n" );
        }
        return;
    }

    // Marine GNSS handles its own uplinks via mob_send_position_uplink()
    // This function is kept for compatibility but marine_gnss manages GNSS lifecycle
    
    const mob_tracker_state_t *state = mob_tracker_get_state( );
    
    if( state->mode == MOB_MODE_CANCELLED )
    {
        LOG_GNSS( "Marine GNSS cancelled - BLE beacon found\n" );
        // Clear scan data - no position uplink needed (cancellation sent by marine_gnss)
        tracker_gps_scan_len = 0;
        scan_result = true;  // Mark as result received to stop retry
    }
    else if( state->last_fix.valid )
    {
        // Store last fix from marine_gnss for gateway assistance
        gateway_assistance_store_own_fix( state->last_fix.latitude, state->last_fix.longitude );
        
        LOG_GNSS( "Marine GNSS fix: lat=%ld, lon=%ld, HDOP=%.1f\n",
                  state->last_fix.latitude, state->last_fix.longitude,
                  state->last_fix.hdop );
        
        // Don't prepare traditional uplink - marine_gnss sends its own MOB uplinks
        // Set scan_result to indicate we got a fix
        tracker_gps_scan_len = 0;  // Prevent duplicate uplink
        scan_result = true;
    }
    else
    {
        LOG_GNSS( "WARN: Marine GNSS - no valid fix yet\n" );
        tracker_gps_scan_len = 0;
    }
}

static uint32_t app_tracker_gnss_next_delay( void )
{
    if( mob_tracker_is_active( ))
    {
        return mob_tracker_process( );
    }

    return gnss_scan_duration > 0 ? gnss_scan_duration : 1;
}

static void app_tracker_scan_result_send( void )
{
    bool send_ok = false;
    bool confirm = false;
    int8_t battery = 0;
    uint8_t outgoing_event_state = event_state | app_tracker_live_event_state_get( );

    if(( packet_policy == RETRY_STATE_1C ) || ( event_state == TRACKER_STATE_BIT8_USER ))
    {
        confirm = true;
    }

    memset( tracker_scan_data_temp, 0, sizeof( tracker_scan_data_temp ));
    tracker_scan_temp_len = 0;

    battery = sensor_bat_sample( );

    PRINTF( "tracker_gps_scan_len: %d\r\n", tracker_gps_scan_len );
    PRINTF( "tracker_wifi_scan_len: %d\r\n", tracker_wifi_scan_len );
    PRINTF( "tracker_ble_scan_len: %d\r\n", tracker_ble_scan_len );
    PRINTF( "scan_result_num: %d\r\n", scan_result_num );

    if( app_tracker_is_sos_event( ))
    {
        scan_result_num = 1;
        send_ok = app_tracker_send_sos_context( outgoing_event_state, battery, confirm );
        if( send_ok )
        {
            tracker_gps_scan_len = 0;
            tracker_wifi_scan_len = 0;
            tracker_ble_scan_len = 0;
        }
    }
    else
    if( gnss_proof_pending )
    {
        scan_result_num = 1;
        send_ok = app_tracker_send_gnss_proof( outgoing_event_state, battery, confirm );
        if( send_ok )
        {
            gnss_proof_pending = false;
            tracker_gps_scan_len = 0;
            tracker_wifi_scan_len = 0;
            tracker_ble_scan_len = 0;
        }
    }
    else
    {
        scan_result_num = 1;
        send_ok = app_tracker_send_compact_presence( outgoing_event_state, battery, confirm );
        if( send_ok )
        {
            tracker_gps_scan_len = 0;
            tracker_wifi_scan_len = 0;
            tracker_ble_scan_len = 0;
        }
    }

    crew_dr_after_vessel_uplink( send_ok );

    if( send_ok ) scan_result_num -= 1;
    if( scan_result_num )
    {
        smtc_modem_alarm_start_timer( LORWAN_SEND_INTERVAL_MIN );
        HAL_DBG_TRACE_PRINTF( "next send, new alarm %d s\n\n", LORWAN_SEND_INTERVAL_MIN );
    }
    else
    {
        tracker_scan_status = 0;
    int32_t next_delay = (int32_t)( tracker_periodic_interval ) - ( hal_rtc_get_time_s( ) - tracker_scan_begin );
        smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
        HAL_DBG_TRACE_PRINTF( "send end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
    }
}

static bool app_tracker_send_compact_presence( uint8_t outgoing_event_state, int8_t battery, bool confirm )
{
    uint8_t payload[sizeof( crew_presence_compact_t ) + sizeof( crew_presence_wifi_ext_t )] = { 0 };
    uint8_t len = 0;
    uint8_t phase = CREW_PRESENCE_PHASE_ROUTINE;
    uint8_t source_flags = CREW_SOURCE_NONE;
    uint8_t source_type = CREW_SOURCE_NONE;

    payload[len++] = CREW_PAYLOAD_SCHEMA_PHASE( CREW_PAYLOAD_SCHEMA_VERSION, phase );
    payload[len++] = outgoing_event_state;
    payload[len++] = (uint8_t)battery;
    payload[len++] = gateway_assistance_get_location_age_min( );
    payload[len++] = source_flags;

    if( tracker_ble_scan_len >= CREW_BLE_UPLINK_RECORD_LEN )
    {
        uint16_t major = 0;
        uint16_t minor = 0;

        phase = CREW_PRESENCE_PHASE_ONBOARD;
        source_type = CREW_SOURCE_BLE;
        source_flags = CREW_SOURCE_BLE | CREW_SOURCE_FLAG_ACCEPTED |
                       CREW_SOURCE_FLAG_RSSI_PRESENT | CREW_SOURCE_FLAG_IDENTITY_PRESENT;

        memcpyr( (uint8_t*)&major, tracker_ble_scan_data, 2 );
        memcpyr( (uint8_t*)&minor, tracker_ble_scan_data + 2, 2 );
        app_tracker_u16_le( payload + len, major );
        len += 2;
        app_tracker_u16_le( payload + len, minor );
        len += 2;
        payload[len++] = tracker_ble_scan_data[4];
    }
    else if( tracker_wifi_scan_len >= 7 )
    {
        bool provisional = true;

        for( uint8_t i = 0; i < 6; i++ )
        {
            if( tracker_wifi_scan_data[i] != 0xFF )
            {
                provisional = false;
                break;
            }
        }

        phase = CREW_PRESENCE_PHASE_ONBOARD;
        source_type = provisional ? CREW_SOURCE_WIFI_PROVISIONAL : CREW_SOURCE_WIFI_FIXED;
        source_flags = source_type | CREW_SOURCE_FLAG_ACCEPTED |
                       CREW_SOURCE_FLAG_RSSI_PRESENT | CREW_SOURCE_FLAG_IDENTITY_PRESENT;

        memcpy( payload + len, tracker_wifi_scan_data, 6 );
        len += 6;
        payload[len++] = tracker_wifi_scan_data[6];
        payload[len++] = provisional ? 0x01 : 0x00;
    }
    else if( tracker_gps_scan_len > 0 )
    {
        phase = CREW_PRESENCE_PHASE_ONBOARD;
        source_type = CREW_SOURCE_GNSS_OR_ASSISTANCE;
        source_flags = CREW_SOURCE_GNSS_OR_ASSISTANCE | CREW_SOURCE_FLAG_ACCEPTED;
    }

    payload[0] = CREW_PAYLOAD_SCHEMA_PHASE( CREW_PAYLOAD_SCHEMA_VERSION, phase );
    payload[4] = source_flags;

    LOG_LORA( "Compact presence uplink: phase=%u source=%u len=%u loc_age=%u\n",
              phase, source_type, len, payload[3] );

    return app_send_frame( payload, len, confirm, false );
}

static bool app_tracker_send_gnss_proof( uint8_t outgoing_event_state, int8_t battery, bool confirm )
{
    uint8_t payload[sizeof( crew_gnss_proof_t )] = { 0 };
    uint8_t len = 0;
    uint8_t quality_flags = CREW_GNSS_PROOF_QUALITY_FIX_VALID;
    uint16_t hdop_x10 = (uint16_t)( gnss_proof_fix.hdop * 10.0f );

    if( gnss_proof_quality_ok )
    {
        quality_flags |= CREW_GNSS_PROOF_QUALITY_OK;
    }
    if( gateway_assistance_is_charging( ))
    {
        quality_flags |= CREW_GNSS_PROOF_QUALITY_ON_CHARGE;
    }

    payload[len++] = CREW_ALERT_SUBTYPE_GNSS_PROOF;
    payload[len++] = CREW_PAYLOAD_SCHEMA_PHASE( CREW_PAYLOAD_SCHEMA_VERSION, CREW_PRESENCE_PHASE_UNCERTAIN );
    payload[len++] = outgoing_event_state;
    payload[len++] = (uint8_t)battery;
    payload[len++] = gateway_assistance_get_location_age_min( );
    payload[len++] = quality_flags;
    payload[len++] = ( hdop_x10 > 255U ) ? 255U : (uint8_t)hdop_x10;
    payload[len++] = gnss_proof_fix.satellites;
    app_tracker_i32_le( payload + len, gnss_proof_fix.latitude );
    len += 4;
    app_tracker_i32_le( payload + len, gnss_proof_fix.longitude );
    len += 4;

    LOG_LORA( "GNSS proof uplink: lat=%ld lon=%ld HDOP=%.1f sats=%u quality=%02X len=%u\n",
              gnss_proof_fix.latitude,
              gnss_proof_fix.longitude,
              gnss_proof_fix.hdop,
              gnss_proof_fix.satellites,
              quality_flags,
              len );

    return app_send_mob_frame( payload, len, confirm, APP_MOB_DR_PERSISTENCE );
}

static uint8_t app_tracker_live_event_state_get( void )
{
    uint8_t live_state = 0;

    if( gateway_assistance_is_charging( ))
    {
        live_state |= TRACKER_STATE_BIT3_ON_CHARGE;
    }
    if( gnss_almanac_is_valid( ))
    {
        live_state |= TRACKER_STATE_BIT6_GNSS_READY;
    }

    return live_state;
}

static bool app_tracker_is_sos_event( void )
{
    return ( event_state & TRACKER_STATE_BIT7_SOS ) != 0;
}

static bool app_tracker_payload_is_sos_context( const uint8_t* buffer, uint8_t length )
{
    return ( length >= 7 )
        && ( buffer[0] == CREW_SOS_CONTEXT_SUBTYPE )
        && (( buffer[1] & 0xF0 ) == CREW_SOS_CONTEXT_SCHEMA )
        && (( buffer[2] & TRACKER_STATE_BIT7_SOS ) != 0 );
}

static void app_tracker_u16_le( uint8_t* buffer, uint16_t value )
{
    buffer[0] = (uint8_t)( value & 0xFF );
    buffer[1] = (uint8_t)(( value >> 8 ) & 0xFF );
}

static void app_tracker_i32_le( uint8_t* buffer, int32_t value )
{
    uint32_t raw = (uint32_t)value;
    buffer[0] = (uint8_t)( raw & 0xFF );
    buffer[1] = (uint8_t)(( raw >> 8 ) & 0xFF );
    buffer[2] = (uint8_t)(( raw >> 16 ) & 0xFF );
    buffer[3] = (uint8_t)(( raw >> 24 ) & 0xFF );
}

static bool app_tracker_send_sos_context( uint8_t outgoing_event_state, int8_t battery, bool confirm )
{
    uint8_t payload[LORAWAN_APP_DATA_MAX_SIZE];
    uint8_t len = 0;
    uint8_t source_flags = 0;
    uint8_t evidence_flags = 0;
    uint8_t reason = 0;
    bool local_source_accepted = false;
    bool background_active = gateway_assistance_is_background_gnss_active( );
    bool prestarted_active = sos_gnss_prestarted && gnss_is_active( );
    bool gnss_active_for_sos = background_active || prestarted_active || gnss_is_active( );
    gnss_fix_t fix;
    bool got_good_fix = false;

    memset( &fix, 0, sizeof( fix ));

    if(( tracker_scan_type == TRACKER_SCAN_BLE_ONLY )
       || ( tracker_scan_type == TRACKER_SCAN_BLE_WIFI )
       || ( tracker_scan_type == TRACKER_SCAN_BLE_GNSS )
       || ( tracker_scan_type == TRACKER_SCAN_BLE_WIFI_GNSS )
       || ( tracker_ble_scan_len > 0 ))
    {
        evidence_flags |= 0x01;
    }
    if(( tracker_scan_type == TRACKER_SCAN_WIFI_ONLY )
       || ( tracker_scan_type == TRACKER_SCAN_WIFI_GNSS )
       || ( tracker_scan_type == TRACKER_SCAN_GNSS_WIFI )
       || ( tracker_scan_type == TRACKER_SCAN_BLE_WIFI )
       || ( tracker_scan_type == TRACKER_SCAN_BLE_WIFI_GNSS )
       || ( tracker_wifi_scan_len > 0 ))
    {
        evidence_flags |= 0x02;
    }

    evidence_flags |= 0x04;

    if( tracker_ble_scan_len >= CREW_BLE_UPLINK_RECORD_LEN )
    {
        source_flags = 0x01 | 0x08 | 0x10 | 0x20;
        local_source_accepted = true;
        evidence_flags |= 0x08;
    }
    else if( tracker_wifi_scan_len >= 7 )
    {
        bool provisional = true;
        for( uint8_t i = 0; i < 6; i++ )
        {
            if( tracker_wifi_scan_data[i] != 0xFF )
            {
                provisional = false;
                break;
            }
        }
        source_flags = ( provisional ? 0x03 : 0x02 ) | 0x08 | 0x10 | 0x20;
        local_source_accepted = true;
        evidence_flags |= 0x08;
    }

    LOG_GNSS( "SOS GNSS context: sampling current GNSS fix after local scans\n\n" );
    if( gnss_active_for_sos && gnss_get_quality_fix( &fix ))
    {
        got_good_fix = ( fix.hdop <= PIW_GNSS_MAX_HDOP ) && ( fix.hacc <= PIW_GNSS_MAX_HACC_M );
        LOG_GNSS( "SOS GNSS context: fix sampled HDOP=%.1f HACC=%.1f sats=%u quality=%s\n\n",
                  fix.hdop,
                  fix.hacc,
                  fix.satellites,
                  got_good_fix ? "GOOD" : "MARGINAL" );
    }
    else
    {
        LOG_GNSS( "SOS GNSS context: no GNSS fix available at local scan completion\n\n" );
    }
    if( fix.valid )
    {
        evidence_flags |= 0x10;
        gateway_assistance_store_own_fix( fix.latitude, fix.longitude );
    }
    if( got_good_fix )
    {
        evidence_flags |= 0x20;
    }
    if( !fix.valid )
    {
        evidence_flags |= 0x40;
        reason = 5;
    }
    else if( !local_source_accepted )
    {
        source_flags = 0x04;
    }

    if( background_active )
    {
        LOG_GNSS( "SOS GNSS context: pausing background GNSS before LoRa TX\n\n" );
        gateway_assistance_stop_background_gnss( );
    }
    else if( prestarted_active )
    {
        LOG_GNSS( "SOS GNSS context: stopping prestarted GNSS before LoRa TX\n\n" );
        gnss_scan_stop( );
    }
    sos_gnss_prestarted = false;

    payload[len++] = CREW_SOS_CONTEXT_SUBTYPE;
    payload[len++] = CREW_SOS_CONTEXT_SCHEMA | 0x01;
    payload[len++] = outgoing_event_state | TRACKER_STATE_BIT7_SOS;
    payload[len++] = (uint8_t)battery;
    payload[len++] = source_flags;
    payload[len++] = evidence_flags;
    payload[len++] = reason;

    if( tracker_ble_scan_len >= CREW_BLE_UPLINK_RECORD_LEN )
    {
        uint16_t major = 0;
        uint16_t minor = 0;

        memcpyr( (uint8_t*)&major, tracker_ble_scan_data, 2 );
        memcpyr( (uint8_t*)&minor, tracker_ble_scan_data + 2, 2 );
        app_tracker_u16_le( payload + len, major );
        len += 2;
        app_tracker_u16_le( payload + len, minor );
        len += 2;
        payload[len++] = tracker_ble_scan_data[4];
    }
    else if( tracker_wifi_scan_len >= 7 )
    {
        memcpy( payload + len, tracker_wifi_scan_data, 6 );
        len += 6;
        payload[len++] = (( source_flags & 0x07 ) == 0x03 ) ? 0x01 : 0x00;
        payload[len++] = tracker_wifi_scan_data[6];
    }

    if( fix.valid && ( len + 10 <= sizeof( payload )))
    {
        uint16_t hdop_x10 = (uint16_t)( fix.hdop * 10.0f );
        app_tracker_i32_le( payload + len, fix.latitude );
        len += 4;
        app_tracker_i32_le( payload + len, fix.longitude );
        len += 4;
        payload[len++] = ( hdop_x10 > 255U ) ? 255U : (uint8_t)hdop_x10;
        payload[len++] = fix.satellites;
    }

    LOG_LORA( "SOS context uplink: local=%u gnss_valid=%u good=%u no_fix=%u len=%u\n",
              local_source_accepted ? 1 : 0,
              fix.valid ? 1 : 0,
              got_good_fix ? 1 : 0,
              ( evidence_flags & 0x40 ) ? 1 : 0,
              len );

    return app_send_frame( payload, len, confirm, true );
}

static void app_tracker_sos_gnss_prestart( void )
{
    if( !app_tracker_is_sos_event( ))
    {
        return;
    }

    if( gateway_assistance_is_background_gnss_active( ) || gnss_is_active( ))
    {
        sos_gnss_prestarted = false;
        LOG_GNSS( "SOS GNSS prestart: GNSS already active\n\n" );
        return;
    }

    if( !app_gnss_initialized )
    {
        LOG_GNSS( "SOS GNSS prestart skipped: GNSS not initialized for current positioning mode\n\n" );
        return;
    }

    LOG_GNSS( "SOS GNSS prestart: starting GNSS immediately on SOS enter\n\n" );
    sos_gnss_prestarted = gnss_scan_start( );
}

static void app_tracker_scan_process( void )
{
    int32_t next_delay = 0;

    scan_result = false;

    if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_GNSS_ONLY ))
    {
        if( tracker_scan_status == 0 )
        {
            // Start marine_gnss quality-driven scanning
            LOG_GNSS( "marine_gnss begin\n\n" );
            tracker_scan_begin = hal_rtc_get_time_s( );
            if( app_tracker_gnss_scan_begin( ))
            {
                // Process first cycle and get next delay
                uint32_t marine_delay = app_tracker_gnss_next_delay( );
                smtc_modem_alarm_start_timer( marine_delay );
                LOG_GNSS( "marine_gnss next alarm %lu s\n\n", marine_delay );
                tracker_scan_status = 1;
            }
            else
            {
                tracker_scan_status = 0xff;
            }
        }
        else if( tracker_scan_status == 1 )
        {
            // Process marine_gnss state machine
            if( mob_tracker_is_active( ))
            {
                uint32_t marine_delay = app_tracker_gnss_next_delay( );
                smtc_modem_alarm_start_timer( marine_delay );
                LOG_GNSS( "marine_gnss continuing, next alarm %lu s\n\n", marine_delay );
                // Stay in status 1 until cancelled
            }
            else
            {
                // Marine GNSS cancelled or idle
                app_tracker_gnss_scan_end( );
                next_delay = (int32_t)( tracker_periodic_interval ) - ( hal_rtc_get_time_s( ) - tracker_scan_begin );
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_GNSS( "marine_gnss end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
        }
    }
    else if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_WIFI_ONLY ))
    {
        if( tracker_scan_status == 0 )
        {
            smtc_modem_alarm_start_timer( wifi_scan_duration );
            LOG_WIFI( "wifi begin, new alarm %d s\n\n", wifi_scan_duration );
            tracker_scan_begin = hal_rtc_get_time_s( );
            app_tracker_wifi_scan_begin( );
            tracker_scan_status = 1;
        }
        else if( tracker_scan_status == 1 )
        {
            if( app_tracker_wifi_scan_end( ) == false )
            {
                smtc_modem_alarm_start_timer( wifi_scan_duration );
                LOG_WIFI( "wifi remainder begin, new alarm %d s\n\n", wifi_scan_duration );
            }
            else
            {
                next_delay = (int32_t)( tracker_periodic_interval ) - wifi_scan_duration;
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_WIFI( "wifi end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
        }
    }
    else if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_WIFI_GNSS ))
    {
        if( tracker_scan_status == 0 )
        {
            smtc_modem_alarm_start_timer( wifi_scan_duration );
            LOG_WIFI( "wifi begin, new alarm %d s\n\n", wifi_scan_duration );
            tracker_scan_begin = hal_rtc_get_time_s( );
            app_tracker_wifi_scan_begin( );
            tracker_scan_status = 1;
        }
        else if( tracker_scan_status == 1 )
        {
            if( app_tracker_wifi_scan_end( ) == false )
            {
                smtc_modem_alarm_start_timer( wifi_scan_duration );
                LOG_WIFI( "wifi remainder begin, new alarm %d s\n\n", wifi_scan_duration );
                return;
            }
            if( scan_result )
            {
                next_delay = (int32_t)( tracker_periodic_interval ) - wifi_scan_duration;
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_WIFI( "wifi end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
            else
            {
                // WiFi failed - start marine_gnss
                LOG_WIFI( "wifi end\r\n" );
                LOG_GNSS( "marine_gnss begin\n\n" );
                if( app_tracker_gnss_scan_begin( ))
                {
                    uint32_t marine_delay = app_tracker_gnss_next_delay( );
                    smtc_modem_alarm_start_timer( marine_delay );
                    tracker_scan_status = 2;
                }
                else
                {
                    tracker_scan_status = 0xff;
                }
            }          
        }
        else if( tracker_scan_status == 2 )
        {
            // Process marine_gnss state machine
            if( mob_tracker_is_active( ))
            {
                uint32_t marine_delay = app_tracker_gnss_next_delay( );
                smtc_modem_alarm_start_timer( marine_delay );
                LOG_GNSS( "marine_gnss continuing, next alarm %lu s\n\n", marine_delay );
            }
            else
            {
                app_tracker_gnss_scan_end( );
                next_delay = (int32_t)( tracker_periodic_interval ) - ( hal_rtc_get_time_s( ) - tracker_scan_begin );
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_GNSS( "marine_gnss end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
        }
    }
    else if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_GNSS_WIFI ))
    {
        if( tracker_scan_status == 0 )
        {
            // Start marine_gnss first
            LOG_GNSS( "marine_gnss begin\n\n" );
            tracker_scan_begin = hal_rtc_get_time_s( );
            if( app_tracker_gnss_scan_begin( ))
            {
                uint32_t marine_delay = app_tracker_gnss_next_delay( );
                smtc_modem_alarm_start_timer( marine_delay );
                tracker_scan_status = 1;
            }
            else
            {
                tracker_scan_status = 0xff;
            }
        }
        else if( tracker_scan_status == 1 )
        {
            if( mob_tracker_is_active( ))
            {
                uint32_t marine_delay = app_tracker_gnss_next_delay( );
                smtc_modem_alarm_start_timer( marine_delay );
                LOG_GNSS( "marine_gnss continuing, next alarm %lu s\n\n", marine_delay );
            }
            else
            {
                app_tracker_gnss_scan_end( );
                if( scan_result )
                {
                    next_delay = (int32_t)( tracker_periodic_interval ) - ( hal_rtc_get_time_s( ) - tracker_scan_begin );
                    smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                    LOG_GNSS( "marine_gnss end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                    tracker_scan_status = 0xff;
                }
                else
                {
                    smtc_modem_alarm_start_timer( wifi_scan_duration );
                    LOG_GNSS( "marine_gnss end\r\n" );
                    LOG_WIFI( "wifi begin, new alarm %d s\n\n", wifi_scan_duration );
                    app_tracker_wifi_scan_begin( );
                    tracker_scan_status = 2;
                }
            }
        }
        else if( tracker_scan_status == 2 )
        {
            if( app_tracker_wifi_scan_end( ) == false )
            {
                smtc_modem_alarm_start_timer( wifi_scan_duration );
                LOG_WIFI( "wifi remainder begin, new alarm %d s\n\n", wifi_scan_duration );
            }
            else
            {
                next_delay = (int32_t)( tracker_periodic_interval ) - ( hal_rtc_get_time_s( ) - tracker_scan_begin );
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_WIFI( "wifi end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
        }
    }
    else if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_BLE_ONLY ))
    {
        if( tracker_scan_status == 0 )
        {
            smtc_modem_alarm_start_timer( ble_scan_duration );
            LOG_BLE( "ble begin, new alarm %d s\n\n", ble_scan_duration );
            tracker_scan_begin = hal_rtc_get_time_s( );
            app_tracker_ble_scan_begin( );
            tracker_scan_status = 1;
        }
        else if( tracker_scan_status == 1 )
        {
            next_delay = (int32_t)( tracker_periodic_interval ) - ble_scan_duration;
            smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
            LOG_BLE( "ble end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
            app_tracker_ble_scan_end( );
            tracker_scan_status = 0xff;
        }
    }
    else if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_BLE_WIFI ))
    {
        if( tracker_scan_status == 0 )
        {
            smtc_modem_alarm_start_timer( ble_scan_duration );
            LOG_BLE( "ble begin, new alarm %d s\n\n", ble_scan_duration );
            tracker_scan_begin = hal_rtc_get_time_s( );
            app_tracker_ble_scan_begin( );
            tracker_scan_status = 1;
        }
        else if( tracker_scan_status == 1 )
        {
            app_tracker_ble_scan_end( );
            if( scan_result )
            {
                next_delay = (int32_t)( tracker_periodic_interval ) - ble_scan_duration;
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_BLE( "ble end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
            else
            {
                smtc_modem_alarm_start_timer( wifi_scan_duration );
                LOG_BLE( "ble end\r\n" );
                LOG_WIFI( "wifi begin, new alarm %d s\n\n", wifi_scan_duration );
                app_tracker_wifi_scan_begin( );
                tracker_scan_status = 2;
            }
        }
        else if( tracker_scan_status == 2 )
        {
            if( app_tracker_wifi_scan_end( ) == false )
            {
                smtc_modem_alarm_start_timer( wifi_scan_duration );
                LOG_WIFI( "wifi remainder begin, new alarm %d s\n\n", wifi_scan_duration );
            }
            else
            {
                next_delay = (int32_t)( tracker_periodic_interval ) - ble_scan_duration - wifi_scan_duration;
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_WIFI( "wifi end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
        }
    }
    else if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_BLE_GNSS ))
    {
        if( tracker_scan_status == 0 )
        {
            // Run BLE scan first
            smtc_modem_alarm_start_timer( ble_scan_duration );
            LOG_BLE( "ble begin, new alarm %d s\n\n", ble_scan_duration );
            tracker_scan_begin = hal_rtc_get_time_s( );
            app_tracker_ble_scan_begin( );
            tracker_scan_status = 1;
        }
        else if( tracker_scan_status == 1 )
        {
            app_tracker_ble_scan_end( );
            if( scan_result )
            {
                // BLE found - cancel marine_gnss if active
                if( mob_tracker_is_active( ))
                {
                    mob_tracker_cancel( );
                }
                next_delay = (int32_t)( tracker_periodic_interval ) - ble_scan_duration;
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_BLE( "ble end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
            else
            {
                // BLE failed - start marine_gnss
                LOG_BLE( "ble end\r\n" );
                LOG_GNSS( "marine_gnss begin\n\n" );
                if( app_tracker_gnss_scan_begin( ))
                {
                    uint32_t marine_delay = app_tracker_gnss_next_delay( );
                    smtc_modem_alarm_start_timer( marine_delay );
                    tracker_scan_status = 2;
                }
                else
                {
                    tracker_scan_status = 0xff;
                }
            }
        }
        else if( tracker_scan_status == 2 )
        {
            if( mob_tracker_is_active( ))
            {
                uint32_t marine_delay = app_tracker_gnss_next_delay( );
                smtc_modem_alarm_start_timer( marine_delay );
                LOG_GNSS( "marine_gnss continuing, next alarm %lu s\n\n", marine_delay );
            }
            else
            {
                app_tracker_gnss_scan_end( );
                next_delay = (int32_t)( tracker_periodic_interval ) - ( hal_rtc_get_time_s( ) - tracker_scan_begin );
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_GNSS( "marine_gnss end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
        }
    }
    else if(( scan_result == false ) && ( tracker_scan_type == TRACKER_SCAN_BLE_WIFI_GNSS ))
    {
        if( tracker_scan_status == 0 )
        {
            // Always run BLE scan first, even during almanac maintenance
            smtc_modem_alarm_start_timer( ble_scan_duration );
            LOG_BLE( "ble begin, new alarm %d s\n\n", ble_scan_duration );
            tracker_scan_begin = hal_rtc_get_time_s( );
            app_tracker_ble_scan_begin( );
            tracker_scan_status = 1;
        }
        else if( tracker_scan_status == 1 )
        {
            app_tracker_ble_scan_end( );
            if( scan_result )
            {
                // BLE found - cancel marine_gnss if active
                if( mob_tracker_is_active( ))
                {
                    mob_tracker_cancel( );
                }
                next_delay = (int32_t)( tracker_periodic_interval ) - ble_scan_duration;
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_BLE( "ble end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
            else
            {
                smtc_modem_alarm_start_timer( wifi_scan_duration );
                LOG_BLE( "ble end\r\n" );
                LOG_WIFI( "wifi begin, new alarm %d s\n\n", wifi_scan_duration );
                app_tracker_wifi_scan_begin( );
                tracker_scan_status = 2;
            }
        }
        else if( tracker_scan_status == 2 )
        {
            if( app_tracker_wifi_scan_end( ) == false )
            {
                smtc_modem_alarm_start_timer( wifi_scan_duration );
                LOG_WIFI( "wifi remainder begin, new alarm %d s\n\n", wifi_scan_duration );
                return;
            }
            if( scan_result )
            {
                next_delay = (int32_t)( tracker_periodic_interval ) - ble_scan_duration - wifi_scan_duration;
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_WIFI( "wifi end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
            else
            {
                // WiFi failed - start marine_gnss
                LOG_WIFI( "wifi end\r\n" );
                LOG_GNSS( "marine_gnss begin\n\n" );
                if( app_tracker_gnss_scan_begin( ))
                {
                    uint32_t marine_delay = app_tracker_gnss_next_delay( );
                    smtc_modem_alarm_start_timer( marine_delay );
                    tracker_scan_status = 3;
                }
                else
                {
                    tracker_scan_status = 0xff;
                }
            }
        }
        else if( tracker_scan_status == 3 )
        {
            if( mob_tracker_is_active( ))
            {
                uint32_t marine_delay = app_tracker_gnss_next_delay( );
                smtc_modem_alarm_start_timer( marine_delay );
                LOG_GNSS( "marine_gnss continuing, next alarm %lu s\n\n", marine_delay );
            }
            else
            {
                app_tracker_gnss_scan_end( );
                next_delay = (int32_t)( tracker_periodic_interval ) - ( hal_rtc_get_time_s( ) - tracker_scan_begin );
                smtc_modem_alarm_start_timer( next_delay > 0 ? next_delay : 1 );
                LOG_GNSS( "marine_gnss end, new alarm %d s\n\n", next_delay > 0 ? next_delay : 1 );
                tracker_scan_status = 0xff;
            }
        }
    }

    if( tracker_scan_status == 0xff )
    {
        app_tracker_scan_result_send( );
    }
}

static bool app_send_frame_on_port( uint8_t port, const uint8_t* buffer, const uint8_t length, bool tx_confirmed,
                                    bool emergency )
{
    uint8_t tx_max_payload;
    int32_t duty_cycle;

    /* Check if duty cycle is available */
    ASSERT_SMTC_MODEM_RC( smtc_modem_get_duty_cycle_status( &duty_cycle ) );
    if( duty_cycle < 0 )
    {
        LOG_LORA( "WARN: Duty-cycle limitation - next possible uplink in %d ms \n\n", duty_cycle );
        return false;
    }

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_next_tx_max_payload( stack_id, &tx_max_payload ) );
    if( length > tx_max_payload )
    {
        LOG_LORA( "WARN: Not enough space in buffer - send empty uplink to flush MAC commands \n" );
        ASSERT_SMTC_MODEM_RC( smtc_modem_request_empty_uplink( stack_id, true, port, tx_confirmed ));
        return false;
    }
    else
    {
        LOG_LORA( "Request uplink\n" );
        if( emergency )
        {
            ASSERT_SMTC_MODEM_RC ( smtc_modem_request_emergency_uplink( stack_id, port, tx_confirmed, buffer, length ));
        }
        else
        {
            ASSERT_SMTC_MODEM_RC( smtc_modem_request_uplink( stack_id, port, tx_confirmed, buffer, length ));
        }
        return true;
    }
}

static bool app_send_frame_on_port_ext( uint8_t port, const uint8_t* buffer, const uint8_t length, bool tx_confirmed,
                                        bool emergency, uint8_t extended_id )
{
    uint8_t tx_max_payload;
    int32_t duty_cycle;

    if(( extended_id == 0 ) || ( extended_id > 2 ))
    {
        return app_send_frame_on_port( port, buffer, length, tx_confirmed, emergency );
    }

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_duty_cycle_status( &duty_cycle ) );
    if( duty_cycle < 0 )
    {
        LOG_LORA( "WARN: Duty-cycle limitation - next possible uplink in %d ms \n\n", duty_cycle );
        return false;
    }

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_next_tx_max_payload( stack_id, &tx_max_payload ) );
    if( length > tx_max_payload )
    {
        LOG_LORA( "WARN: Not enough space in extended uplink buffer\n" );
        return false;
    }

    memcpy( crew_burst_ext_payload[extended_id - 1], buffer, length );
    crew_burst_ext_len[extended_id - 1] = length;

    /* The SDK has no emergency-priority extended uplink API; only burst slot 0 can use emergency priority. */
    UNUSED( emergency );
    ASSERT_SMTC_MODEM_RC( smtc_modem_request_extended_uplink( stack_id, port, tx_confirmed,
                                                              crew_burst_ext_payload[extended_id - 1],
                                                              crew_burst_ext_len[extended_id - 1],
                                                              extended_id, crew_extended_uplink_done ));

    return true;
}

bool app_send_frame( const uint8_t* buffer, const uint8_t length, bool tx_confirmed, bool emergency )
{
    bool tracker_scan_payload = ( length > 1 ) && ( buffer[0] != DATA_ID_UP_PACKET_POWER );
    bool sos_context_payload = app_tracker_payload_is_sos_context( buffer, length );
    bool on_charge = gateway_assistance_is_charging( );

    if(( tracker_scan_payload && (( buffer[1] & TRACKER_STATE_BIT7_SOS ) != 0 )) || sos_context_payload )
    {
        if( crew_dr_ready )
        {
            return crew_send_dr_burst_on_port( CREW_ALERT_APP_PORT, buffer, length, tx_confirmed, true );
        }

        return app_send_frame_on_port( CREW_ALERT_APP_PORT, buffer, length, tx_confirmed, true );
    }

    /*
     * Normal vessel uplinks must re-tag the queued modem task with the current crew DR.
     * Background MAC tasks such as DeviceTimeReq/LinkCheckReq can otherwise advance or alter the
     * modem ADR state between BLE scans, which showed up in field logs as Minor 3 selecting DR3
     * once and later FPort 5 uplinks drifting down to SF11.
     */
    if( crew_dr_ready )
    {
        crew_dr_prepare_next_uplink( crew_dr.vessel_current );
    }

    if( gateway_assistance_is_background_gnss_active( ))
    {
        /*
         * AG3335 and LR11xx TX are both high-current paths. Pause charger
         * maintenance before every normal LoRa uplink, including the startup
         * power-on packet, then TXDONE can restart maintenance if still needed.
         */
        LOG_LORA( "Normal uplink: pausing background GNSS before LoRa TX\n" );
        gateway_assistance_stop_background_gnss( );
    }
    else if( gnss_is_active( ))
    {
        LOG_LORA( "Normal uplink: stopping active GNSS before LoRa TX\n" );
        gnss_scan_stop( );
        gnss_proof_scan_active = false;
        sos_gnss_prestarted = false;
    }

    if( on_charge )
    {
        LOG_LORA( "Routine uplink on charger: route FPort %u\n", CREW_CHARGER_APP_PORT );
        return app_send_frame_on_port( CREW_CHARGER_APP_PORT, buffer, length, tx_confirmed, emergency );
    }

    return app_send_frame_on_port( LORAWAN_APP_PORT, buffer, length, tx_confirmed, emergency );
}

bool app_send_mob_frame( const uint8_t* buffer, const uint8_t length, bool tx_confirmed, app_mob_dr_policy_t policy )
{
    if( crew_dr_ready )
    {
        crew_dr_prepare_next_uplink( crew_dr_for_mob_policy( policy ) );
    }

    return app_send_frame_on_port( CREW_ALERT_APP_PORT, buffer, length, tx_confirmed, false );
}

bool app_send_mob_initial_burst( const uint8_t* buffer, const uint8_t length, bool tx_confirmed )
{
    if( crew_dr_ready == false )
    {
        return app_send_frame_on_port( CREW_ALERT_APP_PORT, buffer, length, tx_confirmed, true );
    }

    return crew_send_dr_burst_on_port( CREW_ALERT_APP_PORT, buffer, length, tx_confirmed, true );
}

static bool crew_send_dr_burst_on_port( uint8_t port, const uint8_t* buffer, const uint8_t length, bool tx_confirmed,
                                        bool emergency )
{
    uint8_t burst_dr[3];
    bool send_ok = true;

    if( crew_dr_ready == false )
    {
        return app_send_frame_on_port( port, buffer, length, tx_confirmed, emergency );
    }

    /* Same payload, three link budgets: fast reacquisition first, then persistence, then SOS penetration DR. */
    burst_dr[0] = crew_dr.normal;
    burst_dr[1] = crew_dr.persistence;
    burst_dr[2] = crew_dr.sos_low;

    for( uint8_t i = 0; i < 3; i++ )
    {
        crew_dr_prepare_next_uplink( burst_dr[i] );
        send_ok &= app_send_frame_on_port_ext( port, buffer, length, tx_confirmed, emergency, i );

        if( i < 2 )
        {
            /* Jitter reduces self-collision with queued radio work and avoids fixed burst timing. */
            uint32_t jitter_ms = smtc_modem_hal_get_random_nb_in_range( 1000, 2000 );
            hal_mcu_wait_ms( jitter_ms );
        }
    }

    crew_dr_apply_fixed( crew_dr.normal );
    return send_ok;
}

static void crew_extended_uplink_done( void )
{
}

void app_tracker_new_run( uint8_t event )
{
    event_state = event;
    if( tracker_scan_status == 0 ) // Not tracking is doing
    {
        smtc_modem_status_mask_t modem_status;
        smtc_modem_get_status( 0, &modem_status );
        if(( modem_status & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED )
        {
            smtc_modem_alarm_clear_timer( );
            app_tracker_sos_gnss_prestart( );
            smtc_modem_alarm_start_timer( 1 );
            hal_sleep_exit( );
        }
        else
        {
            event_state = 0;
            scan_result_num = 0;
            PRINTF( "\r\nNOT JOINED, SKIP NEW TRACKING\r\n" );
        }
    }
    else
    {
        PRINTF( "\r\nTRACKING IS DOING, SKIP NEW ONE\r\n" );
    }
}

void app_radio_set_sleep( void )
{
    lr11xx_system_sleep_cfg_t radio_sleep_cfg;

    radio_sleep_cfg.is_warm_start  = true;
    radio_sleep_cfg.is_rtc_timeout = false;

    if( lr11xx_system_cfg_lfclk( modem_radio->ral.context, LR11XX_SYSTEM_LFCLK_RC, true ) != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to set LF clock\n" );
    }
    if( lr11xx_system_set_sleep( modem_radio->ral.context, radio_sleep_cfg, 0 ) != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to set the radio to sleep\n" );
    }
}

void app_lora_stack_suspend( void )
{
    smtc_modem_alarm_clear_timer( );
    smtc_modem_leave_network( stack_id );
    smtc_modem_suspend_radio_communications( true );
}
/* --- EOF ------------------------------------------------------------------ */
