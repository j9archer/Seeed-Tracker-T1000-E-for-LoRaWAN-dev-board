/*!
 * @file      marine_gnss.c
 *
 * @brief     Marine GNSS tracking implementation for MOB/PIW scenarios
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "marine_gnss.h"
#include "smtc_hal.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_modem_api.h"
#include "ag3335.h"
#include "sensor.h"
#include "gateway_assistance.h"
#include "app_ble_all.h"
#include "main_lorawan_tracker_api.h"
#include "log_filter.h"
#include <string.h>
#include <math.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define MOB_TRACE_INFO(...)     LOG_GNSS(__VA_ARGS__)
#define MOB_TRACE_WARNING(...)  LOG_GNSS("WARN: " __VA_ARGS__)
#define MOB_TRACE_ERROR(...)    LOG_GNSS("ERROR: " __VA_ARGS__)

extern uint32_t ble_scan_duration;

// NMEA debug macro - enabled for MOB burst and PIW Phase 1
#define MOB_NMEA_DEBUG(mode, ...)  do { \
    if ((mode) == MOB_MODE_BURST || (mode) == MOB_MODE_PIW_PHASE1) { \
        LOG_NMEA(__VA_ARGS__); \
    } \
} while(0)

// Data IDs for MOB position uplinks (reserve new IDs)
#define DATA_ID_MOB_POSITION        0x20    // MOB position with quality
#define DATA_ID_MOB_CANCELLED       0x21    // MOB cancelled (BLE found)
#define DATA_ID_MOB_NO_FIX          0x22    // MOB no fix available
#define MOB_PAYLOAD_FLAG_ON_CHARGE  0x80    // ORed into mode byte for FPort 6 MOB records
#define MOB_QUALITY_FLAG_ON_CHARGE  0x04    // Position quality flag metadata
#define MOB_CANCEL_FLAG_ON_CHARGE   0x01    // Optional cancellation flags byte metadata

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

static mob_tracker_state_t tracker_state = {
    .mode = MOB_MODE_IDLE,
    .activation_rtc = 0,
    .elapsed_s = 0,
    .last_fix_good = false,
    .uplink_count = 0,
    .ble_found = false
};

// Flag for continuous GNSS in burst mode
static bool gnss_continuous_active = false;

// Stack ID for LoRaWAN operations
static const uint8_t stack_id = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void mob_update_elapsed( void );
static mob_tracker_mode_t mob_get_mode_for_elapsed( uint32_t elapsed_s );
static uint32_t mob_get_interval_for_mode( mob_tracker_mode_t mode );
static bool mob_send_position_uplink( const gnss_fix_t *fix, bool quality_ok, bool confirmed );
static bool mob_send_position_with_policy( const gnss_fix_t *fix, bool quality_ok, bool confirmed,
                                           app_mob_dr_policy_t policy );
static void mob_send_cancellation_uplink( void );
static void mob_send_no_fix_uplink( void );
static void mob_send_no_fix_with_policy( app_mob_dr_policy_t policy );
static uint32_t mob_process_burst( void );
static uint32_t mob_process_piw( void );
static void mob_run_ble_scan( void );

static bool initial_burst_sent = false;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void mob_tracker_init( void )
{
    memset( &tracker_state, 0, sizeof( mob_tracker_state_t ));
    tracker_state.mode = MOB_MODE_IDLE;
    gnss_continuous_active = false;
    
    MOB_TRACE_INFO( "MOB/PIW tracker initialized\n" );
}

bool mob_tracker_activate( void )
{
    if( tracker_state.mode != MOB_MODE_IDLE && tracker_state.mode != MOB_MODE_CANCELLED )
    {
        MOB_TRACE_WARNING( "MOB tracker already active in mode: %s\n", 
                          mob_tracker_mode_str( tracker_state.mode ));
        return false;
    }
    
    // Initialize state
    memset( &tracker_state, 0, sizeof( mob_tracker_state_t ));
    tracker_state.mode = MOB_MODE_BURST;
    tracker_state.activation_rtc = hal_rtc_get_time_s( );
    initial_burst_sent = false;
    tracker_state.elapsed_s = 0;
    gnss_continuous_active = false;
    
    MOB_TRACE_INFO( "========================================\n" );
    MOB_TRACE_INFO( "MOB ACTIVATED - Entering BURST mode\n" );
    MOB_TRACE_INFO( "NMEA debug enabled for quality verification\n" );
    MOB_TRACE_INFO( "========================================\n" );
    
    // Enable NMEA debug output for burst mode
    gnss_enable_nmea_debug(true);
    
    // Start continuous GNSS for burst mode (if not already running in background)
    if( !gateway_assistance_is_background_gnss_active() )
    {
        gnss_scan_start( );
    }
    else
    {
        MOB_TRACE_INFO( "GNSS already running in background mode\n" );
    }
    gnss_continuous_active = true;
    
    // Inject time to GNSS (module is now powered)
    gateway_assistance_send_time_to_gnss(true);
    
    // Send warm start for MOB burst (PAIR005)
    // Warm start is best when we have almanac but may not have fresh ephemeris
    if( gateway_assistance_is_gnss_ready( ))
    {
        gateway_assistance_send_warm_start( );
        MOB_TRACE_INFO( "GNSS warm start (PAIR005) sent for MOB burst\n" );
    }
    
    return true;
}

void mob_tracker_cancel( void )
{
    MOB_TRACE_INFO( "========================================\n" );
    MOB_TRACE_INFO( "MOB CANCELLED - BLE beacon detected\n" );
    MOB_TRACE_INFO( "========================================\n" );
    
    // Disable NMEA debug
    gnss_enable_nmea_debug(false);
    
    // Stop GNSS if running (but not if background mode is active)
    if( gnss_continuous_active && !gateway_assistance_is_background_gnss_active() )
    {
        gnss_scan_stop( );
    }
    gnss_continuous_active = false;
    
    // Send cancellation uplink
    mob_send_cancellation_uplink( );
    
    // Update state
    tracker_state.mode = MOB_MODE_CANCELLED;
    tracker_state.ble_found = true;
}

const mob_tracker_state_t* mob_tracker_get_state( void )
{
    return &tracker_state;
}

bool mob_tracker_is_active( void )
{
    return ( tracker_state.mode != MOB_MODE_IDLE && 
             tracker_state.mode != MOB_MODE_CANCELLED );
}

uint32_t mob_tracker_process( void )
{
    if( !mob_tracker_is_active( ))
    {
        return 60; // Check again in 60 seconds if somehow called while idle
    }
    
    // Update elapsed time
    mob_update_elapsed( );
    
    // Check for mode transition
    mob_tracker_mode_t new_mode = mob_get_mode_for_elapsed( tracker_state.elapsed_s );
    if( new_mode != tracker_state.mode )
    {
        MOB_TRACE_INFO( "MOB mode transition: %s -> %s (elapsed: %lu s)\n",
                       mob_tracker_mode_str( tracker_state.mode ),
                       mob_tracker_mode_str( new_mode ),
                       tracker_state.elapsed_s );
        
        // Handle transition from burst to PIW
        if( tracker_state.mode == MOB_MODE_BURST && new_mode != MOB_MODE_BURST )
        {
            // Stop continuous GNSS - PIW uses quality-driven scans
            if( gnss_continuous_active )
            {
                gnss_scan_stop( );
                gnss_continuous_active = false;
            }
        }
        
        // Handle NMEA debug: enable for burst and phase1, disable for phase2+
        if( new_mode == MOB_MODE_BURST || new_mode == MOB_MODE_PIW_PHASE1 )
        {
            gnss_enable_nmea_debug(true);
            MOB_TRACE_INFO( "NMEA debug enabled for %s\n", mob_tracker_mode_str(new_mode) );
        }
        else if( tracker_state.mode == MOB_MODE_PIW_PHASE1 && new_mode == MOB_MODE_PIW_PHASE2 )
        {
            gnss_enable_nmea_debug(false);
            MOB_TRACE_INFO( "NMEA debug disabled for battery conservation\n" );
        }
        
        tracker_state.mode = new_mode;
    }
    
    // Run BLE scan in background (all modes)
    mob_run_ble_scan( );
    
    // Check if BLE found (could be set during scan)
    if( tracker_state.ble_found )
    {
        mob_tracker_cancel( );
        return 60;
    }
    
    // Process based on current mode
    uint32_t next_delay;
    if( tracker_state.mode == MOB_MODE_BURST )
    {
        next_delay = mob_process_burst( );
    }
    else
    {
        next_delay = mob_process_piw( );
    }
    
    return next_delay;
}

void mob_tracker_on_ble_result( bool found )
{
    if( found && mob_tracker_is_active( ))
    {
        tracker_state.ble_found = true;
        gnss_set_ble_found( true );  // Signal GNSS to interrupt
        MOB_TRACE_INFO( "BLE result: beacon FOUND - flagging for cancellation\n" );
    }
}

const char* mob_tracker_mode_str( mob_tracker_mode_t mode )
{
    switch( mode )
    {
        case MOB_MODE_IDLE:       return "IDLE";
        case MOB_MODE_BURST:      return "BURST";
        case MOB_MODE_PIW_PHASE1: return "PIW_PHASE1";
        case MOB_MODE_PIW_PHASE2: return "PIW_PHASE2";
        case MOB_MODE_PIW_PHASE3: return "PIW_PHASE3";
        case MOB_MODE_CANCELLED:  return "CANCELLED";
        default:                  return "UNKNOWN";
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void mob_update_elapsed( void )
{
    uint32_t current_rtc = hal_rtc_get_time_s( );
    tracker_state.elapsed_s = current_rtc - tracker_state.activation_rtc;
}

static mob_tracker_mode_t mob_get_mode_for_elapsed( uint32_t elapsed_s )
{
    if( elapsed_s < MOB_BURST_DURATION_S )
    {
        return MOB_MODE_BURST;
    }
    else if( elapsed_s < PIW_PHASE1_END_S )
    {
        return MOB_MODE_PIW_PHASE1;
    }
    else if( elapsed_s < PIW_PHASE2_END_S )
    {
        return MOB_MODE_PIW_PHASE2;
    }
    else
    {
        return MOB_MODE_PIW_PHASE3;
    }
}

static uint32_t mob_get_interval_for_mode( mob_tracker_mode_t mode )
{
    switch( mode )
    {
        case MOB_MODE_BURST:      return MOB_UPLINK_INTERVAL_S;
        case MOB_MODE_PIW_PHASE1: return PIW_PHASE1_INTERVAL_S;
        case MOB_MODE_PIW_PHASE2: return PIW_PHASE2_INTERVAL_S;
        case MOB_MODE_PIW_PHASE3: return PIW_PHASE3_INTERVAL_S;
        default:                  return 60;
    }
}

static bool mob_send_position_uplink( const gnss_fix_t *fix, bool quality_ok, bool confirmed )
{
    app_mob_dr_policy_t policy = APP_MOB_DR_PERSISTENCE;

    /* BURST favors temporal density at max DR; PHASE3 mostly persists but periodically probes minimum DR. */
    if( tracker_state.mode == MOB_MODE_BURST )
    {
        policy = APP_MOB_DR_MAX;
    }
    else if( tracker_state.mode == MOB_MODE_PIW_PHASE3 )
    {
        policy = APP_MOB_DR_PHASE3_ALTERNATING;
    }

    return mob_send_position_with_policy( fix, quality_ok, confirmed, policy );
}

static bool mob_send_position_with_policy( const gnss_fix_t *fix, bool quality_ok, bool confirmed,
                                           app_mob_dr_policy_t policy )
{
    mob_position_uplink_t payload;
    bool on_charge = gateway_assistance_is_charging( );
    memset( &payload, 0, sizeof( payload ));
    
    payload.data_id = DATA_ID_MOB_POSITION;
    payload.event_state = (uint8_t)tracker_state.mode;
    if( on_charge )
    {
        payload.event_state |= MOB_PAYLOAD_FLAG_ON_CHARGE;
    }
    payload.latitude = fix->latitude;
    payload.longitude = fix->longitude;
    payload.hdop_x10 = (uint8_t)( fix->hdop * 10 );
    if( payload.hdop_x10 > 255 ) payload.hdop_x10 = 255;
    
    payload.quality_flags = 0;
    if( fix->valid ) payload.quality_flags |= 0x01;
    if( quality_ok ) payload.quality_flags |= 0x02;
    if( on_charge ) payload.quality_flags |= MOB_QUALITY_FLAG_ON_CHARGE;
    
    payload.battery = sensor_bat_sample( );
    
    MOB_TRACE_INFO( "MOB uplink: lat=%ld, lon=%ld, HDOP=%.1f, qual=%02X, batt=%d%%, on_charge=%u\n",
                   payload.latitude, payload.longitude, 
                   fix->hdop, payload.quality_flags, payload.battery, on_charge ? 1 : 0 );
    
    if( tracker_state.mode == MOB_MODE_BURST && initial_burst_sent == false )
    {
        /* First MOB location sends the same payload across three DRs before normal 30s max-DR tracking resumes. */
        initial_burst_sent = true;
        return app_send_mob_initial_burst( (uint8_t*)&payload, sizeof( payload ), confirmed );
    }

    return app_send_mob_frame( (uint8_t*)&payload, sizeof( payload ), confirmed, policy );
}

static void mob_send_cancellation_uplink( void )
{
    uint8_t payload[5];
    bool on_charge = gateway_assistance_is_charging( );

    payload[0] = DATA_ID_MOB_CANCELLED;
    payload[1] = (uint8_t)( tracker_state.elapsed_s >> 8 );
    payload[2] = (uint8_t)( tracker_state.elapsed_s & 0xFF );
    payload[3] = sensor_bat_sample( );
    payload[4] = on_charge ? MOB_CANCEL_FLAG_ON_CHARGE : 0x00;
    
    MOB_TRACE_INFO( "MOB cancellation uplink: elapsed=%lu s, batt=%d%%, on_charge=%u\n",
                   tracker_state.elapsed_s, payload[3], on_charge ? 1 : 0 );
    
    app_send_mob_frame( payload, sizeof( payload ), true, APP_MOB_DR_PERSISTENCE );
}

static void mob_send_no_fix_uplink( void )
{
    app_mob_dr_policy_t policy = APP_MOB_DR_PERSISTENCE;

    /* No-fix reports follow the same DR strategy as position reports so the state timeline stays visible. */
    if( tracker_state.mode == MOB_MODE_BURST )
    {
        policy = APP_MOB_DR_MAX;
    }
    else if( tracker_state.mode == MOB_MODE_PIW_PHASE3 )
    {
        policy = APP_MOB_DR_PHASE3_ALTERNATING;
    }

    mob_send_no_fix_with_policy( policy );
}

static void mob_send_no_fix_with_policy( app_mob_dr_policy_t policy )
{
    uint8_t payload[5];
    bool on_charge = gateway_assistance_is_charging( );

    payload[0] = DATA_ID_MOB_NO_FIX;
    payload[1] = (uint8_t)tracker_state.mode;
    if( on_charge )
    {
        payload[1] |= MOB_PAYLOAD_FLAG_ON_CHARGE;
    }
    payload[2] = (uint8_t)( tracker_state.elapsed_s >> 8 );
    payload[3] = (uint8_t)( tracker_state.elapsed_s & 0xFF );
    payload[4] = sensor_bat_sample( );
    
    MOB_TRACE_INFO( "MOB no-fix uplink: mode=%s, elapsed=%lu s, batt=%d%%, on_charge=%u\n",
                   mob_tracker_mode_str( tracker_state.mode ),
                   tracker_state.elapsed_s, payload[4], on_charge ? 1 : 0 );
    
    if( tracker_state.mode == MOB_MODE_BURST && initial_burst_sent == false )
    {
        /* If GNSS has no fix yet, still send the initial DR burst so the MOB state is announced immediately. */
        initial_burst_sent = true;
        app_send_mob_initial_burst( payload, sizeof( payload ), false );
        return;
    }

    app_send_mob_frame( payload, sizeof( payload ), false, policy );
}

static uint32_t mob_process_burst( void )
{
    // In burst mode, GNSS runs continuously
    // We check for fix and send double uplinks every 30 seconds
    
    gnss_fix_t fix;
    bool got_fix = gnss_get_quality_fix( &fix );
    
    if( got_fix )
    {
        tracker_state.last_fix = fix;
        tracker_state.last_fix_good = true;  // In burst mode, any fix is acceptable
        
        // Send double uplink (first one)
        mob_send_position_uplink( &fix, true, false );
        tracker_state.uplink_count++;
        
        // Wait 6 seconds
        hal_mcu_wait_ms( MOB_DOUBLE_UPLINK_GAP_S * 1000 );
        
        // Check for BLE interrupt during wait
        if( tracker_state.ble_found )
        {
            return 1; // Cancel will be handled in next process call
        }
        
        // Get fresh fix for second uplink
        gnss_get_quality_fix( &fix );
        mob_send_position_uplink( &fix, true, false );
        tracker_state.uplink_count++;
    }
    else
    {
        // No fix yet - send no-fix packet
        mob_send_no_fix_uplink( );
    }
    
    // Next callback in 30 seconds (minus the 6-second gap we already waited)
    return MOB_UPLINK_INTERVAL_S - MOB_DOUBLE_UPLINK_GAP_S;
}

static uint32_t mob_process_piw( void )
{
    gnss_fix_t fix;
    bool got_good_fix;

    // Check if background GNSS is already active (charging mode)
    bool background_active = gateway_assistance_is_background_gnss_active();

    if( background_active )
    {
        MOB_TRACE_INFO( "GNSS already running in background mode\n" );
    }

    // Send time to GNSS (module is now powered - either background or will be started by scan)
    gateway_assistance_send_time_to_gnss(true);
    
    // Send hot start for PIW phases (PAIR004)
    // Hot start uses ephemeris if available from recent fix
    if( gateway_assistance_is_gnss_ready( ))
    {
        gateway_assistance_send_hot_start( );
        MOB_TRACE_INFO( "GNSS hot start (PAIR004) sent for PIW\n" );
    }
    
    // Log NMEA debug info for Phase 1
    MOB_NMEA_DEBUG(tracker_state.mode, "[PIW Phase 1] Starting quality scan\n");
    
    // Quality-driven scan with early exit
    // When background_active=true, skip power management to keep GNSS running
    got_good_fix = gnss_scan_until_good( 
        PIW_GNSS_MAX_SCAN_MS,
        PIW_GNSS_MAX_HDOP,
        PIW_GNSS_MAX_HACC_M,
        &fix,
        background_active );  // Skip power management if background GNSS active
    
    // No need to manually stop GNSS - gnss_scan_until_good handles it based on skip_power_management
    
    // Check for BLE interrupt during scan
    if( tracker_state.ble_found )
    {
        return 1; // Cancel will be handled
    }
    
    if( fix.valid )
    {
        tracker_state.last_fix = fix;
        tracker_state.last_fix_good = got_good_fix;
        
        // Store fix for assistance
        gateway_assistance_store_own_fix( fix.latitude, fix.longitude );
        
        // Log fix quality for debug
        MOB_NMEA_DEBUG(tracker_state.mode, 
            "[PIW] Fix: lat=%ld, lon=%ld, HDOP=%.1f, HACC=%.1fm, sats=%d, quality=%s\n",
            fix.latitude, fix.longitude, fix.hdop, fix.hacc, 
            fix.satellites, got_good_fix ? "GOOD" : "MARGINAL");
        
        // Send double uplink
        mob_send_position_uplink( &fix, got_good_fix, false );
        
        hal_mcu_wait_ms( MOB_DOUBLE_UPLINK_GAP_S * 1000 );
        
        if( !tracker_state.ble_found )
        {
            mob_send_position_uplink( &fix, got_good_fix, false );
        }
    }
    else
    {
        // No fix - send last known position with BAD flag or no-fix packet
        if( tracker_state.last_fix.valid )
        {
            mob_send_position_uplink( &tracker_state.last_fix, false, false );
        }
        else
        {
            mob_send_no_fix_uplink( );
        }
    }
    
    // Return interval for current mode
    return mob_get_interval_for_mode( tracker_state.mode );
}

static void mob_run_ble_scan( void )
{
    uint32_t scan_duration_s = ( ble_scan_duration > 0 ) ? ble_scan_duration : MOB_BLE_SCAN_DURATION_S;

    // Start BLE scan
    ble_scan_start( );
    
    // Wait for scan duration
    MOB_TRACE_INFO( "MOB BLE scan duration %lu s\n", scan_duration_s );
    hal_mcu_wait_ms( scan_duration_s * 1000 );
    
    // Stop scan and check results
    ble_scan_stop( );
    
    uint8_t ble_data[64];
    uint8_t ble_len = 0;
    ble_get_results( ble_data, &ble_len );
    
    if( ble_len > 0 )
    {
        // BLE beacon found!
        MOB_TRACE_INFO( "BLE scan found beacon (%d bytes)\n", ble_len );
        tracker_state.ble_found = true;
        gnss_set_ble_found( true );
    }
}

/* --- EOF ------------------------------------------------------------------ */
