/*!
 * @file      marine_gnss.h
 *
 * @brief     Marine GNSS tracking profiles for MOB/PIW scenarios
 *
 * Implements quality-driven GNSS tracking profiles optimized for maritime safety.
 * Replaces the default GNSS scan in app_tracker_scan_process when BLE scan fails.
 * 
 * MOB Detection (0-5 min): Burst mode with continuous GNSS, loose accuracy
 * - GNSS on continuously for minimum TTFF (PAIR005 warm start)
 * - Double uplinks every 30 seconds (6 second gap)
 * - Each uplink includes position + HDOP
 * - Priority: Speed over accuracy
 * - NMEA debug enabled for fix quality verification
 * 
 * PIW Tracking (5 min - 2+ hours): Battery-aware, quality-driven
 * - 5-30 min: Quality scans every 30 seconds (PAIR004 hot start)
 * - 30 min - 2 hr: Quality scans every 60 seconds
 * - After 2 hr: Quality scans every 120 seconds
 * - Double uplinks continue
 * - BLE scan runs in background, interrupts GNSS if beacon found
 * - NMEA debug enabled in Phase 1 only
 */

#ifndef MARINE_GNSS_H
#define MARINE_GNSS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "ag3335.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// MOB Burst Mode timing (0-5 minutes after activation)
#define MOB_BURST_DURATION_S        ( 5 * 60 )      // 5 minutes
#define MOB_UPLINK_INTERVAL_S       30              // 30 seconds between uplink pairs
#define MOB_DOUBLE_UPLINK_GAP_S     6               // 6 seconds between double uplinks

// PIW Mode timing intervals
#define PIW_PHASE1_END_S            ( 30 * 60 )     // 30 minutes
#define PIW_PHASE2_END_S            ( 2 * 60 * 60 ) // 2 hours
#define PIW_PHASE1_INTERVAL_S       30              // 30 seconds
#define PIW_PHASE2_INTERVAL_S       60              // 60 seconds
#define PIW_PHASE3_INTERVAL_S       120             // 120 seconds

// Quality thresholds for PIW mode
#define PIW_GNSS_MAX_SCAN_MS        20000           // 20 second max scan
#define PIW_GNSS_MAX_HDOP           3.0f            // Maximum acceptable HDOP
#define PIW_GNSS_MAX_HACC_M         15.0f           // Maximum acceptable HACC (meters)

// BLE scan parameters (runs in parallel)
#define MOB_BLE_SCAN_DURATION_S     3               // BLE scan duration

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief MOB/PIW tracker operating mode
 */
typedef enum {
    MOB_MODE_IDLE,          // Not active
    MOB_MODE_BURST,         // MOB detection burst (0-5 min)
    MOB_MODE_PIW_PHASE1,    // PIW tracking phase 1 (5-30 min)
    MOB_MODE_PIW_PHASE2,    // PIW tracking phase 2 (30 min - 2 hr)
    MOB_MODE_PIW_PHASE3,    // PIW tracking phase 3 (after 2 hr)
    MOB_MODE_CANCELLED      // Cancelled by BLE detection
} mob_tracker_mode_t;

/*!
 * @brief MOB/PIW tracker state
 */
typedef struct {
    mob_tracker_mode_t  mode;           // Current operating mode
    uint32_t            activation_rtc; // RTC timestamp when MOB activated
    uint32_t            elapsed_s;      // Seconds since activation
    gnss_fix_t          last_fix;       // Last GNSS fix (may be stale)
    bool                last_fix_good;  // Was last fix quality acceptable?
    uint8_t             uplink_count;   // Uplinks sent this interval
    bool                ble_found;      // BLE beacon detected
} mob_tracker_state_t;

/*!
 * @brief Uplink payload for MOB/PIW position report
 */
typedef struct __attribute__((packed)) {
    uint8_t  data_id;       // Packet type identifier
    uint8_t  event_state;   // Event flags
    int32_t  latitude;      // Latitude * 1e6
    int32_t  longitude;     // Longitude * 1e6
    uint8_t  hdop_x10;      // HDOP * 10 (e.g., 15 = HDOP 1.5)
    uint8_t  quality_flags; // Bit 0: fix_valid, Bit 1: quality_ok
    int8_t   battery;       // Battery percentage
} mob_position_uplink_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize MOB/PIW tracker system
 */
void mob_tracker_init( void );

/*!
 * @brief Activate MOB mode (called when BLE scan fails after button press)
 *
 * Starts the MOB burst mode which will automatically transition to PIW phases.
 * 
 * @returns true if activation successful
 */
bool mob_tracker_activate( void );

/*!
 * @brief Cancel MOB/PIW tracking (called when BLE beacon found)
 *
 * Immediately stops GNSS scanning and sends cancellation uplink.
 */
void mob_tracker_cancel( void );

/*!
 * @brief Get current tracker state
 *
 * @returns Pointer to current state (read-only)
 */
const mob_tracker_state_t* mob_tracker_get_state( void );

/*!
 * @brief Check if MOB/PIW tracker is active
 *
 * @returns true if in any active mode (not IDLE or CANCELLED)
 */
bool mob_tracker_is_active( void );

/*!
 * @brief Process MOB/PIW tracker (called from main loop or timer)
 *
 * Handles state transitions, GNSS scanning, BLE checks, and uplinks.
 * Should be called at regular intervals when tracker is active.
 * 
 * @returns Next recommended callback delay in seconds
 */
uint32_t mob_tracker_process( void );

/*!
 * @brief Handle BLE scan result (callback from BLE module)
 *
 * If BLE beacon is found while MOB/PIW is active, triggers cancellation.
 * 
 * @param [in] found true if BLE beacon was found
 */
void mob_tracker_on_ble_result( bool found );

/*!
 * @brief Get mode name string for logging
 *
 * @param [in] mode Operating mode
 * @returns Human-readable mode name
 */
const char* mob_tracker_mode_str( mob_tracker_mode_t mode );

#ifdef __cplusplus
}
#endif

#endif // MARINE_GNSS_H

/* --- EOF ------------------------------------------------------------------ */
