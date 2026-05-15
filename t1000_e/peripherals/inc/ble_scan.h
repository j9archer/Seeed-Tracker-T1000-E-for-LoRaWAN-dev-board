
#ifndef __PERIPHERAL_BLE_SCAN_H__
#define __PERIPHERAL_BLE_SCAN_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sBleBeacons
{
    uint16_t company_id;
    uint8_t uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t rssi;
    int8_t rssi_;
    uint8_t mac[8];
}BleBeacons_t;

typedef struct
{
    uint8_t uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t rssi;
    uint8_t mac[8];
} ble_beacon_hint_t;

/*!
 * @brief Init ble scan
 */
void ble_scan_init( void );

/*!
 * @brief Start ble scan
 */
bool ble_scan_start( void );

/*!
 * @brief Get ble scan results
 * 
 * @param [out] result Pointer to buffer to save results
 * @param [out] size Pointer to buffer to save results length
 */
bool ble_get_results( uint8_t *result, uint8_t *size );

/*!
 * @brief Get the strongest approved iBeacon regardless of Minor value.
 *
 * Movement detection uses this because a commissioned vessel beacon is still useful
 * as a location signal even when its Minor has not been programmed as a DR hint.
 *
 * @param [out] hint Strongest approved beacon fields
 *
 * @returns true if a beacon matching the approved UUID filter was found
 */
bool ble_get_strongest_approved_beacon( ble_beacon_hint_t *hint );

/*!
 * @brief Get the strongest approved iBeacon with a usable RF hint Minor value.
 *
 * @param [out] hint Strongest beacon fields
 *
 * @returns true if a beacon matching the UUID filter and Minor 1..5 was found
 */
bool ble_get_strongest_hint( ble_beacon_hint_t *hint );

/*!
 * @brief Stop ble scan
 */
void ble_scan_stop( void );

/*!
 * @brief Display ble scan results
 */
void ble_display_results( void );

/*
 * @brief Returns true if BLE scanning is currently active
 */
bool ble_scan_is_active( void );

#ifdef __cplusplus
}
#endif

#endif
