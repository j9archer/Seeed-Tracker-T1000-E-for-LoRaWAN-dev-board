
#ifndef __PERIPHERAL_WIFI_SCAN_H__
#define __PERIPHERAL_WIFI_SCAN_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "wifi_helpers_defs.h"

/*!
 * @brief Start wifi scan
 * 
 * @param [in] modem_radio Chip implementation context
 */
bool wifi_scan_start( ralf_t* modem_radio );

/*!
 * @brief Start wifi scan with an explicit LR11xx result limit
 *
 * @param [in] modem_radio Chip implementation context
 * @param [in] max_results Maximum raw results requested from LR11xx
 */
bool wifi_scan_start_with_max_results( ralf_t* modem_radio, uint8_t max_results );

/*!
 * @brief Return whether the just-completed primary channel scan should be followed by a remainder-channel scan.
 */
bool wifi_scan_should_scan_remainder_channels( void );

/*!
 * @brief Configure the next Wi-Fi scan to use the remainder channel mask.
 */
void wifi_scan_prepare_remainder_channels( void );

/*!
 * @brief Get wifi scan results
 * 
 * @param [in] modem_radio Chip implementation context
 * @param [out] result Pointer to buffer to save results
 * @param [out] size Pointer to buffer to save results length
 */
bool wifi_get_results( ralf_t* modem_radio, uint8_t *result, uint8_t *size );

/*!
 * @brief Update adaptive Wi-Fi scan effort and return whether the last filtered-only hit should be accepted provisionally.
 */
bool wifi_scan_update_adaptive_state( void );

/*!
 * @brief Return the result limit planned for the next Wi-Fi scan.
 */
uint8_t wifi_scan_get_next_max_results( void );

/*!
 * @brief Stop wifi scan
 * 
 * @param [in] modem_radio Chip implementation context
 */
void wifi_scan_stop( ralf_t* modem_radio );

/*!
 * @brief Display wifi scan results
 */
void wifi_display_results( void );

#endif
