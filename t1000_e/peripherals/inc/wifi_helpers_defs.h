#ifndef __WIFI_HELPERS_DEFS_H__
#define __WIFI_HELPERS_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include "lr11xx_wifi.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief The maximal time to spend in preamble detection for each single scan, in ms
 */
#define WIFI_TIMEOUT_PER_SCAN_DEFAULT ( 90 )

/*!
 * @brief The time to spend scanning one channel, in ms
 */
#define WIFI_TIMEOUT_PER_CHANNEL_DEFAULT ( 300 )

/*!
 * @brief The maximal number of results to gather. Maximum value is 32
 */
#define WIFI_MAX_RESULTS ( 5 )

/*!
 * @brief Number of Wi-Fi results requested from the LR11xx scan.
 */
#define WIFI_SCAN_TARGET_RESULTS ( 1 )

/*!
 * @brief Maximum adaptive Wi-Fi scan effort before falling through to other geolocation methods.
 */
#define WIFI_SCAN_ADAPTIVE_MAX_RESULTS ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Structure representing the configuration of Wi-Fi scan
 */
typedef struct
{
    lr11xx_wifi_channel_mask_t     channels;             //!< A mask of the channels to be scanned
    lr11xx_wifi_signal_type_scan_t types;                //!< Wi-Fi types to be scanned
    uint8_t                        max_results;          //!< Maximum number of results expected for a scan
    uint32_t                       timeout_per_channel;  //!< Time to spend scanning one channel, in ms
    uint32_t timeout_per_scan;  //!< Maximal time to spend in preamble detection for each single scan, in ms
} wifi_settings_t;

/*!
 * @brief Structure representing a single scan result
 */
typedef struct
{
    lr11xx_wifi_mac_address_t        mac_address;  //!< MAC address of the Wi-Fi access point which has been detected
    lr11xx_wifi_channel_t            channel;      //!< Channel on which the access point has been detected
    lr11xx_wifi_signal_type_result_t type;         //!< Type of Wi-Fi which has been detected
    int8_t                           rssi;         //!< Strength of the detected signal
    bool                             rssi_validity;
    lr11xx_wifi_mac_origin_t         origin;
} wifi_scan_single_result_t;

/*!
 * @brief Structure representing a collection of scan results
 */
typedef struct
{
    uint8_t                   nbr_results;                //!< Number of results
    uint8_t                   raw_results;                //!< Number of raw results returned by LR11xx
    uint8_t                   mobile_ap_results;          //!< Number of raw results filtered as mobile APs
    uint8_t                   local_admin_results;        //!< Number of locally administered MACs filtered locally
    uint8_t                   whitelisted_results;        //!< Number of raw results accepted by fixed AP whitelist
    uint8_t                   duplicate_results;          //!< Number of duplicate MACs filtered locally
    uint8_t                   unknown_origin_results;     //!< Number of accepted results with unknown AP origin
    uint32_t                  power_consumption_uah;      //!< Power consumption to acquire this set of results
    uint32_t                  power_consumption_nah;      //!< Power consumption to acquire this set of results, in nAh
    uint32_t                  rx_detection_us;            //!< Time spent during NFE or TOA
    uint32_t                  rx_correlation_us;          //!< Time spent during preamble detection
    uint32_t                  rx_capture_us;              //!< Time spent during signal acquisition
    uint32_t                  demodulation_us;            //!< Time spent during software demodulation
    uint32_t                  timestamp;                  //!< Timestamp at which the data set has been completed
    wifi_scan_single_result_t results[WIFI_MAX_RESULTS];  //!< Buffer containing the results
} wifi_scan_all_result_t;

#ifdef __cplusplus
}
#endif

#endif
