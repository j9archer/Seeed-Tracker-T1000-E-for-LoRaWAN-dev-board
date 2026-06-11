#ifndef DEFAULT_CONFIG_SETTINGS_H
#define DEFAULT_CONFIG_SETTINGS_H

#include <stdbool.h>

#include "app_config_param.h"

/*
 * RemEX Crew Tag first-boot defaults.
 *
 * These values are applied once when RemEX firmware first sees an older saved
 * config record. A normal firmware flash does not erase FDS, so this migration
 * makes the fleet build deterministic without requiring a visit to the config
 * app. Later user changes remain persistent until this version is increased.
 */
#define REMEX_CREW_CONFIG_VERSION              2

#define REMEX_DEFAULT_PLATFORM                 IOT_PLATFORM_OTHER
#define REMEX_DEFAULT_ACTIVE_REGION            LORAMAC_REGION_EU868
#define REMEX_DEFAULT_CHANNEL_GROUP            1
#define REMEX_DEFAULT_ACTIVATION_TYPE          ACTIVATION_TYPE_ABP
#define REMEX_DEFAULT_RETRY                    RETRY_STATE_1N

/*
 * Deterministic ABP derivation inputs.
 *
 * DevEUI is always read from the device. These build-time values define the
 * fleet secret and client/project DevAddr allocation used to derive DevAddr,
 * NwkSKey, and AppSKey.
 */
#define REMEX_ABP_MASTER_KEY                   "00007955000980007000400600002000"
#define REMEX_ABP_NET_ID_START                 "E00110"
#define REMEX_ABP_NET_ID_END                   "E00110"
#define REMEX_ABP_GROUP1_BLOCK_SIZE            32

#define REMEX_DEFAULT_LORAWAN_ADR_ENABLED      false
#define REMEX_DEFAULT_LORAWAN_DR_MIN           1
#define REMEX_DEFAULT_LORAWAN_DR_MAX           5

#define REMEX_DEFAULT_ACCELEROMETER_ENABLED    false
#define REMEX_DEFAULT_SOS_MODE                 1
#define REMEX_DEFAULT_UPLINK_INTERVAL_MIN      1
#define REMEX_DEFAULT_POSITION_STRATEGY        6
#define REMEX_DEFAULT_GNSS_MAX_SCAN_TIME_S     30
#define REMEX_DEFAULT_IBEACON_SCAN_MAX         3
#define REMEX_DEFAULT_IBEACON_SCAN_TIMEOUT_S   6
#define REMEX_DEFAULT_GROUP_UUID_LEN           0

/*
 * PIW drift-vector estimation knobs.
 *
 * COG/SOG should be estimated from a short history of noisy GNSS fixes rather
 * than from instantaneous receiver COG or from only the last two points. These
 * values tune the firmware's robust weighted line fit:
 *
 * - FIX_WINDOW: number of valid PIW fixes retained in the sliding window.
 * - MIN_FIXES: minimum fixes after outlier rejection before a vector is emitted.
 * - MIN_BASELINE_S: minimum elapsed time covered by retained fixes.
 * - DUPLICATE_FIX_S: ignore repeat samples with identical coordinates inside
 *   this interval. This prevents PIW double-uplinks from being counted as two
 *   independent GNSS fixes.
 * - MIN_TRACK_SPAN_M: minimum fitted movement over that baseline. This keeps
 *   COG from becoming authoritative when the fitted motion is still dominated
 *   by position noise.
 * - UERE_M: fallback user-equivalent range error. If GST/HACC is unavailable,
 *   sigma is estimated as HDOP * UERE.
 * - SIGMA_FLOOR/CEILING_M: bounds for each fix's sigma. Poor fixes remain in
 *   the fit but receive lower weight; the ceiling prevents them becoming zero
 *   weight.
 * - OUTLIER_SIGMA/MARGIN_M: track-residual gate used in the robust second pass.
 *   A fix is rejected from the final fit only when its residual from the
 *   provisional fitted track is greater than sigma * OUTLIER_SIGMA + MARGIN.
 * - MAX_TRACK_SPEED_MPS: final sanity limit for PIW drift.
 */
#define REMEX_PIW_DRIFT_FIX_WINDOW             10
#define REMEX_PIW_DRIFT_MIN_FIXES              3
#define REMEX_PIW_DRIFT_MIN_BASELINE_S         90
#define REMEX_PIW_DRIFT_DUPLICATE_FIX_S        15
#define REMEX_PIW_DRIFT_MIN_TRACK_SPAN_M       25.0f
#define REMEX_PIW_DRIFT_UERE_M                 5.0f
#define REMEX_PIW_DRIFT_SIGMA_FLOOR_M          5.0f
#define REMEX_PIW_DRIFT_SIGMA_CEILING_M        80.0f
#define REMEX_PIW_DRIFT_OUTLIER_SIGMA          3.0f
#define REMEX_PIW_DRIFT_OUTLIER_MARGIN_M       15.0f
#define REMEX_PIW_DRIFT_MAX_TRACK_SPEED_MPS    5.0f

#endif /* DEFAULT_CONFIG_SETTINGS_H */
