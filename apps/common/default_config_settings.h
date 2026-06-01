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

#endif /* DEFAULT_CONFIG_SETTINGS_H */
