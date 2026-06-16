#ifndef CREW_PAYLOAD_SCHEMA_H
#define CREW_PAYLOAD_SCHEMA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Compact crew-tag uplink payload schemas from MDR-022.
 */

#define CREW_PAYLOAD_SCHEMA_VERSION              1U
#define CREW_PAYLOAD_SCHEMA_PHASE( version, phase ) \
    ( ( uint8_t )( ( ( ( version ) & 0x0FU ) << 4 ) | \
                   ( ( phase ) & 0x0FU ) ) )
#define CREW_PAYLOAD_SCHEMA_VERSION_GET( byte )  ( ( uint8_t )( ( ( byte ) >> 4 ) & 0x0FU ) )
#define CREW_PAYLOAD_PHASE_GET( byte )           ( ( uint8_t )( ( byte ) & 0x0FU ) )

typedef enum
{
    CREW_PRESENCE_PHASE_ROUTINE = 0,
    CREW_PRESENCE_PHASE_ONBOARD = 1,
    CREW_PRESENCE_PHASE_UNCERTAIN = 2,
    CREW_PRESENCE_PHASE_MOB = 3,
    CREW_PRESENCE_PHASE_PIW = 4,
    CREW_PRESENCE_PHASE_SOS = 5,
} crew_presence_phase_t;

typedef enum
{
    CREW_SOURCE_NONE = 0,
    CREW_SOURCE_BLE = 1,
    CREW_SOURCE_WIFI_FIXED = 2,
    CREW_SOURCE_WIFI_PROVISIONAL = 3,
    CREW_SOURCE_GNSS_OR_ASSISTANCE = 4,
} crew_source_type_t;

#define CREW_SOURCE_TYPE_MASK                    0x07U
#define CREW_SOURCE_FLAG_ACCEPTED                0x08U
#define CREW_SOURCE_FLAG_RSSI_PRESENT            0x10U
#define CREW_SOURCE_FLAG_IDENTITY_PRESENT        0x20U

typedef struct __attribute__( ( packed ) )
{
    uint8_t schema_phase;
    uint8_t event_flags;
    uint8_t battery;
    uint8_t location_age_min;
    uint8_t source_flags;
} crew_presence_compact_t;

typedef struct __attribute__( ( packed ) )
{
    uint16_t major;
    uint16_t minor;
    int8_t   rssi_dbm;
} crew_presence_ble_ext_t;

typedef struct __attribute__( ( packed ) )
{
    uint8_t bssid[6];
    int8_t  rssi_dbm;
    uint8_t wifi_flags;
} crew_presence_wifi_ext_t;

typedef enum
{
    CREW_HEALTH_EVENT_FAMILY_BATTERY = 1,
    CREW_HEALTH_EVENT_FAMILY_LOW_BATTERY = 2,
    CREW_HEALTH_EVENT_FAMILY_SHOCK = 3,
    CREW_HEALTH_EVENT_FAMILY_LIGHT = 4,
    CREW_HEALTH_EVENT_FAMILY_TEMPERATURE = 5,
    CREW_HEALTH_EVENT_FAMILY_FCNT_DOWN_SYNC = 6,
} crew_health_event_family_t;

typedef struct __attribute__( ( packed ) )
{
    uint8_t schema_family;
    uint8_t event_flags;
    uint8_t battery;
    uint8_t value_1;
    uint8_t value_2;
} crew_health_event_t;

#define CREW_FCNT_DOWN_SYNC_FLAG_PENDING         0x01U
#define CREW_FCNT_DOWN_SYNC_FLAG_STALE_SEEN      0x02U

typedef struct __attribute__( ( packed ) )
{
    uint8_t  schema_family;
    uint8_t  flags;
    uint8_t  battery;
    uint32_t fcnt_down;
} crew_fcnt_down_sync_t;

#define CREW_ALERT_SUBTYPE_MOB_POSITION          0x20U
#define CREW_ALERT_SUBTYPE_MOB_CANCELLED         0x21U
#define CREW_ALERT_SUBTYPE_MOB_NO_FIX            0x22U
#define CREW_ALERT_SUBTYPE_SOS_CONTEXT           0x23U
#define CREW_ALERT_SUBTYPE_GNSS_PROOF            0x24U
#define CREW_ALERT_SUBTYPE_UNCERTAIN_COMPACT     0x30U

typedef struct __attribute__( ( packed ) )
{
    uint8_t subtype;
    uint8_t schema_phase;
    uint8_t event_flags;
    uint8_t battery;
    uint8_t location_age_min;
    uint8_t attempt_flags;
    uint8_t reason;
} crew_uncertain_compact_t;

#define CREW_UNCERTAIN_ATTEMPT_BLE               0x01U
#define CREW_UNCERTAIN_ATTEMPT_WIFI              0x02U
#define CREW_UNCERTAIN_ATTEMPT_GNSS              0x04U
#define CREW_UNCERTAIN_LOCAL_RF_FAILED           0x08U
#define CREW_UNCERTAIN_GNSS_FAILED               0x10U

typedef enum
{
    CREW_UNCERTAIN_REASON_NONE = 0,
    CREW_UNCERTAIN_REASON_NO_APPROVED_BLE = 1,
    CREW_UNCERTAIN_REASON_WIFI_REJECTED_AT_CAP = 2,
    CREW_UNCERTAIN_REASON_GNSS_UNAVAILABLE = 3,
    CREW_UNCERTAIN_REASON_TIMEOUT = 4,
} crew_uncertain_reason_t;

#define CREW_SOS_EVIDENCE_BLE_ATTEMPTED          0x01U
#define CREW_SOS_EVIDENCE_WIFI_ATTEMPTED         0x02U
#define CREW_SOS_EVIDENCE_GNSS_ATTEMPTED         0x04U
#define CREW_SOS_EVIDENCE_LOCAL_ACCEPTED         0x08U
#define CREW_SOS_EVIDENCE_GNSS_FIX_VALID         0x10U
#define CREW_SOS_EVIDENCE_GNSS_QUALITY_OK        0x20U
#define CREW_SOS_EVIDENCE_NO_FIX_YET             0x40U

typedef enum
{
    CREW_SOS_REASON_NONE = 0,
    CREW_SOS_REASON_NO_APPROVED_BLE = 1,
    CREW_SOS_REASON_WIFI_PROVISIONAL_REJECTED = 2,
    CREW_SOS_REASON_GNSS_UNAVAILABLE = 3,
    CREW_SOS_REASON_GNSS_TIMEOUT = 4,
    CREW_SOS_REASON_GNSS_NO_FIX_YET = 5,
} crew_sos_reason_t;

typedef struct __attribute__( ( packed ) )
{
    uint8_t subtype;
    uint8_t schema_phase;
    uint8_t event_flags;
    uint8_t battery;
    uint8_t local_source_flags;
    uint8_t evidence_flags;
    uint8_t reason;
} crew_sos_context_t;

typedef struct __attribute__( ( packed ) )
{
    uint16_t major;
    uint16_t minor;
    int8_t   rssi_dbm;
} crew_sos_ble_ext_t;

typedef struct __attribute__( ( packed ) )
{
    uint8_t bssid[6];
    uint8_t wifi_flags;
    int8_t  rssi_dbm;
} crew_sos_wifi_ext_t;

typedef struct __attribute__( ( packed ) )
{
    int32_t latitude;
    int32_t longitude;
    uint8_t hdop_x10;
    uint8_t satellites;
} crew_sos_gnss_ext_t;

#define CREW_GNSS_PROOF_QUALITY_FIX_VALID        0x01U
#define CREW_GNSS_PROOF_QUALITY_OK               0x02U
#define CREW_GNSS_PROOF_QUALITY_ON_CHARGE        0x04U

typedef struct __attribute__( ( packed ) )
{
    uint8_t subtype;
    uint8_t schema_phase;
    uint8_t event_flags;
    uint8_t battery;
    uint8_t location_age_min;
    uint8_t quality_flags;
    uint8_t hdop_x10;
    uint8_t satellites;
    int32_t latitude;
    int32_t longitude;
} crew_gnss_proof_t;

#define CREW_MOB_QUALITY_FLAG_FIX_VALID          0x01U
#define CREW_MOB_QUALITY_FLAG_QUALITY_OK         0x02U
#define CREW_MOB_QUALITY_FLAG_ON_CHARGE          0x04U
#define CREW_MOB_QUALITY_FLAG_VECTOR             0x08U

#define CREW_MOB_COG_X2_UNKNOWN                  0xFFFFU
#define CREW_MOB_SOG_DMPS_SATURATED              0xFEU
#define CREW_MOB_SOG_DMPS_UNKNOWN                0xFFU

typedef struct __attribute__( ( packed ) )
{
    uint16_t cog_x2;
    uint8_t  sog_dmps;
} crew_mob_movement_ext_t;

typedef struct __attribute__( ( packed ) )
{
    uint8_t schema_phase;
    uint8_t event_flags;
    uint8_t battery;
    uint8_t location_age_min;
    uint8_t record_count;
} crew_rf_fingerprint_t;

#define CREW_RF_RECORD_TYPE_BLE_IBEACON          0x01U
#define CREW_RF_RECORD_TYPE_WIFI                 0x02U

typedef struct __attribute__( ( packed ) )
{
    uint8_t  record_type;
    uint16_t major;
    uint16_t minor;
    int8_t   rssi_dbm;
} crew_rf_ble_record_t;

typedef struct __attribute__( ( packed ) )
{
    uint8_t record_type;
    uint8_t bssid[6];
    int8_t  rssi_dbm;
    uint8_t flags;
} crew_rf_wifi_record_t;

#ifdef __cplusplus
}
#endif

#endif /* CREW_PAYLOAD_SCHEMA_H */
