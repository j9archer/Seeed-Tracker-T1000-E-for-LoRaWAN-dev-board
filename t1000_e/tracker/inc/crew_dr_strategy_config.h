#ifndef CREW_DR_STRATEGY_CONFIG_H
#define CREW_DR_STRATEGY_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Build-time switches for crew-tag DR policy variants.
 * These are intentionally compile-time constants so we can build different firmware versions
 * without changing the BLE configuration payload format.
 *
 * Decision summary:
 * - BLE Minor 1..5 is a local RF hint captured during the short pre-uplink scan.
 * - BLE hints provide the fast initial DR selection when moving through the vessel.
 * - LinkCheckReq provides slower network validation using demodulation margin and gateway count.
 * - TX power hinting stays disabled for now; DR-only control is the tested path.
 *
 * See docs/MDR-012-ble-hint-linkcheck-dr-strategy.md for the rationale.
 */

#define CREW_DR_BLE_HINT_ENABLE                 true
#define CREW_DR_LINKCHECK_ENABLE                true

#define CREW_DR_BLE_HINT_STABLE_LINKCHECK_S     ( 3 * 60 )
#define CREW_DR_BLE_HINT_UPGRADE_CONFIRM_SCANS  2
#define CREW_DR_BLE_HINT_MOVEMENT_CONFIRM_SCANS 2
#define CREW_DR_BLE_HINT_LINKCHECK_COOLDOWN_S   ( 10 * 60 )

#define CREW_DR_LINKCHECK_INTERVAL_NO_HINT      20
#define CREW_DR_LINKCHECK_INTERVAL_WITH_HINT    60

#define CREW_DR_LINKCHECK_LOW_MARGIN_DB         5
#define CREW_DR_LINKCHECK_HIGH_MARGIN_DB        15
/*
 * Approximate EU868 LoRa sensitivity cost per DR step around SF9..SF7 is 2.5-3 dB.
 * Use 5 dB here as a conservative whole-system allowance for antenna/body/vessel fading.
 */
#define CREW_DR_LINKCHECK_MARGIN_PER_DR_DB      5

/*
 * Approved iBeacon UUIDs for BLE RF hints.
 *
 * The config app's "Group UUID" field is treated as one additional full UUID for trials
 * or one-off installs. These build-time UUIDs are the default production whitelist.
 */
#define CREW_DR_BLE_UUID_WHITELIST_ENABLE       true
#define CREW_DR_BLE_UUID_WHITELIST_COUNT        1

static const uint8_t CREW_DR_BLE_UUID_WHITELIST[CREW_DR_BLE_UUID_WHITELIST_COUNT][16] = {
    /* RemEX / common test iBeacon UUID: FDA50693-A4E2-4FB1-AFCF-C6EB07647825 */
    { 0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1, 0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25 },
};

#endif /* CREW_DR_STRATEGY_CONFIG_H */
