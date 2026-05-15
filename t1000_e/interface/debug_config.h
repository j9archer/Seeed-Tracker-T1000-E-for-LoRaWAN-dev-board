/**
 * @file debug_config.h
 * @brief Debug output configuration for T1000-E tracker
 * 
 * This file controls which debug output categories are enabled.
 * Set to 1 to enable, 0 to disable.
 * 
 * After changing, rebuild the firmware.
 */

#ifndef DEBUG_CONFIG_H
#define DEBUG_CONFIG_H

/*
 * -----------------------------------------------------------------------------
 * LoRa Modem Core Debug (radio planner, TX/RX events)
 * -----------------------------------------------------------------------------
 * Controls the verbose LoRa modem messages like:
 *   - "Send Payload HOOK ID = X"
 *   - "TX DONE"
 *   - "RX1 Timeout / RX2 Timeout"
 *   - "RP: Task #X enqueue..."
 *   - "Open RX1/RX2 for Hook Id..."
 * 
 * Set to 0 to reduce log noise when debugging GNSS or other subsystems.
 */
#define DEBUG_LORA_MODEM_VERBOSE    1

/*
 * -----------------------------------------------------------------------------
 * LoRa Application Events
 * -----------------------------------------------------------------------------
 * Controls application-level LoRa event logging like:
 *   - "###### ===== TX DONE EVENT ==== ######"
 *   - "Uplink count: X"
 *   - "Downlink received..."
 */
#define DEBUG_LORA_APP_EVENTS       1

/*
 * -----------------------------------------------------------------------------
 * GNSS Debug
 * -----------------------------------------------------------------------------
 * Controls GNSS-related debug output:
 *   - AG3335 command/response logging
 *   - Almanac status
 *   - Fix quality information
 */
#define DEBUG_GNSS                  1

/*
 * -----------------------------------------------------------------------------
 * Gateway Assistance Debug
 * -----------------------------------------------------------------------------
 * Controls gateway assistance and position injection logging
 */
#define DEBUG_GATEWAY_ASSISTANCE    1

/*
 * -----------------------------------------------------------------------------
 * Marine/MOB Debug
 * -----------------------------------------------------------------------------
 * Controls MOB/PIW marine tracking debug output
 */
#define DEBUG_MARINE                1

#endif /* DEBUG_CONFIG_H */
