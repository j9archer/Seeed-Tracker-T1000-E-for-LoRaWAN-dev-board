#ifndef MAIN_LORAWAN_TRACKER_API_H
#define MAIN_LORAWAN_TRACKER_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Minimal API surface used by tracker modules without depending on SES app path */

typedef enum
{
    /* MOB/PIW chooses intent here; the LoRa app maps it to the configured region/user DR window. */
    APP_MOB_DR_MAX = 0,
    APP_MOB_DR_PERSISTENCE,
    APP_MOB_DR_MINIMUM,
    APP_MOB_DR_PHASE3_ALTERNATING
} app_mob_dr_policy_t;

bool app_send_frame( const uint8_t* buffer, const uint8_t length, bool tx_confirmed, bool emergency );

bool app_send_mob_frame( const uint8_t* buffer, const uint8_t length, bool tx_confirmed, app_mob_dr_policy_t policy );

bool app_send_mob_initial_burst( const uint8_t* buffer, const uint8_t length, bool tx_confirmed );

void app_tracker_new_run( uint8_t event );

void app_radio_set_sleep( void );

void app_lora_stack_suspend( void );

#ifdef __cplusplus
}
#endif

#endif /* MAIN_LORAWAN_TRACKER_API_H */
