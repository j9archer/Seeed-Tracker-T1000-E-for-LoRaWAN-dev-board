#ifndef MAIN_LORAWAN_TRACKER_API_H
#define MAIN_LORAWAN_TRACKER_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Minimal API surface used by tracker modules without depending on SES app path */

bool app_send_frame( const uint8_t* buffer, const uint8_t length, bool tx_confirmed, bool emergency );

void app_tracker_new_run( uint8_t event );

void app_radio_set_sleep( void );

void app_lora_stack_suspend( void );

#ifdef __cplusplus
}
#endif

#endif /* MAIN_LORAWAN_TRACKER_API_H */
