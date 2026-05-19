#ifndef REMEX_ABP_DERIVE_H
#define REMEX_ABP_DERIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool remex_abp_derive_session( const uint8_t dev_eui[8], uint32_t* dev_addr, uint8_t nwk_s_key[16],
                               uint8_t app_s_key[16] );

#ifdef __cplusplus
}
#endif

#endif
