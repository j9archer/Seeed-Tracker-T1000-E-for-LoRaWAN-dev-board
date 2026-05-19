#include "remex_abp_derive.h"

#include <string.h>

#include "apps_utilities.h"
#include "cmac.h"
#include "lorawan_key_config.h"
#include "smtc_hal.h"

#ifndef REMEX_ABP_NET_ID
#define REMEX_ABP_NET_ID "E00110"
#endif

#ifndef REMEX_ABP_MASTER_KEY
#error "REMEX_ABP_MASTER_KEY must be defined for deterministic ABP derivation"
#endif

#define REMEX_ABP_GROUP_ID 2
#define REMEX_ABP_EPOCH 0
#define REMEX_CREW_DEVADDR_BASE 0xFE008900UL
#define REMEX_CREW_DEVADDR_COUNT 1792UL

/*
 * Deterministic ABP for Crew Tags.
 *
 * DevEUI remains the factory identity read from device storage; it is not a
 * secret and must not come from this build. The only build-time secrets here
 * are the RemEX fleet master key and NetID/allocation context. The server-side
 * derive_abp.py helper must build the same CMAC context byte-for-byte.
 */
static void remex_abp_cmac( const uint8_t key[16], const char* label, const uint8_t dev_eui[8], uint8_t out[16] )
{
    uint8_t      net_id[3] = { 0 };
    uint8_t      context[64];
    uint32_t     offset = 0;
    const char*  prefix = "remex-abp-v1";
    const size_t prefix_len = strlen( prefix );
    const size_t label_len  = strlen( label );
    AES_CMAC_CTX cmac_ctx;

    hal_hex_to_bin( REMEX_ABP_NET_ID, net_id, sizeof( net_id ) );

    memcpy( &context[offset], prefix, prefix_len );
    offset += prefix_len;
    context[offset++] = 0;
    memcpy( &context[offset], label, label_len );
    offset += label_len;
    context[offset++] = 0;
    memcpy( &context[offset], net_id, sizeof( net_id ) );
    offset += sizeof( net_id );
    context[offset++] = REMEX_ABP_GROUP_ID;
    context[offset++] = ( uint8_t )( REMEX_ABP_EPOCH >> 8 );
    context[offset++] = ( uint8_t )( REMEX_ABP_EPOCH & 0xFF );
    memcpy( &context[offset], dev_eui, 8 );
    offset += 8;

    AES_CMAC_Init( &cmac_ctx );
    AES_CMAC_SetKey( &cmac_ctx, key );
    AES_CMAC_Update( &cmac_ctx, context, offset );
    AES_CMAC_Final( out, &cmac_ctx );
}

bool remex_abp_derive_session( const uint8_t dev_eui[8], uint32_t* dev_addr, uint8_t nwk_s_key[16],
                               uint8_t app_s_key[16] )
{
    uint8_t master_key[16] = { 0 };
    uint8_t digest[16]     = { 0 };
    uint32_t raw_offset;

    if( ( dev_eui == NULL ) || ( dev_addr == NULL ) || ( nwk_s_key == NULL ) || ( app_s_key == NULL ) )
    {
        return false;
    }

    hal_hex_to_bin( REMEX_ABP_MASTER_KEY, master_key, sizeof( master_key ) );

    remex_abp_cmac( master_key, "nwkskey", dev_eui, nwk_s_key );
    remex_abp_cmac( master_key, "appskey", dev_eui, app_s_key );
    remex_abp_cmac( master_key, "devaddr", dev_eui, digest );

    raw_offset = ( ( uint32_t ) digest[0] << 24 ) | ( ( uint32_t ) digest[1] << 16 ) |
                 ( ( uint32_t ) digest[2] << 8 ) | ( uint32_t ) digest[3];
    *dev_addr = REMEX_CREW_DEVADDR_BASE + ( raw_offset % REMEX_CREW_DEVADDR_COUNT );

    return true;
}
