#include "remex_abp_derive.h"

#include <string.h>

#include "apps_utilities.h"
#include "cmac.h"
#include "lorawan_key_config.h"
#include "smtc_hal.h"

#ifndef REMEX_ABP_NET_ID_START
#ifdef REMEX_ABP_NET_ID
#define REMEX_ABP_NET_ID_START REMEX_ABP_NET_ID
#else
#define REMEX_ABP_NET_ID_START "E00110"
#endif
#endif

#ifndef REMEX_ABP_NET_ID_END
#define REMEX_ABP_NET_ID_END REMEX_ABP_NET_ID_START
#endif

#ifndef REMEX_ABP_MASTER_KEY
#error "REMEX_ABP_MASTER_KEY must be defined for deterministic ABP derivation"
#endif

#ifndef REMEX_ABP_GROUP1_BLOCK_SIZE
#ifdef Group_1_block
#define REMEX_ABP_GROUP1_BLOCK_SIZE Group_1_block
#else
#define REMEX_ABP_GROUP1_BLOCK_SIZE 32
#endif
#endif

#define REMEX_ABP_GROUP_ID 2
#define REMEX_ABP_EPOCH 0
#define REMEX_TYPE7_DEVADDR_PREFIX 0xFE000000UL
#define REMEX_TYPE7_NWKID_MASK 0x1FFFFUL
#define REMEX_TYPE7_BLOCK_SIZE 128UL

/*
 * Deterministic ABP for Crew Tags.
 *
 * DevEUI remains the factory identity read from device storage; it is not a
 * secret and must not come from this build. The only build-time secrets here
 * are the RemEX fleet master key and project allocation context.
 *
 * DevAddr allocation follows the LoRaWAN Type 7 layout: each NetID maps to one
 * 128-address /25 block, so a client/project can be filtered at the gateway by
 * its assigned block(s). Group 1 gets the first REMEX_ABP_GROUP1_BLOCK_SIZE
 * addresses in every block for local vessel devices; this Crew Tag firmware is
 * Group 2 and derives into the remainder.
 *
 * The server-side derive_abp.py helper must build the same CMAC context
 * byte-for-byte. Include the full allocation context in the CMAC so moving a
 * device between client/project blocks intentionally derives a new session.
 */
static uint32_t remex_abp_net_id_to_u32( const uint8_t net_id[3] )
{
    return ( ( uint32_t ) net_id[0] << 16 ) | ( ( uint32_t ) net_id[1] << 8 ) | ( uint32_t ) net_id[2];
}

static void remex_abp_cmac( const uint8_t key[16], const char* label, const uint8_t dev_eui[8], uint8_t out[16] )
{
    uint8_t      net_id_start[3] = { 0 };
    uint8_t      net_id_end[3]   = { 0 };
    uint8_t      context[64];
    uint32_t     offset = 0;
    const char*  prefix = "remex-abp-v1";
    const size_t prefix_len = strlen( prefix );
    const size_t label_len  = strlen( label );
    AES_CMAC_CTX cmac_ctx;

    hal_hex_to_bin( REMEX_ABP_NET_ID_START, net_id_start, sizeof( net_id_start ) );
    hal_hex_to_bin( REMEX_ABP_NET_ID_END, net_id_end, sizeof( net_id_end ) );

    memcpy( &context[offset], prefix, prefix_len );
    offset += prefix_len;
    context[offset++] = 0;
    memcpy( &context[offset], label, label_len );
    offset += label_len;
    context[offset++] = 0;
    memcpy( &context[offset], net_id_start, sizeof( net_id_start ) );
    offset += sizeof( net_id_start );
    memcpy( &context[offset], net_id_end, sizeof( net_id_end ) );
    offset += sizeof( net_id_end );
    context[offset++] = REMEX_ABP_GROUP_ID;
    context[offset++] = ( uint8_t ) REMEX_ABP_GROUP1_BLOCK_SIZE;
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
    uint8_t net_id_start_bin[3] = { 0 };
    uint8_t net_id_end_bin[3]   = { 0 };
    uint32_t net_id_start;
    uint32_t net_id_end;
    uint32_t block_count;
    uint32_t block_index;
    uint32_t group2_count;
    uint32_t raw_block;
    uint32_t raw_offset;
    uint32_t selected_net_id;
    uint32_t offset;

    if( ( dev_eui == NULL ) || ( dev_addr == NULL ) || ( nwk_s_key == NULL ) || ( app_s_key == NULL ) )
    {
        return false;
    }

    if( ( REMEX_ABP_GROUP1_BLOCK_SIZE == 0 ) || ( REMEX_ABP_GROUP1_BLOCK_SIZE >= REMEX_TYPE7_BLOCK_SIZE ) )
    {
        return false;
    }

    hal_hex_to_bin( REMEX_ABP_MASTER_KEY, master_key, sizeof( master_key ) );
    hal_hex_to_bin( REMEX_ABP_NET_ID_START, net_id_start_bin, sizeof( net_id_start_bin ) );
    hal_hex_to_bin( REMEX_ABP_NET_ID_END, net_id_end_bin, sizeof( net_id_end_bin ) );

    net_id_start = remex_abp_net_id_to_u32( net_id_start_bin );
    net_id_end   = remex_abp_net_id_to_u32( net_id_end_bin );
    if( net_id_start > net_id_end )
    {
        return false;
    }

    block_count  = net_id_end - net_id_start + 1;
    group2_count = REMEX_TYPE7_BLOCK_SIZE - REMEX_ABP_GROUP1_BLOCK_SIZE;

    remex_abp_cmac( master_key, "nwkskey", dev_eui, nwk_s_key );
    remex_abp_cmac( master_key, "appskey", dev_eui, app_s_key );
    remex_abp_cmac( master_key, "devaddr", dev_eui, digest );

    raw_block  = ( ( uint32_t ) digest[0] << 24 ) | ( ( uint32_t ) digest[1] << 16 ) |
                ( ( uint32_t ) digest[2] << 8 ) | ( uint32_t ) digest[3];
    raw_offset = ( ( uint32_t ) digest[4] << 24 ) | ( ( uint32_t ) digest[5] << 16 ) |
                 ( ( uint32_t ) digest[6] << 8 ) | ( uint32_t ) digest[7];

    block_index     = raw_block % block_count;
    selected_net_id = net_id_start + block_index;
    offset          = REMEX_ABP_GROUP1_BLOCK_SIZE + ( raw_offset % group2_count );
    *dev_addr       = REMEX_TYPE7_DEVADDR_PREFIX | ( ( selected_net_id & REMEX_TYPE7_NWKID_MASK ) << 7 ) | offset;

    return true;
}
