
#include "app_config_param.h"
#include "default_config_settings.h"

app_param_t app_param = 
{ 
    .param_version = REMEX_CREW_CONFIG_VERSION,
    .lora_info = {
        .Platform = REMEX_DEFAULT_PLATFORM,
        .ActiveRegion = REMEX_DEFAULT_ACTIVE_REGION,
        .ChannelGroup = REMEX_DEFAULT_CHANNEL_GROUP,
        .ActivationType = REMEX_DEFAULT_ACTIVATION_TYPE,
        .Retry = REMEX_DEFAULT_RETRY,
        .lr_ADR_en = REMEX_DEFAULT_LORAWAN_ADR_ENABLED,
        .lr_DR_min = REMEX_DEFAULT_LORAWAN_DR_MIN,
        .lr_DR_max = REMEX_DEFAULT_LORAWAN_DR_MAX,
        .DevEui = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .JoinEui = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .AppKey = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .DevAddr = 0x00000000,
        .NwkKey= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .AppSKey = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .NwkSKey = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .DeviceCode = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .DeviceKey = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    },
    .hardware_info = {
        .Sn = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        .pos_strategy = REMEX_DEFAULT_POSITION_STRATEGY,
        .pos_interval = REMEX_DEFAULT_UPLINK_INTERVAL_MIN,
        .sos_mode = REMEX_DEFAULT_SOS_MODE,
        .acc_en = REMEX_DEFAULT_ACCELEROMETER_ENABLED,
        .gnss_overtime = REMEX_DEFAULT_GNSS_MAX_SCAN_TIME_S,
        .wifi_max = 3, // default 3
        .beac_overtime = REMEX_DEFAULT_IBEACON_SCAN_TIMEOUT_S,
        .beac_max = REMEX_DEFAULT_IBEACON_SCAN_MAX,
        .beac_uuid = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // uuid filter
        .uuid_num = REMEX_DEFAULT_GROUP_UUID_LEN,
        .test_mode = 0,
    }
};

void hexTonum( unsigned char *out_data, unsigned char *in_data, unsigned short Size ) 
{
    for( unsigned char i = 0; i < Size; i++ )
    {
        out_data[2*i] = (in_data[i] >> 4);
        out_data[2*i+1] = (in_data[i] & 0x0F);  
    }
    for( unsigned char i = 0; i < 2 * Size; i++ )
    {
        if(( out_data[i] >= 0 ) && ( out_data[i] <= 9 )) 
        {
            out_data[i] = '0'+ out_data[i];
        } 
        else if(( out_data[i] >= 0x0A ) && ( out_data[i] <= 0x0F )) 
        {
            out_data[i] = 'A'- 10 + out_data[i];
        } 
        else 
        {
            return;
        }
    }
}

void numTohex( unsigned char *out_data, unsigned char *in_data, unsigned short Size ) 
{
    unsigned char temp;
    for( unsigned char i = 0; i < Size; i++ )
    {
        temp = Char2Nibble( in_data[2 * i] );
        temp = temp<<4;
        temp += Char2Nibble( in_data[2 * i + 1] );
        out_data[i] = temp;
    }
}

uint8_t Char2Nibble( char Char ) 
{
    if((( Char >= '0' ) && ( Char <= '9' ))) 
    {
        return Char - '0';
    } 
    else if((( Char >= 'a' ) && ( Char <= 'f' ))) 
    {
        return Char - 'a' + 10;
    } 
    else if(( Char >= 'A' ) && ( Char <= 'F' )) 
    {
        return Char - 'A' + 10;
    } 
    else 
    {
        return 0xF0;
    }
}
