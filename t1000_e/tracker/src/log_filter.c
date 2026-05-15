#include "log_filter.h"
#include "smtc_hal_dbg_trace.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    char key;
    const char* name;
    const char* prefix;
    bool enabled;
} log_filter_state_t;

static log_filter_state_t log_filters[LOG_FILTER_COUNT] = {
    [LOG_FILTER_LORA] = { 'L', "LORA", "LORA: ", true },
    [LOG_FILTER_NMEA] = { 'N', "NMEA", "[NMEA] ", true },
    [LOG_FILTER_GNSS] = { 'G', "GNSS", "GNSS: ", true },
    [LOG_FILTER_BLE]  = { 'B', "BLE",  "BLE: ",  true },
    [LOG_FILTER_WIFI] = { 'W', "WIFI", "WIFI: ", true },
};

static bool log_filter_has_prefix( const char* text, const char* prefix )
{
    return strncmp( text, prefix, strlen( prefix ) ) == 0;
}

static bool log_filter_is_blank_fragment( const char* text )
{
    while( *text != '\0' )
    {
        if( !isspace( ( unsigned char )*text ) )
        {
            return false;
        }
        text++;
    }
    return true;
}

static void log_filter_print_status( void )
{
    PRINTF( "\r\nLOG FILTER:" );
    for( uint8_t i = 0; i < LOG_FILTER_COUNT; i++ )
    {
        PRINTF( " %c=%s", log_filters[i].key, log_filters[i].enabled ? "ON" : "OFF" );
    }
    PRINTF( "\r\n" );
}

bool log_filter_is_enabled( log_filter_category_t category )
{
    if( category >= LOG_FILTER_COUNT )
    {
        return true;
    }
    return log_filters[category].enabled;
}

void log_filter_vprintf( log_filter_category_t category, const char* fmt, va_list args )
{
    if( category >= LOG_FILTER_COUNT || log_filters[category].enabled == false )
    {
        return;
    }

    char line[256];
    vsnprintf( line, sizeof( line ), fmt, args );

    // Semtech radio-planner traces sometimes write indentation or reset colors as
    // separate fragments. Suppress those by themselves so the next real RF line
    // gets the LORA tag cleanly.
    if( line[0] == '\0' || log_filter_is_blank_fragment( line ) )
    {
        return;
    }
    if(( category == LOG_FILTER_LORA ) && ( strcmp( line, "LORA: " ) == 0 ))
    {
        return;
    }

    const char* prefix = log_filters[category].prefix;
    if( log_filter_has_prefix( line, prefix ) ||
        ( category == LOG_FILTER_LORA && log_filter_has_prefix( line, "LORA:" ) ) ||
        ( category == LOG_FILTER_NMEA && log_filter_has_prefix( line, "[NMEA]" ) ) ||
        ( category == LOG_FILTER_GNSS && log_filter_has_prefix( line, "GNSS:" ) ) )
    {
        PRINTF( "%s", line );
    }
    else
    {
        PRINTF( "%s%s", prefix, line );
    }
}

void log_filter_printf( log_filter_category_t category, const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    log_filter_vprintf( category, fmt, args );
    va_end( args );
}

bool log_filter_handle_serial_char( uint8_t ch )
{
    if( ch == '?' )
    {
        log_filter_print_status( );
        return true;
    }

    const char key = ( char )toupper( ch );
    for( uint8_t i = 0; i < LOG_FILTER_COUNT; i++ )
    {
        if( key == log_filters[i].key )
        {
            log_filters[i].enabled = !log_filters[i].enabled;
            PRINTF( "\r\nLOG FILTER: %s %s\r\n", log_filters[i].name,
                    log_filters[i].enabled ? "ON" : "OFF" );
            return true;
        }
    }

    return false;
}
