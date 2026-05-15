#include "ag3335.h"
#include "smtc_hal.h"
#include "smtc_hal_dbg_trace.h"
#include "log_filter.h"
#include <math.h>
// Silence redefinition warning and keep local buffer size choice
#ifdef MINMEA_MAX_SENTENCE_LENGTH
#undef MINMEA_MAX_SENTENCE_LENGTH
#endif
#include "minmea.h"
// Override header's default length with local choice
#undef MINMEA_MAX_SENTENCE_LENGTH

// Lightweight, easily toggled troubleshooting tracing for GNSS power lifecycle.
// To disable at build time, pass -DGNSS_TRACE=0 in your project defines.
#ifndef GNSS_TRACE
#define GNSS_TRACE 1
#endif

#if GNSS_TRACE
#define GNSS_TRACE_INFO(...) LOG_GNSS(__VA_ARGS__)
#else
#define GNSS_TRACE_INFO(...)
#endif

/* Current AG3335M_V2.6.0 firmware rejects PAIR080,7 with PAIR001 status=4.
 * Keep the Swimming-mode hook compiled out rather than deleting it so a future
 * GNSS chip/firmware revision can re-enable MOB/PIW navigation-mode testing.
 */
#define AG3335_ENABLE_SWIMMING_NAV_MODE 0

#define GPS_INFO_PRINTF false

// Local parsing buffer size
#define MINMEA_MAX_SENTENCE_LENGTH  128
static char gps_nmea_line[MINMEA_MAX_SENTENCE_LENGTH] = { 0 };

static struct minmea_sentence_rmc frame_rmc;
static struct minmea_sentence_gga frame_gga;
static struct minmea_sentence_gst frame_gst;
static struct minmea_sentence_gsv frame_gsv;
static struct minmea_sentence_vtg frame_vtg;
static struct minmea_sentence_zda frame_zda;

static int32_t latitude_i32 = 0, longitude_i32 = 0, speed_i32 = 0;

// BLE interrupt flag for quality-driven scanning
static volatile bool ble_beacon_found = false;

// NMEA debug flag for MOB/PIW quality verification
static bool nmea_debug_enabled = false;

// Almanac status from PAIR550 response
// Each bit represents a satellite PRN (1-32 for GPS)
static uint32_t almanac_gps_valid = 0;      // GPS satellites with valid almanac
static uint32_t almanac_glonass_valid = 0;  // GLONASS satellites with valid almanac
static uint32_t almanac_beidou_valid = 0;   // BeiDou satellites with valid almanac  
static uint32_t almanac_last_check_rtc = 0; // RTC timestamp of last almanac check
static bool almanac_status_valid = false;   // True if we have valid almanac status

// Forward declarations for functions used before their definitions
static void gnss_scan_clean( void );
static void gnss_scan_lock_sleep( void );
static void gnss_scan_unlock_sleep( void );
static void gnss_scan_enter_rtc_mode( void );
static void gnss_set_navigation_mode( uint8_t mode );
void gnss_parse_pair550_response( const char* response );

static uint8_t app_nmea_check_sum( char *buf )
{
    uint8_t i = 0;
    uint8_t chk = 0;
    uint8_t len = strlen( buf );

	for( chk=buf[1], i = 2; i < len; i++ )
	{
		chk ^= buf[i];
	}

	return chk;
}

// Command retry configuration
#define GNSS_CMD_RETRIES        3
#define GNSS_CMD_RETRY_DELAY_MS 200

bool gnss_send_command( const char* command )
{
    char full_command[128];
    uint8_t checksum = app_nmea_check_sum( (char*)command );
    
    // Format: $PAIR_CMD,params*CS\r\n
    snprintf( full_command, sizeof(full_command), "%s*%02X\r\n", command, checksum );
    
    GNSS_TRACE_INFO( "GNSS CMD: %s (x%d)\n", command, GNSS_CMD_RETRIES );
    
    // Send command multiple times for reliability
    for( uint8_t i = 0; i < GNSS_CMD_RETRIES; i++ )
    {
        hal_uart_0_tx( (uint8_t*)full_command, strlen( full_command ) );
        hal_mcu_wait_ms( GNSS_CMD_RETRY_DELAY_MS );
    }
    
    return true;
}

void gnss_nmea_parse_line( char *line )
{
    // PRINTF( "%s\r\n", line );
    switch( minmea_sentence_id( line, false ))
    {
        case MINMEA_SENTENCE_RMC: // use for app
        {
            if( minmea_parse_rmc( &frame_rmc, line ))
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\r\n",
                        frame_rmc.latitude.value, frame_rmc.latitude.scale,
                        frame_rmc.longitude.value, frame_rmc.longitude.scale,
                        frame_rmc.speed.value, frame_rmc.speed.scale );
                PRINTF( "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\r\n",
                        minmea_rescale( &frame_rmc.latitude, 1000 ),
                        minmea_rescale( &frame_rmc.longitude, 1000 ),
                        minmea_rescale( &frame_rmc.speed, 1000 ));
                PRINTF( "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\r\n",
                        minmea_tocoord( &frame_rmc.latitude ),
                        minmea_tocoord( &frame_rmc.longitude ),
                        minmea_tofloat( &frame_rmc.speed ));
#endif
            }
            else
            {
#if GPS_INFO_PRINTF
                PRINTF(  "$xxRMC sentence is not parsed\r\n" );
#endif
            }
            break;
        }

        case MINMEA_SENTENCE_GGA: // use for app
        {
            if( minmea_parse_gga( &frame_gga, line ))
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxGGA: fix quality: %d\r\n", frame_gga.fix_quality );
#endif
                // Dynamic NMEA debug for MOB/PIW modes
                if( nmea_debug_enabled )
                {
                    if( frame_gga.fix_quality > 0 )
                    {
                        LOG_NMEA( "[NMEA] GGA: FIX=%d, sats=%d, HDOP=%.1f, lat=%.6f, lon=%.6f\r\n",
                                frame_gga.fix_quality,
                                frame_gga.satellites_tracked,
                                minmea_tofloat( &frame_gga.hdop ),
                                minmea_tocoord( &frame_gga.latitude ),
                                minmea_tocoord( &frame_gga.longitude ));
                    }
                    else
                    {
                        // Show no-fix status periodically (every ~5 seconds based on satellite count changes)
                        static int last_sats = -1;
                        if( frame_gga.satellites_tracked != last_sats )
                        {
                            LOG_NMEA( "[NMEA] GGA: NO FIX (sats tracked=%d, waiting for ephemeris)\r\n",
                                    frame_gga.satellites_tracked );
                            last_sats = frame_gga.satellites_tracked;
                        }
                    }
                }
            }
            else
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxGGA sentence is not parsed\r\n" );
#endif
            }
            break;
        }

        case MINMEA_SENTENCE_GST:
        {
            if( minmea_parse_gst( &frame_gst, line ))
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\r\n",
                        frame_gst.latitude_error_deviation.value, frame_gst.latitude_error_deviation.scale,
                        frame_gst.longitude_error_deviation.value, frame_gst.longitude_error_deviation.scale,
                        frame_gst.altitude_error_deviation.value, frame_gst.altitude_error_deviation.scale );
                PRINTF( "$xxGST fixed point latitude,longitude and altitude error deviation"
                        " scaled to one decimal place: (%d,%d,%d)\r\n",
                        minmea_rescale( &frame_gst.latitude_error_deviation, 10 ),
                        minmea_rescale( &frame_gst.longitude_error_deviation, 10 ),
                        minmea_rescale( &frame_gst.altitude_error_deviation, 10 ));
                PRINTF( "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)\r\n",
                        minmea_tofloat( &frame_gst.latitude_error_deviation ),
                        minmea_tofloat( &frame_gst.longitude_error_deviation ),
                        minmea_tofloat( &frame_gst.altitude_error_deviation ));
#endif
                // Dynamic NMEA debug for MOB/PIW modes - show horizontal accuracy
                if( nmea_debug_enabled )
                {
                    float lat_err = minmea_tofloat( &frame_gst.latitude_error_deviation );
                    float lon_err = minmea_tofloat( &frame_gst.longitude_error_deviation );
                    float hacc = sqrtf( lat_err * lat_err + lon_err * lon_err );
                    LOG_NMEA( "[NMEA] GST: HACC=%.1fm (lat_err=%.1f, lon_err=%.1f)\r\n",
                            hacc, lat_err, lon_err );
                }
            }
            else
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxGST sentence is not parsed\r\n" );
#endif
            }
            break;
        }

        case MINMEA_SENTENCE_GSV:
        {
            if( minmea_parse_gsv( &frame_gsv, line ))
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxGSV: message %d of %d\r\n", frame_gsv.msg_nr, frame_gsv.total_msgs );
                PRINTF( "$xxGSV: satellites in view: %d\r\n", frame_gsv.total_sats );
                for( int i = 0; i < 4; i++ )
                    PRINTF( "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\r\n",
                        frame_gsv.sats[i].nr,
                        frame_gsv.sats[i].elevation,
                        frame_gsv.sats[i].azimuth,
                        frame_gsv.sats[i].snr );
#endif
                // Dynamic NMEA debug - show satellite signal strengths
                // Only show first message of each constellation and only sats with signal
                if( nmea_debug_enabled && frame_gsv.msg_nr == 1 && frame_gsv.total_sats > 0 )
                {
                    // Determine constellation from NMEA talker ID in the original line
                    // GP = GPS, GL = GLONASS, GA = Galileo, GB/BD = BeiDou
                    const char* constellation = "??";
                    if( strncmp( line, "$GP", 3 ) == 0 ) constellation = "GPS";
                    else if( strncmp( line, "$GL", 3 ) == 0 ) constellation = "GLO";
                    else if( strncmp( line, "$GA", 3 ) == 0 ) constellation = "GAL";
                    else if( strncmp( line, "$GB", 3 ) == 0 || strncmp( line, "$BD", 3 ) == 0 ) constellation = "BDS";
                    
                    // Count satellites with actual signal (SNR > 0)
                    int with_signal = 0;
                    for( int i = 0; i < 4; i++ )
                    {
                        if( frame_gsv.sats[i].nr > 0 && frame_gsv.sats[i].snr > 0 )
                            with_signal++;
                    }
                    
                    char nmea_line[96];
                    int nmea_len = snprintf( nmea_line, sizeof( nmea_line ),
                                             "[NMEA] %s: %d SVs, %d with signal - ",
                                             constellation, frame_gsv.total_sats, with_signal );
                    for( int i = 0; i < 4; i++ )
                    {
                        // Only show satellites with valid PRN and signal
                        if( frame_gsv.sats[i].nr > 0 && frame_gsv.sats[i].snr > 0 )
                        {
                            nmea_len += snprintf( nmea_line + nmea_len, sizeof( nmea_line ) - nmea_len,
                                                  "%d:%ddB ", frame_gsv.sats[i].nr, frame_gsv.sats[i].snr );
                        }
                    }
                    LOG_NMEA( "%s\r\n", nmea_line );
                }
            }
            else
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxGSV sentence is not parsed\r\n");
#endif
            }
            break;
        }

        case MINMEA_SENTENCE_VTG:
        {
            if( minmea_parse_vtg( &frame_vtg, line ))
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxVTG: true track degrees = %f\r\n",
                        minmea_tofloat( &frame_vtg.true_track_degrees ));
                PRINTF( "        magnetic track degrees = %f\r\n",
                        minmea_tofloat( &frame_vtg.magnetic_track_degrees ));
                PRINTF( "        speed knots = %f\r\n",
                        minmea_tofloat( &frame_vtg.speed_knots ));
                PRINTF( "        speed kph = %f\r\n",
                        minmea_tofloat( &frame_vtg.speed_kph ));
#endif
            }
            else
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxVTG sentence is not parsed\r\n" );
#endif
            }
            break;
        }

        case MINMEA_SENTENCE_ZDA: // use for app
        {
            if( minmea_parse_zda( &frame_zda, line ))
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\r\n",
                        frame_zda.time.hours,
                        frame_zda.time.minutes,
                        frame_zda.time.seconds,
                        frame_zda.date.day,
                        frame_zda.date.month,
                        frame_zda.date.year,
                        frame_zda.hour_offset,
                        frame_zda.minute_offset );
#endif
            }
            else
            {
#if GPS_INFO_PRINTF
                PRINTF( "$xxZDA sentence is not parsed\r\n" );
#endif
            }
            break;
        }

        case MINMEA_INVALID:
        {
#if GPS_INFO_PRINTF
            PRINTF( "$xxxxx sentence is not valid\r\n" );
#endif
        }
        break;

        default:
        {
#if GPS_INFO_PRINTF
            PRINTF( "$xxxxx sentence is not parsed\r\n" );
#endif
        }
        break;
    }
}

void gnss_nmea_parse( char *str )
{
    uint16_t len = strlen( str );
    uint16_t begin = 0, end = 0;
    for( uint16_t i = 0; i < len; i++ )
    {
        if( str[i] == '$' ) begin = i;
        if( str[i] == '\r' ) end = i;

        if( end && end > begin )
        {
            memset( gps_nmea_line, 0, sizeof( gps_nmea_line ));
            memcpy( gps_nmea_line, str + begin, end - begin );
            if( strncmp( gps_nmea_line,"$PAIR",5 ) == 0 ) // ag3335 cmd parse
            {
                // Parse PAIR command responses (acknowledgments)
                // Format: $PAIR001,<cmd>,<status>*CS
                // Example: $PAIR001,590,0*37 (PAIR590 success)
                // Status: 0 = success, non-zero = error
                if( strncmp( gps_nmea_line, "$PAIR001", 8 ) == 0 )
                {
                    // Extract command number and status
                    int cmd_num = 0;
                    int status = -1;  // Default to -1 to detect parse issues
                    if( sscanf( gps_nmea_line, "$PAIR001,%d,%d", &cmd_num, &status ) == 2 )
                    {
                        if( status == 0 )
                        {
                            GNSS_TRACE_INFO( "AG3335 ACK: PAIR%03d OK: %s\n", cmd_num, gps_nmea_line );
                        }
                        else
                        {
                            GNSS_TRACE_INFO( "AG3335 ACK: PAIR%03d FAILED (status=%d): %s\n", cmd_num, status, gps_nmea_line );
                        }
                    }
                    else
                    {
                        GNSS_TRACE_INFO( "AG3335 ACK: Parse failed: %s\n", gps_nmea_line );
                    }
                }
                else if( strncmp( gps_nmea_line, "$PAIR550", 8 ) == 0 )
                {
                    // Parse almanac status response (data line, not ACK)
                    // Format: $PAIR550,<Constellation>,<L1_SV>,<Midi_SV>*CS
                    GNSS_TRACE_INFO( "GNSS: PAIR550 data received: %s\n", gps_nmea_line );
                    gnss_parse_pair550_response( gps_nmea_line );
                }
                else if( strncmp( gps_nmea_line, "$PAIR081", 8 ) == 0 )
                {
                    // Parse navigation mode query response
                    // Format: $PAIR081,<mode>*CS
                    int nav_mode = -1;
                    if( sscanf( gps_nmea_line, "$PAIR081,%d", &nav_mode ) == 1 )
                    {
                        GNSS_TRACE_INFO( "GNSS: Current navigation mode = %d\n", nav_mode );
                    }
                    else
                    {
                        GNSS_TRACE_INFO( "GNSS: PAIR081 response: %s\n", gps_nmea_line );
                    }
                }
                else
                {
                    // Log other PAIR responses for debugging
                    GNSS_TRACE_INFO( "AG3335: %s\n", gps_nmea_line );
                }
            }
            else
            {
                gnss_nmea_parse_line( gps_nmea_line );  //          
            }
            begin = 0;
            end = 0;
        }
    }
}

static void gnss_enable_nvram_auto_save( void )
{
    // PAIR510,1 - Enable NVRAM auto save for ephemeris/almanac persistence
    gnss_send_command( "$PAIR510,1" );
}

// Flag to indicate waiting for PAIR550 response
static volatile bool pair550_pending = false;
static volatile bool pair550_received = false;

void gnss_parse_pair550_response( const char* response )
{
    // PAIR550 response format per manual:
    // $PAIR550,<Constellation>,<L1_SV>,<Midi_SV>*CS
    //   Constellation: 0=GPS, 1=GLONASS, 2=Galileo, 3=BeiDou, 4=QZSS
    //   L1_SV: Hex bitmask of L1 satellites with valid almanac
    //   Midi_SV: Hex bitmask of Midi satellites (dual-band only, GLONASS only has L1)
    // Example: $PAIR550,0,FEC0BFFF,00000FFF*24
    //   Constellation 0 (GPS), L1=FEC0BFFF, Midi=00000FFF
    
    int constellation = -1;
    uint32_t l1_sv = 0, midi_sv = 0;
    
    // Try parsing with both L1 and Midi fields
    int parsed = sscanf( response, "$PAIR550,%d,%lX,%lX", &constellation, &l1_sv, &midi_sv );
    
    if( parsed >= 2 )  // At minimum need constellation and L1_SV
    {
        GNSS_TRACE_INFO( "GNSS: PAIR550 response - Constellation=%d, L1_SV=0x%08lX, Midi_SV=0x%08lX\n", 
                         constellation, l1_sv, midi_sv );
        
        // Store based on constellation type
        switch( constellation )
        {
            case 0:  // GPS
                almanac_gps_valid = l1_sv;
                break;
            case 1:  // GLONASS
                almanac_glonass_valid = l1_sv;
                break;
            case 3:  // BeiDou
                almanac_beidou_valid = l1_sv;
                break;
            default:
                GNSS_TRACE_INFO( "GNSS: Unknown constellation type %d\n", constellation );
                break;
        }
        
        almanac_last_check_rtc = hal_rtc_get_time_s();
        almanac_status_valid = true;
        pair550_received = true;
        
        // Count satellites with valid almanac (use L1 bitmask)
        uint8_t sv_count = __builtin_popcount( l1_sv );
        uint8_t midi_count = __builtin_popcount( midi_sv );
        
        const char* constellation_name = "Unknown";
        switch( constellation )
        {
            case 0: constellation_name = "GPS"; break;
            case 1: constellation_name = "GLONASS"; break;
            case 2: constellation_name = "Galileo"; break;
            case 3: constellation_name = "BeiDou"; break;
            case 4: constellation_name = "QZSS"; break;
        }
        
        GNSS_TRACE_INFO( "GNSS Almanac Status (%s, 1-day horizon):\n", constellation_name );
        GNSS_TRACE_INFO( "  L1 SVs:   0x%08lX (%u satellites)\n", l1_sv, sv_count );
        if( parsed >= 3 && midi_sv != 0 )
        {
            GNSS_TRACE_INFO( "  Midi SVs: 0x%08lX (%u satellites)\n", midi_sv, midi_count );
        }
        
        if( sv_count >= 4 )
        {
            GNSS_TRACE_INFO( "  Almanac: GOOD (>= 4 SVs valid)\n" );
        }
        else if( sv_count > 0 )
        {
            GNSS_TRACE_INFO( "  Almanac: PARTIAL (%u SVs valid, maintenance recommended)\n", sv_count );
        }
        else
        {
            GNSS_TRACE_INFO( "  Almanac: STALE (no valid almanac, cold start required)\n" );
        }
    }
    else
    {
        GNSS_TRACE_INFO( "GNSS: Failed to parse PAIR550 response: %s\n", response );
        almanac_status_valid = false;
        pair550_received = true;  // Mark as received even if parse failed
    }
}

static void gnss_check_almanac_status( void )
{
    // Reset response flag and mark as pending
    pair550_received = false;
    pair550_pending = true;
    
    // PAIR550,0,1 - Query almanac validity 1 day in future (Type 0 = GPS)
    // Using 1-day horizon since we check daily while charging anyway
    // Note: This command may fail with status=1 if module is busy or almanac not yet initialized
    // The actual response comes as $PAIR550,<gps>,<glonass>,<beidou>*CS
    GNSS_TRACE_INFO( "GNSS: Querying almanac status (PAIR550,0,1)...\n" );
    gnss_send_command( "$PAIR550,0,1" );
    
    // Wait for response (gnss_send_command already waits 3x200ms = 600ms)
    // Additional wait if needed
    if( !pair550_received )
    {
        uint32_t start_time = hal_rtc_get_time_ms();
        while( !pair550_received && ( hal_rtc_get_time_ms() - start_time ) < 500 )
        {
            hal_mcu_wait_ms( 50 );
        }
    }
    
    pair550_pending = false;
    
    if( !pair550_received )
    {
        // PAIR550 command was ACKed but no data response received
        // This is normal on cold start - almanac must be downloaded from satellites first
        // Almanac download takes 12.5 minutes of continuous satellite tracking
        GNSS_TRACE_INFO( "GNSS: Almanac not yet available (cold start - need satellite tracking)\n" );
        almanac_status_valid = false;
        
        // Set default values indicating unknown/cold state
        almanac_gps_valid = 0;
        almanac_glonass_valid = 0;
        almanac_beidou_valid = 0;
    }
}

void gnss_init( void )
{
    // Delay to allow serial connection before GNSS init logs
    GNSS_TRACE_INFO( "Waiting 5s for serial connection...\n" );
    hal_mcu_wait_ms( 5000 );
    
    // GPIO initialization only - no UART commands here
    // Commands are sent in gnss_scan_start() after PAIR382 succeeds
    hal_gpio_init_out( AG3335_POWER_EN, HAL_GPIO_SET ); // GPS_POWER_EN_PIN
    GNSS_TRACE_INFO( "GNSS: POWER_EN -> ON\n" );
    hal_mcu_wait_ms( 10 );
    hal_gpio_init_out( AG3335_VRTC_EN, HAL_GPIO_SET ); // GPS_VRTC_EN_PIN
    GNSS_TRACE_INFO( "GNSS: VRTC_EN -> ON\n" );
    hal_mcu_wait_ms( 10 );

    hal_gpio_init_out( AG3335_RESET, HAL_GPIO_SET ); // GPS_RST_PIN, reset by high
    hal_mcu_wait_ms( 10 );
    hal_gpio_set_value( AG3335_RESET, HAL_GPIO_RESET );
    GNSS_TRACE_INFO( "GNSS: RESET pulse (RST=HI then LOW)\n" );

    hal_gpio_init_out( AG3335_SLEEP_INT, HAL_GPIO_SET ); // GPS_SLEEP_INT_PIN, set GPS quit sleep mode, low active
    GNSS_TRACE_INFO( "GNSS: SLEEP_INT -> HIGH (awake)\n" );
    hal_gpio_init_out( AG3335_RTC_INT, HAL_GPIO_RESET ); // GPS_RTC_INT_PIN, set GPS quit rtc mode, high pulse(1ms)active
    hal_gpio_init_in( AG3335_RESETB_OUT, HAL_GPIO_PULL_MODE_UP, HAL_GPIO_IRQ_MODE_OFF, NULL ); // GPS_RESETB_OUT_PIN, gps reset ok, to mcu
    
    GNSS_TRACE_INFO( "GNSS: GPIO init complete\n" );
}

static void gnss_query_navigation_mode( void )
{
    // PAIR081 - Query current navigation mode
    gnss_send_command( "$PAIR081" );
}

static void gnss_set_navigation_mode( uint8_t mode )
{
    char command[24];
    
    // PAIR080,<mode> - Set navigation mode
    // 0 = Normal, 1 = Fitness, 4 = Stationary, 7 = Swimming
    snprintf( command, sizeof(command), "$PAIR080,%u", mode );
    gnss_send_command( command );
}

bool gnss_scan_start( void )
{
    gnss_scan_clean( );

    hal_uart_0_init( );

    hal_gpio_set_value( AG3335_POWER_EN, HAL_GPIO_SET );
    GNSS_TRACE_INFO( "GNSS: POWER_EN -> ON (scan_start)\n" );
    hal_mcu_wait_ms( 50 );

    // Power up GPS
    hal_gpio_set_value( AG3335_RTC_INT, HAL_GPIO_SET );
    GNSS_TRACE_INFO( "GNSS: RTC_INT -> HIGH (wake pulse begin)\n" );
    hal_mcu_wait_ms( 3 );
    hal_gpio_set_value( AG3335_RTC_INT, HAL_GPIO_RESET );
    GNSS_TRACE_INFO( "GNSS: RTC_INT -> LOW (wake pulse end)\n" );
    hal_mcu_wait_ms( 50 );

    gnss_scan_lock_sleep( );
    GNSS_TRACE_INFO( "GNSS: lock sleep (active tracking)\n" );
    
    // Now module is locked and responsive - send configuration commands
    // Use single sends with proper waits (not aggressive retries)
    hal_mcu_wait_ms( 100 );
    
#if AG3335_ENABLE_SWIMMING_NAV_MODE
    // Query current navigation mode first
    gnss_query_navigation_mode( );

    // Try Swimming navigation mode for MOB/PIW drift tracking. Field logs on AG3335M_V2.6.0
    // show PAIR080,7 is rejected with status=4, so this remains disabled until a future chip
    // or firmware revision supports it.
    gnss_set_navigation_mode( 7 );

    // Query again to see if it changed
    gnss_query_navigation_mode( );
#endif
    
    // Enable NVRAM auto-save for ephemeris/almanac persistence
    gnss_enable_nvram_auto_save( );

    // Wait a bit longer before almanac query - module needs time after wake
    hal_mcu_wait_ms( 500 );
    
    // Query almanac status (1-day horizon check)
    gnss_check_almanac_status( );

    return true;
}

void gnss_scan_stop( void )
{
    gnss_scan_unlock_sleep( );
    GNSS_TRACE_INFO( "GNSS: unlock sleep\n" );
    gnss_scan_enter_rtc_mode( );
    GNSS_TRACE_INFO( "GNSS: enter RTC mode\n" );
    hal_mcu_wait_ms( 50 );
    hal_gpio_set_value( AG3335_POWER_EN, HAL_GPIO_RESET );
    GNSS_TRACE_INFO( "GNSS: POWER_EN -> OFF (scan_stop)\n" );
    hal_uart_0_deinit( );
}

static void gnss_scan_lock_sleep( void )
{
    // PAIR382,1 - Lock sleep mode (prevent module from sleeping during scan)
    // Aggressive retries (25x) to wake module from dormant state
    char full_command[32];
    uint8_t checksum = app_nmea_check_sum( "$PAIR382,1" );
    snprintf( full_command, sizeof(full_command), "$PAIR382,1*%02X\r\n", checksum );
    
    GNSS_TRACE_INFO( "GNSS CMD: $PAIR382,1 (x25 aggressive wake)\n" );
    for( uint8_t i = 0; i < 25; i++ )
    {
        hal_uart_0_tx( (uint8_t*)full_command, strlen( full_command ) );
        hal_mcu_wait_ms( 40 );
    }
}

static void gnss_scan_unlock_sleep( void )
{
    // PAIR382,0 - Unlock sleep mode (allow module to sleep)
    gnss_send_command( "$PAIR382,0" );
}

static void gnss_scan_enter_rtc_mode( void )
{
    // PAIR650,0 - Enter RTC low-power mode
    // Aggressive retries (25x) to ensure module receives command
    char full_command[32];
    uint8_t checksum = app_nmea_check_sum( "$PAIR650,0" );
    snprintf( full_command, sizeof(full_command), "$PAIR650,0*%02X\r\n", checksum );
    
    GNSS_TRACE_INFO( "GNSS CMD: $PAIR650,0 (x25 enter RTC mode)\n" );
    for( uint8_t i = 0; i < 25; i++ )
    {
        hal_uart_0_tx( (uint8_t*)full_command, strlen( full_command ) );
        hal_mcu_wait_ms( 40 );
    }
}

static void gnss_scan_clean( void )
{
    memset( &frame_rmc, 0, sizeof( struct minmea_sentence_rmc ));
    memset( &frame_gga, 0, sizeof( struct minmea_sentence_gga ));
    memset( &frame_gst, 0, sizeof( struct minmea_sentence_gst ));
    memset( &frame_gsv, 0, sizeof( struct minmea_sentence_gsv ));
    memset( &frame_vtg, 0, sizeof( struct minmea_sentence_vtg ));
    memset( &frame_zda, 0, sizeof( struct minmea_sentence_zda ));
}  

bool gnss_get_fix_status( void )
{
    bool result = false;
    float latitude = 0, longitude = 0, speed = 0;

    if( frame_rmc.latitude.scale && frame_rmc.longitude.scale && frame_rmc.speed.scale )
    {
        latitude = minmea_tocoord( &frame_rmc.latitude );
        longitude = minmea_tocoord( &frame_rmc.longitude );
        speed = minmea_tofloat( &frame_rmc.speed );
        if( latitude <= 180 && longitude <= 360 )
        {
            latitude *= 1000000;
            longitude *= 1000000;
            speed *= 1000000;

            latitude_i32 = latitude;
            longitude_i32 = longitude;
            speed_i32 = speed;

            result =  true;
        }
    }

    return result;
}

void gnss_get_position( int32_t *lat, int32_t *lon )
{
    *lat = latitude_i32;
    *lon = longitude_i32;
}

void gnss_parse_handler( char *nmea )
{
    gnss_nmea_parse( nmea );
}

/*
 * -----------------------------------------------------------------------------
 * --- QUALITY-DRIVEN GNSS SCANNING FOR MOB/PIW --------------------------------
 * -----------------------------------------------------------------------------
 */

bool gnss_get_quality_fix( gnss_fix_t *fix )
{
    if( fix == NULL )
    {
        return false;
    }
    
    // Initialize to invalid state
    memset( fix, 0, sizeof( gnss_fix_t ));
    fix->valid = false;
    fix->hdop = 99.9f;  // Invalid HDOP
    fix->hacc = 999.0f; // Invalid accuracy
    
    // Check basic fix validity from RMC
    if( !frame_rmc.latitude.scale || !frame_rmc.longitude.scale )
    {
        return false;
    }
    
    float latitude = minmea_tocoord( &frame_rmc.latitude );
    float longitude = minmea_tocoord( &frame_rmc.longitude );
    
    if( latitude > 180 || longitude > 360 )
    {
        return false;
    }
    
    // Populate fix structure
    fix->latitude = (int32_t)( latitude * 1000000 );
    fix->longitude = (int32_t)( longitude * 1000000 );
    fix->speed = (int32_t)( minmea_tofloat( &frame_rmc.speed ) * 1000000 );
    
    // Get HDOP from GGA sentence
    if( frame_gga.hdop.scale > 0 )
    {
        fix->hdop = minmea_tofloat( &frame_gga.hdop );
    }
    
    // Get fix quality and satellites from GGA
    fix->fix_quality = frame_gga.fix_quality;
    fix->satellites = frame_gga.satellites_tracked;
    
    // Calculate horizontal accuracy from GST if available
    // HACC ≈ sqrt(lat_err² + lon_err²) in meters
    if( frame_gst.latitude_error_deviation.scale > 0 && 
        frame_gst.longitude_error_deviation.scale > 0 )
    {
        float lat_err = minmea_tofloat( &frame_gst.latitude_error_deviation );
        float lon_err = minmea_tofloat( &frame_gst.longitude_error_deviation );
        fix->hacc = sqrtf( lat_err * lat_err + lon_err * lon_err );
    }
    else if( fix->hdop < 99.0f )
    {
        // Estimate HACC from HDOP if GST not available
        // Typical GPS error ≈ HDOP * 5 meters (conservative)
        fix->hacc = fix->hdop * 5.0f;
    }
    
    // Valid if we have a real GPS fix (quality >= 1)
    fix->valid = ( fix->fix_quality >= 1 ); 
    
    // Update legacy static variables for backward compatibility
    if( fix->valid )
    {
        latitude_i32 = fix->latitude;
        longitude_i32 = fix->longitude;
        speed_i32 = fix->speed;
    }
    
    return fix->valid;
}

bool gnss_scan_until_good( uint32_t max_ms, float max_hdop, float max_hacc, gnss_fix_t *fix, bool skip_power_management )
{
    if( fix == NULL )
    {
        return false;
    }
    
    // Initialize fix structure
    memset( fix, 0, sizeof( gnss_fix_t ));
    fix->valid = false;
    
    // Clear BLE interrupt flag
    ble_beacon_found = false;
    
    // Start GNSS scan (unless in background mode)
    if( !skip_power_management )
    {
        gnss_scan_start( );
    }
    
    uint32_t start_time = hal_rtc_get_time_ms( );
    uint32_t elapsed = 0;
    bool got_good_fix = false;
    
    GNSS_TRACE_INFO( "GNSS quality scan: max %lu ms, HDOP<%.1f, HACC<%.1f m\n", 
                     max_ms, max_hdop, max_hacc );
    
    // Poll at ~1 Hz until good fix or timeout
    while( elapsed < max_ms )
    {
        // Wait 1 second between checks
        hal_mcu_wait_ms( 1000 );
        elapsed = hal_rtc_get_time_ms( ) - start_time;
        
        // Check for BLE interrupt
        if( ble_beacon_found )
        {
            GNSS_TRACE_INFO( "GNSS scan interrupted by BLE beacon at %lu ms\n", elapsed );
            break;
        }
        
        // Try to get a quality fix
        if( gnss_get_quality_fix( fix ))
        {
            GNSS_TRACE_INFO( "GNSS fix @ %lu ms: HDOP=%.1f, HACC=%.1f m, sats=%d\n",
                            elapsed, fix->hdop, fix->hacc, fix->satellites );
            
            // Check if fix meets quality thresholds
            if( fix->hdop <= max_hdop && fix->hacc <= max_hacc )
            {
                got_good_fix = true;
                GNSS_TRACE_INFO( "GNSS GOOD FIX at %lu ms - quality OK!\n", elapsed );
                break;
            }
        }
    }
    
    // Stop GNSS module (unless in background mode)
    if( !skip_power_management )
    {
        gnss_scan_stop( );
    }
    
    // Get final fix status if we didn't get a good one during polling
    if( !got_good_fix && !ble_beacon_found )
    {
        gnss_get_quality_fix( fix );
        GNSS_TRACE_INFO( "GNSS timeout after %lu ms: valid=%d, HDOP=%.1f, HACC=%.1f\n",
                        elapsed, fix->valid, fix->hdop, fix->hacc );
    }
    
    return got_good_fix;
}

bool gnss_check_ble_interrupt( void )
{
    return ble_beacon_found;
}

void gnss_set_ble_found( bool found )
{
    ble_beacon_found = found;
    if( found )
    {
        GNSS_TRACE_INFO( "BLE beacon found - GNSS interrupt flag set\n" );
    }
}

void gnss_enable_nmea_debug( bool enable )
{
    nmea_debug_enabled = enable;
    GNSS_TRACE_INFO( "NMEA debug output %s\n", enable ? "ENABLED" : "DISABLED" );
}

/*
 * -----------------------------------------------------------------------------
 * --- ALMANAC STATUS FUNCTIONS -----------------------------------------------
 * -----------------------------------------------------------------------------
 */

bool gnss_almanac_is_valid( void )
{
    if( !almanac_status_valid )
    {
        return false;
    }
    
    // Consider almanac valid if at least 4 GPS satellites have valid almanac
    // (minimum for a 3D fix with one redundant)
    uint8_t gps_count = __builtin_popcount( almanac_gps_valid );
    return ( gps_count >= 4 );
}

bool gnss_almanac_needs_maintenance( void )
{
    if( !almanac_status_valid )
    {
        // If we've never checked, assume maintenance is needed
        return true;
    }
    
    uint8_t gps_count = __builtin_popcount( almanac_gps_valid );
    uint8_t total_count = gps_count + 
                          __builtin_popcount( almanac_glonass_valid ) +
                          __builtin_popcount( almanac_beidou_valid );
    
    // Need maintenance if:
    // - Less than 4 GPS satellites with valid almanac, OR
    // - Total constellation coverage is very low
    return ( gps_count < 4 || total_count < 8 );
}

uint8_t gnss_almanac_get_valid_sv_count( void )
{
    if( !almanac_status_valid )
    {
        return 0;
    }
    
    return __builtin_popcount( almanac_gps_valid ) +
           __builtin_popcount( almanac_glonass_valid ) +
           __builtin_popcount( almanac_beidou_valid );
}

uint32_t gnss_almanac_get_last_check_time( void )
{
    return almanac_last_check_rtc;
}

void gnss_almanac_refresh_status( void )
{
    // This can be called when GNSS is already powered and UART is initialized
    // to refresh the almanac status
    gnss_check_almanac_status();
}
