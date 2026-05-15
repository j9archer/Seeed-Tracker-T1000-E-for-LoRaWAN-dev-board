#ifndef LOG_FILTER_H
#define LOG_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

typedef enum
{
    LOG_FILTER_LORA = 0,
    LOG_FILTER_NMEA,
    LOG_FILTER_GNSS,
    LOG_FILTER_BLE,
    LOG_FILTER_WIFI,
    LOG_FILTER_COUNT
} log_filter_category_t;

bool log_filter_is_enabled( log_filter_category_t category );
void log_filter_printf( log_filter_category_t category, const char* fmt, ... );
void log_filter_vprintf( log_filter_category_t category, const char* fmt, va_list args );

// Consumes single-key serial toggles before they enter the AT command buffer.
// Keys are intentionally sparse so ordinary AT commands are not intercepted.
bool log_filter_handle_serial_char( uint8_t ch );

#define LOG_LORA( ... ) log_filter_printf( LOG_FILTER_LORA, __VA_ARGS__ )
#define LOG_NMEA( ... ) log_filter_printf( LOG_FILTER_NMEA, __VA_ARGS__ )
#define LOG_GNSS( ... ) log_filter_printf( LOG_FILTER_GNSS, __VA_ARGS__ )
#define LOG_BLE( ... )  log_filter_printf( LOG_FILTER_BLE,  __VA_ARGS__ )
#define LOG_WIFI( ... ) log_filter_printf( LOG_FILTER_WIFI, __VA_ARGS__ )

#ifdef __cplusplus
}
#endif

#endif  // LOG_FILTER_H
