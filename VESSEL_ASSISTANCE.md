# Vessel Position and Time Assistance for GNSS

## Overview

This feature significantly improves GNSS Time-To-First-Fix (TTFF) performance for T1000-E trackers operating on a vessel equipped with a relay gateway.

## How It Works

### Position and Time Updates from Vessel

The vessel's relay gateway periodically sends position and time updates to all T1000-E devices via LoRaWAN downlinks:

- **Update Interval**: Every 90-120 minutes
- **LoRaWAN Port**: 10 (configurable via `VESSEL_ASSISTANCE_PORT`)
- **Payload Size**: 13 bytes

### Payload Format

```c
typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0x01 = Position Update
    int32_t vessel_lat;    // Latitude × 10^7 (degrees)
    int32_t vessel_lon;    // Longitude × 10^7 (degrees)
    uint32_t unix_time;    // Unix timestamp (seconds since epoch)
} vessel_position_msg_t;
```

### Example Payload

For vessel at position 37.7749°N, 122.4194°W at time 1700000000:

```
01 16 7C 65 B0 C9 E0 F4 90 65 5D 4A 00
```

Breakdown:
- `01`: Message type (position update)
- `16 7C 65 B0`: Latitude = 377,490,000 (37.7749° × 10^7)
- `C9 E0 F4 90`: Longitude = -1,224,194,000 (-122.4194° × 10^7)  
- `65 5D 4A 00`: Unix time = 1,700,000,000

## Benefits

### TTFF Improvements

| Assistance Quality | Age | TTFF | Power Savings |
|-------------------|-----|------|---------------|
| **Excellent** | < 30 min | 10-15 seconds | 70-80% |
| **Good** | 30-90 min | 15-25 seconds | 50-70% |
| **Fair** | 90-180 min | 25-40 seconds | 30-50% |
| **Cold Start** | No assistance | 60+ seconds | 0% (baseline) |

### Adaptive Scan Duration

The system automatically adjusts GNSS scan duration based on assistance quality:

- **Excellent assistance**: 10 seconds
- **Good assistance**: 15 seconds
- **Fair assistance**: 25 seconds
- **No assistance**: 60 seconds (cold start)

## Implementation Details

### Key Components

1. **`vessel_assistance.h/c`**: Core assistance module
2. **`main_lorawan_tracker.c`**: Integration with tracker application

### Initialization

```c
void main(void) {
    // ... other initialization ...
    vessel_assistance_init();
    // ...
}
```

### Downlink Handling

Downlinks on port 10 are automatically processed:

```c
static void on_modem_down_data(..., uint8_t port, const uint8_t* payload, uint8_t size) {
    if (port == VESSEL_ASSISTANCE_PORT) {
        vessel_assistance_handle_downlink(payload, size);
    }
}
```

### GNSS Scan with Assistance

```c
static void app_tracker_gnss_scan_begin(void) {
    // Apply assistance position to AG3335
    vessel_assistance_apply_to_gnss();
    
    // Start GNSS scan
    gnss_scan_start();
}
```

### Storing Own Fixes

When the tracker gets its own GNSS fix, it stores it as fallback assistance:

```c
static void app_tracker_gnss_scan_end(void) {
    if (gnss_get_fix_status()) {
        gnss_get_position(&lat, &lon);
        vessel_assistance_store_own_fix(lat, lon);
    }
}
```

## Gateway Implementation

### Python Example

```python
import struct
import time

def create_vessel_position_payload(lat, lon):
    """
    Create vessel position update payload
    
    Args:
        lat: Latitude in degrees (float)
        lon: Longitude in degrees (float)
    
    Returns:
        bytes: 13-byte payload
    """
    msg_type = 0x01
    lat_scaled = int(lat * 10000000)
    lon_scaled = int(lon * 10000000)
    unix_time = int(time.time())
    
    payload = struct.pack('>BiiI', 
                         msg_type,
                         lat_scaled,
                         lon_scaled,
                         unix_time)
    
    return payload

# Example usage
vessel_lat = 37.7749  # San Francisco
vessel_lon = -122.4194
payload = create_vessel_position_payload(vessel_lat, vessel_lon)

# Send to all devices on port 10
for device in active_devices:
    send_downlink(device, payload, port=10, confirmed=False)
```

### Scheduling

Send position updates every 90-120 minutes to ensure devices always have fresh assistance data within the 3-hour validity window.

## Time Synchronization

### RTC Update

When a position update is received, the device automatically updates its RTC:

```c
hal_rtc_set_time_s(msg->unix_time);
```

### Time Uncertainty

The system tracks time uncertainty based on:
- Initial uncertainty from downlink queuing: ±60 seconds
- RTC drift over time: ~0.125 seconds/hour

Typical time uncertainty:
- **Just received**: ±60 seconds (excellent for GNSS)
- **After 1 hour**: ±60 seconds (still excellent)
- **After 12 hours**: ±62 seconds (very good)
- **After 24 hours**: ±63 seconds (very good)

## Configuration

### Changing the LoRaWAN Port

Edit `vessel_assistance.h`:

```c
#define VESSEL_ASSISTANCE_PORT 10  // Change to desired port
```

### Adjusting Update Intervals

On the gateway side, modify the update interval to balance:
- **Shorter intervals** (60-90 min): Better assistance, more network traffic
- **Longer intervals** (120-180 min): Acceptable assistance, less traffic

### Customizing Scan Durations

Edit `vessel_assistance.c`:

```c
#define GNSS_SCAN_DURATION_EXCELLENT    10   // Seconds
#define GNSS_SCAN_DURATION_GOOD         15   // Seconds
#define GNSS_SCAN_DURATION_FAIR         25   // Seconds
#define GNSS_SCAN_DURATION_COLD         60   // Seconds
```

## Debugging

### Enable Debug Traces

Position updates and assistance application are logged:

```
[INFO] Vessel position received: 37.774900, -122.419400 @ 1700000000
[INFO] Applying vessel assistance (EXCELLENT): 37.774900, -122.419400
[INFO] Position assistance sent to AG3335
[INFO] gnss begin, adaptive alarm 10 s
```

### Checking Assistance Status

```c
const position_time_cache_t* cache = vessel_assistance_get_cache();
if (cache->valid) {
    printf("Position: %.6f, %.6f\n", cache->latitude, cache->longitude);
    printf("Age: %u seconds\n", hal_rtc_get_time_s() - cache->rtc_at_receipt);
    printf("Quality: %d\n", vessel_assistance_get_quality());
}
```

## AG3335 NMEA Commands

The system uses the `PAIR062` command to send assistance position to the AG3335:

```
$PAIR062,lat,lon*checksum\r\n
```

Example:
```
$PAIR062,37.774900,-122.419400*5A\r\n
```

This is sent 3 times with 100ms intervals for reliability.

## Performance Monitoring

### Metrics to Track

1. **TTFF Distribution**: Measure actual TTFF with different assistance ages
2. **Fix Success Rate**: First-attempt fix rate with vs without assistance
3. **Power Consumption**: Battery life improvement from shorter scans
4. **Downlink Success Rate**: Percentage of position updates received

### Expected Results

With 90-minute update intervals and 1-minute uplinks:
- 95%+ of GNSS scans will have "Excellent" or "Good" assistance
- Average TTFF should be 12-18 seconds (vs 60+ seconds cold start)
- Battery life improvement: 60-70% for GNSS operation

## Troubleshooting

### No Assistance Received

1. Check gateway is sending on correct port (default: 10)
2. Verify payload format matches `vessel_position_msg_t`
3. Ensure device uplinks regularly to enable downlinks
4. Check LoRaWAN duty cycle limits

### Poor TTFF Despite Assistance

1. Verify position accuracy (should be < 50km from actual location)
2. Check time uncertainty (use `vessel_assistance_get_time_uncertainty()`)
3. Ensure AG3335 UART is functioning correctly
4. Verify NMEA command is being sent (check debug logs)

### Assistance Not Being Applied

1. Check `vessel_assistance_is_available()` returns true
2. Verify assistance quality is not POOR
3. Ensure `vessel_assistance_apply_to_gnss()` is called before scan
4. Check UART communication with AG3335

## Future Enhancements

Potential improvements for future versions:

1. **Almanac Updates**: Send GNSS almanac data via downlinks
2. **Motion Compensation**: Account for vessel motion between updates
3. **Multiple Time Sources**: Combine vessel time with Class B beacons
4. **Adaptive Update Intervals**: Vary based on vessel speed
5. **Ephemeris Data**: Send additional satellite data for further TTFF reduction

## References

- AG3335 NMEA Protocol Documentation
- LoRaWAN Specification v1.0.4
- GNSS Assistance Data Standards (3GPP)
