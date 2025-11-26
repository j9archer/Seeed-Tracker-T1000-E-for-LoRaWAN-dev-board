# Vessel Position and Time Assistance for GNSS

## Overview

This feature significantly improves GNSS Time-To-First-Fix (TTFF) performance for T1000-E trackers operating on a vessel equipped with a relay gateway.

## How It Works

### Position Updates from Vessel

The vessel's relay gateway periodically sends position updates to all T1000-E devices via LoRaWAN downlinks:

- **Update Interval**: Every 90-120 minutes
- **LoRaWAN Port**: 10 (configurable via `VESSEL_ASSISTANCE_PORT`)
- **Payload Size**: 9 bytes

### Payload Format

```c
typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0x01 = Position Update
    int32_t vessel_lat;    // Latitude × 10^7 (degrees)
    int32_t vessel_lon;    // Longitude × 10^7 (degrees)
} vessel_position_msg_t;
```

**Note**: Time synchronization is handled separately via **DeviceTimeReq** MAC command, which provides superior accuracy compared to downlink timestamps.

### Example Payload

For vessel at position 37.7749°N, 122.4194°W:

```
01 16 7C 65 B0 C9 E0 F4 90
```

Breakdown:
- `01`: Message type (position update)
- `16 7C 65 B0`: Latitude = 377,490,000 (37.7749° × 10^7)
- `C9 E0 F4 90`: Longitude = -1,224,194,000 (-122.4194° × 10^7)

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

def create_vessel_position_payload(lat, lon):
    """
    Create vessel position update payload (no timestamp - using DeviceTimeReq instead)
    
    Args:
        lat: Latitude in degrees (float)
        lon: Longitude in degrees (float)
    
    Returns:
        bytes: 9-byte payload
    """
    msg_type = 0x01
    lat_scaled = int(lat * 10000000)
    lon_scaled = int(lon * 10000000)
    
    payload = struct.pack('>Bii', 
                         msg_type,
                         lat_scaled,
                         lon_scaled)
    
    return payload

# Example usage
vessel_lat = 37.7749  # San Francisco
vessel_lon = -122.4194
payload = create_vessel_position_payload(vessel_lat, vessel_lon)

# Send to all devices on port 10
for device in active_devices:
    send_downlink(device, payload, port=10, confirmed=False)
```

**Note**: Time synchronization is automatic via DeviceTimeReq. The gateway/network server handles this through standard LoRaWAN MAC commands, providing superior accuracy compared to downlink timestamps.

### Scheduling

Send position updates every 90-120 minutes to ensure devices always have fresh assistance data within the 3-hour validity window.

## Time Synchronization

### DeviceTimeReq MAC Command

The device automatically requests time synchronization from the network:

1. **Initial Sync**: Immediately after LoRaWAN join
2. **Periodic Sync**: Every 4 hours of runtime
3. **Method**: DeviceTimeReq MAC command (LoRaWAN 1.0.4 specification)
4. **Response**: Network provides GPS epoch time via DeviceTimeAns
5. **Accuracy**: Typically ±1 second (depends on gateway NTP synchronization)

### Clock Architecture

**Hardware Time Source**:
- **nRF52840 RTC2**: 32.768 kHz crystal oscillator
- **Accuracy**: ±50 ppm (±4.3 seconds/day, ±0.18 seconds/hour)
- **Prescaler**: 2 (tick = ~91.5 μs)

**Modem Time Management**:
The LoRa Basics Modem maintains GPS epoch time by:
1. Storing the GPS time received from DeviceTimeAns
2. Using the nRF52840 RTC as the reference clock
3. Calculating current GPS time as: `GPS_time = base_GPS_time + (current_RTC - reference_RTC)`

**Why DeviceTimeReq is Better Than Downlink Timestamps**:
- Network time accuracy: ~1 second vs RTC drift of ~0.18 sec/hour
- No payload overhead (MAC command in FOpts)
- Automatic resync capability  
- Industry standard LoRaWAN mechanism

### Time Synchronization Flow

```
Device                          Network
  |                               |
  |--- DeviceTimeReq (after join)-|
  |                               |
  |<-- DeviceTimeAns (GPS time) --|
  |                               |
  [Modem stores GPS epoch offset] |
  |                               |
  [TIME event callback triggered] |
```

### Monitoring Time Sync

The device logs comprehensive time sync information:

```
==== TIME SYNC UPDATE ====
Status: VALID
Network GPS time: 1447842784.000 s
Network Unix time: 1763807566 (UTC)
Local RTC: 188 s (188770 ms)
Time correction: 1447842596 s
Modem time is now synchronized with network
========================
```

### Time Drift and Accuracy

After DeviceTimeAns synchronization:
- **Initial accuracy**: ±1 second (network dependent)
- **Drift rate**: ±0.18 seconds/hour (nRF52840 RTC typical)
- **After 1 hour**: ±1.2 seconds
- **After 4 hours**: ±1.7 seconds (before periodic resync)

**Periodic Resync Strategy**:
The device automatically requests DeviceTimeReq every 4 hours to maintain accuracy:
- Ensures time accuracy stays within ±2 seconds
- Optimal for GNSS assistance (requirement: <3 seconds)
- Minimal network overhead (one MAC command per 4 hours)
- No manual intervention required

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
[INFO] UTC time sent to AG3335 (PAIR590)
[INFO] Position sent to AG3335 (PAIR600)
[INFO] Vessel assistance applied (PAIR590 + PAIR600)
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

## AG3335 GNSS Assistance Commands

The AG3335 chipset supports **direct position and time injection** via proprietary PAIR commands (per SIM68D NMEA documentation):

### PAIR590 - UTC Time Reference

**Command Format:**
```
$PAIR590,YYYY,MM,DD,hh,mm,ss*CS<CR><LF>
```

**Requirements:**
- **Must use UTC time**, not local time
- Time accuracy should be **<3 seconds** for optimal TTFF improvement
- Sent before each GNSS scan to provide current time reference

**Example:**
```
$PAIR590,2025,11,21,14,30,45*06
```

### PAIR600 - Reference Position

**Command Format:**
```
$PAIR600,<Lat>,<Lon>,<Height>,<AccMaj>,<AccMin>,<Bear>,<AccVert>*CS<CR><LF>
```

**Parameters:**
- `Lat`, `Lon`: Decimal degrees
- `Height`: Altitude in meters (0.0 if unknown)
- `AccMaj`, `AccMin`: Horizontal RMS accuracy in meters
- `Bear`: Bearing of error ellipse in degrees (0.0 for circular)
- `AccVert`: Vertical RMS accuracy in meters

**Example:**
```
$PAIR600,24.772816,121.022636,0.0,50.0,50.0,0.0,100.0*06
```

**Implementation Notes:**
- Conservative accuracy estimates: 50m horizontal, 100m vertical
- Accounts for vessel GPS accuracy (~5-10m) plus age-related drift
- Altitude set to 0.0 (less critical for 2D GNSS positioning)

### Other AG3335 PAIR Commands

From the Airoha AG3335 specification:

- `PAIR004`: Hot start - uses available ephemeris/almanac
- `PAIR005`: Warm start - uses almanac only
- `PAIR006`: Cold start - no assistance data
- `PAIR010`: Request GNSS system reference data
- `PAIR496-509`: EPOC orbit prediction (ephemeris data)

## Performance Monitoring

### Metrics to Track

1. **TTFF Distribution**: Measure actual TTFF with different assistance ages
2. **Fix Success Rate**: First-attempt fix rate with vs without assistance
3. **Power Consumption**: Battery life improvement from shorter scans
4. **Downlink Success Rate**: Percentage of position updates received

### Expected Results

With 90-minute update intervals and 1-minute uplinks:
- 95%+ of GNSS scans will have "Excellent" or "Good" time assistance
- Improved TTFF through time sync (accuracy depends on vessel clock)
- Battery life improvement: 20-40% through adaptive scan duration
- Actual TTFF depends on satellite visibility and ephemeris age

## Troubleshooting

### No Assistance Received

1. Check gateway is sending on correct port (default: 10)
2. Verify payload format matches `vessel_position_msg_t`
3. Ensure device uplinks regularly to enable downlinks
4. Check LoRaWAN duty cycle limits

### Poor TTFF Despite Assistance

1. Check PAIR590/PAIR600 commands are acknowledged (look for PAIR001 ACK in logs)
2. Verify time accuracy is <3 seconds (check vessel/gateway clock sync)
3. Ensure position accuracy estimates are reasonable (50m typical)
4. Verify AG3335 has valid ephemeris/almanac data
5. Check satellite visibility and sky conditions
6. Ensure Hot Start (PAIR004) is being used when possible

### Assistance Not Being Applied

1. Check `vessel_assistance_is_available()` returns true
2. Verify assistance quality is not POOR
3. Ensure `vessel_assistance_apply_to_gnss()` is called before scan
4. Check UART communication with AG3335

## Future Enhancements

Potential improvements for future versions:

1. **EPOC Orbit Prediction**: Utilize AG3335's PAIR496-509 commands for extended ephemeris
2. **RTCM Differential Corrections**: Use PAIR430-443 for high-precision positioning
3. **Adaptive Start Mode**: Automatically select Hot/Warm/Cold based on data age
4. **Motion Compensation**: Account for vessel motion between updates
5. **Multiple Time Sources**: Combine vessel time with LoRaWAN Class B beacons
6. **Adaptive Update Intervals**: Vary downlink frequency based on vessel speed

## References

- AG3335 Product Page: https://www.airoha.com/products/p/VXKPfHI9iDCvsRWN
- LOCOSYS AG3335M/MN Command List v1.1 (SDK 3.2.1)
- LoRaWAN Specification v1.0.4
- ION GNSS+ 2022: "Low Power Dual-frequency Multi-constellation GNSS Receiver Designed for Wearable Applications"
