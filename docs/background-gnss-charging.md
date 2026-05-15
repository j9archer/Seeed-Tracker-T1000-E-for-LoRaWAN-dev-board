# Background GNSS Mode (Charging/Docked)

## Overview

When the T1000-E is connected to a charger or docked, it automatically runs continuous GNSS tracking in the background. This keeps the almanac and ephemeris data fresh, ensuring fast Time-To-First-Fix (TTFF) when the device is later used for MOB/PIW tracking.

## How It Works

### Charge Detection

The firmware monitors the `CHARGER_ADC_DET` GPIO pin on each alarm event (~15-60 seconds):

```
Charger connected  → Start background GNSS (continuous tracking)
Charger removed    → Stop background GNSS (power down module)
```

### Benefits

| Condition | Without Background Mode | With Background Mode |
|-----------|------------------------|---------------------|
| Cold start TTFF | 30-60+ seconds | N/A (already tracking) |
| Warm start TTFF | 5-15 seconds | 1-3 seconds |
| Almanac freshness | Expires after 14 days | Always current |
| Ephemeris freshness | Expires after 4 hours | Always current |

### Coexistence with Normal Tracking

Background GNSS mode is designed to coexist with normal MOB/PIW tracking:

1. **Normal tracking requested** (MOB burst or PIW scan)
2. **Check**: Is background GNSS already active?
   - **YES**: Skip `gnss_scan_start()` - module already running
   - **NO**: Start GNSS normally
3. **Perform scan** (quality checks, fix retrieval)
4. **Check**: Is background GNSS active?
   - **YES**: Skip `gnss_scan_stop()` - keep module running
   - **NO**: Stop GNSS normally

This ensures no conflicts or redundant power cycling.

## API Functions

```c
// Check charge state and manage background mode (call periodically)
void gateway_assistance_check_charge_state(void);

// Check if background GNSS is currently active
bool gateway_assistance_is_background_gnss_active(void);

// Manual control (normally automatic via charge detection)
void gateway_assistance_start_background_gnss(void);
void gateway_assistance_stop_background_gnss(void);
```

## Power Considerations

- **On charge**: Power consumption is not a concern - GNSS runs continuously
- **On battery**: Background mode only activates when charging detected
- **NVRAM persistence**: Almanac/ephemeris saved to AG3335 NVRAM, survives power cycles

## Network Time Sync

When the device receives a `DeviceTimeAns` from the LoRaWAN network (in response to a `DeviceTimeReq`), and background GNSS is active, the firmware sends the updated time to the GNSS module via `PAIR590`.

**Why this matters:**
- If the GNSS has satellite visibility, it already has accurate time from the satellites - no harm done
- If the GNSS is running but has **no satellite visibility** (e.g., device is indoors/docked), the network time keeps the module's clock synchronized
- Accurate time is critical for ephemeris/almanac validity and reduces TTFF when satellites become visible

## Log Output

When charging is connected:
```
=== CHARGE CONNECTED - Starting background GNSS ===
Starting background GNSS mode (almanac/ephemeris maintenance)
GNSS: POWER_EN -> ON (scan_start)
Background GNSS mode ACTIVE - module running continuously
```

When charging is removed:
```
=== CHARGE DISCONNECTED - Stopping background GNSS ===
Stopping background GNSS mode
GNSS: POWER_EN -> OFF (scan_stop)
Background GNSS mode STOPPED
```

## Files Modified

- `gateway_assistance.h` - New function declarations
- `gateway_assistance.c` - Background mode implementation
- `marine_gnss.c` - Check for background mode before start/stop
- `main_lorawan_tracker.c` - Call charge state check in alarm handler
