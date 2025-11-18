# Charging-Based Almanac Maintenance

## Overview

The vessel assistance module now includes intelligent almanac maintenance that triggers automatically when the device is charging. This ensures the GNSS almanac stays fresh without draining the battery during normal operation.

## How It Works

### Automatic Detection

The system monitors two conditions:
1. **Charging Status**: Is USB/charger connected?
2. **Almanac Age**: How long since last GNSS fix?

When **both** conditions are met:
- Device is charging (free power available)
- 14+ days since last GNSS fix (almanac likely stale)

The system automatically extends the next GNSS scan to **12.5 minutes** to allow full almanac download.

### Implementation

```c
static uint32_t app_get_adaptive_gnss_scan_duration(void)
{
    // Check if charging and almanac maintenance needed
    if (vessel_assistance_is_charging() && 
        vessel_assistance_needs_almanac_maintenance(14))
    {
        HAL_DBG_TRACE_INFO("Charging detected - scheduling almanac maintenance\n");
        return vessel_assistance_get_almanac_scan_duration();  // 750s = 12.5 min
    }
    
    // Otherwise use vessel assistance recommended duration
    return vessel_assistance_get_recommended_scan_duration();  // 10-60s
}
```

## Benefits

### Power Optimization

| Scenario | Battery Impact | Almanac Status |
|----------|---------------|----------------|
| **Charging almanac refresh** | Zero (on external power) | Always fresh |
| **Battery almanac refresh** | ~300mAh (12.5 min GNSS on) | Fresh but costly |
| **No almanac refresh** | Zero | Stale after weeks |

### User Experience

**Without charging-based maintenance:**
- Tracker unused for 30 days
- User takes it to sea
- First GNSS attempt: 12.5+ minute wait (frustrating!)

**With charging-based maintenance:**
- Tracker charges before departure (plugged in overnight)
- Almanac automatically refreshed during charging
- First GNSS attempt: 10-25 seconds (excellent!)

## Hardware Detection

The T1000-E has three charger status pins:

```c
#define CHARGER_ADC_DET 5   // P0.05 - USB/charger connected
#define CHARGER_DONE    36  // P1.04 - Charging complete  
#define CHARGER_CHRG    35  // P1.03 - Currently charging
```

The `vessel_assistance_is_charging()` function checks `CHARGER_ADC_DET`:

```c
bool vessel_assistance_is_charging(void)
{
    return (hal_gpio_get_value(CHARGER_ADC_DET) != 0);
}
```

This detects **any** USB/charger connection, regardless of battery charge state.

## Almanac Maintenance Threshold

The default threshold is **14 days** without a GNSS fix:

```c
vessel_assistance_needs_almanac_maintenance(14)  // 14 days
```

### Adjusting the Threshold

You can customize the threshold based on your needs:

```c
// More aggressive (fresher almanac, more frequent maintenance)
vessel_assistance_needs_almanac_maintenance(7)   // Weekly

// More conservative (less frequent maintenance)
vessel_assistance_needs_almanac_maintenance(30)  // Monthly

// Very conservative (only when really stale)
vessel_assistance_needs_almanac_maintenance(60)  // 2 months
```

**Recommendation:** 14-21 days is optimal for vessel trackers
- Almanac remains reasonably current
- Maintenance happens naturally during charging cycles
- Minimal user intervention required

## Almanac Download Duration

The system uses **750 seconds (12.5 minutes)** for almanac maintenance:

```c
uint32_t vessel_assistance_get_almanac_scan_duration(void)
{
    return 750;  // 12.5 minutes - full almanac download time
}
```

### Why 12.5 Minutes?

GPS almanac is broadcast in 25 subframes:
- Each subframe takes 30 seconds
- 25 subframes × 30 seconds = 12.5 minutes
- Requires continuous tracking of **one** satellite

**Indoor Consideration:**
- Near window: Usually succeeds in 12.5-15 minutes
- Deep indoor: May take 20-30 minutes (signal drops)
- Metal enclosure: May fail completely

## Typical Usage Scenarios

### Scenario 1: Overnight Charging

```
Day 0:  Tracker deployed on vessel
Day 15: Tracker brought home, plugged in overnight
        → Charging detected
        → 15 days since last fix
        → Almanac maintenance triggered
        → 12.5 minute GNSS scan during charging
        → Almanac refreshed (zero battery cost)
Day 16: Tracker returned to vessel with fresh almanac
        → First GNSS fix: 10-15 seconds
```

### Scenario 2: Dock Charging

```
Vessel at dock with shore power:
- Tracker on charge for 3 hours
- Almanac 20 days old
- Automatic maintenance during charge period
- Ready for next voyage with fresh almanac
```

### Scenario 3: Regular Use (No Maintenance Needed)

```
Day 0:  Deploy tracker
Day 5:  GNSS fix obtained (almanac refreshed naturally)
Day 10: Device charged
        → Charging detected
        → Only 5 days since last fix
        → NO maintenance needed (almanac still fresh)
        → Normal 10-25 second scans continue
```

## Debug Logging

When almanac maintenance is triggered:

```
[INFO] Charging detected - scheduling almanac maintenance
[INFO] gnss begin, adaptive alarm 750 s
[INFO] GNSS scan started for almanac download
```

After almanac download completes:

```
[INFO] GNSS fix obtained: 37.774900, -122.419400
[INFO] Storing own fix as vessel assistance fallback
```

## Integration with Vessel Position Assistance

The two systems work together synergistically:

### Fresh Almanac + Vessel Position
**TTFF: 8-12 seconds** (best case)
- Receiver knows satellite orbits (almanac)
- Receiver knows approximate position (vessel assistance)
- Minimal satellite search required

### Stale Almanac + Vessel Position
**TTFF: 30-60 seconds** (degraded but functional)
- Receiver must identify visible satellites
- Position helps narrow search space
- Works but slower than ideal

### Fresh Almanac + No Position
**TTFF: 25-40 seconds** (good)
- Standard warm start performance
- Better than cold start

### Stale Almanac + No Position
**TTFF: 5-15 minutes** (cold start)
- Must download almanac first
- Worst case scenario

## Configuration Examples

### Minimum Maintenance (Battery Priority)

```c
// Only refresh when really stale (30+ days)
if (vessel_assistance_is_charging() && 
    vessel_assistance_needs_almanac_maintenance(30))
{
    return vessel_assistance_get_almanac_scan_duration();
}
```

### Aggressive Maintenance (Performance Priority)

```c
// Refresh weekly when charging
if (vessel_assistance_is_charging() && 
    vessel_assistance_needs_almanac_maintenance(7))
{
    return vessel_assistance_get_almanac_scan_duration();
}
```

### Smart Maintenance with Fallback

```c
// Try during charging first, but also allow on-demand if critically stale
if (vessel_assistance_is_charging() && 
    vessel_assistance_needs_almanac_maintenance(14))
{
    // Opportunistic maintenance during charging
    return vessel_assistance_get_almanac_scan_duration();
}
else if (vessel_assistance_needs_almanac_maintenance(60))
{
    // Emergency refresh even on battery if >60 days stale
    HAL_DBG_TRACE_WARNING("Critical almanac staleness - forcing refresh\n");
    return vessel_assistance_get_almanac_scan_duration();
}
else
{
    // Normal operation
    return vessel_assistance_get_recommended_scan_duration();
}
```

## Power Consumption Analysis

### Almanac Download Power Cost

**On Battery:**
- GNSS active: ~35mA @ 3.3V = 115mW
- Duration: 750 seconds (12.5 min)
- Energy: 115mW × 12.5min = 24Wh = ~7.3mAh @ 3.3V

**On Charger:**
- Battery impact: Zero
- Power from USB: ~115mW for 12.5 minutes

### Long-term Battery Impact

**Without charging-based maintenance:**
```
Monthly deployment cycle:
- 1x battery-powered almanac refresh = 7.3mAh
- Annual cost: 12 × 7.3mAh = 88mAh
```

**With charging-based maintenance:**
```
Monthly deployment cycle:
- Almanac refreshed during charging = 0mAh
- Annual cost: 0mAh
- Savings: 88mAh/year
```

For a 1000mAh battery, this represents an **8.8% annual capacity savings**.

## Troubleshooting

### Almanac Maintenance Not Triggering

**Check charging detection:**
```c
if (vessel_assistance_is_charging()) {
    HAL_DBG_TRACE_INFO("Charger connected\n");
} else {
    HAL_DBG_TRACE_INFO("Not charging\n");
}
```

**Check almanac age:**
```c
if (vessel_assistance_needs_almanac_maintenance(14)) {
    HAL_DBG_TRACE_INFO("Almanac maintenance needed\n");
} else {
    HAL_DBG_TRACE_INFO("Almanac still fresh\n");
}
```

### Almanac Download Fails Indoors

**Solutions:**
1. Move tracker near window during charging
2. Increase download duration to 900s (15 minutes)
3. Accept that indoor charging may not refresh almanac
4. Rely on outdoor GNSS fixes to refresh almanac naturally

### Excessive Power During Charging

**Symptoms:**
- Device gets warm during charging
- Charging takes very long

**Cause:**
- GNSS running continuously at 35mA
- Normal behavior during almanac maintenance

**Solutions:**
- Ensure good ventilation during charging
- This is expected and safe
- Maintenance only happens once per 14+ days

## Future Enhancements

### Phase 2: Almanac via LoRaWAN

Instead of GNSS download, send almanac via downlinks:
- Gateway fetches almanac from internet
- Sends compressed almanac pages via LoRaWAN
- Works completely indoors
- All devices updated simultaneously

**Pros:**
- Works anywhere (no GNSS signal needed)
- Very power efficient (no GNSS radio)
- Guaranteed success

**Cons:**
- Requires ~30 downlinks (900 bytes total)
- More complex gateway implementation
- Network traffic overhead

## Summary

The charging-based almanac maintenance provides:

✅ **Zero battery cost** - Uses external power when charging  
✅ **Automatic operation** - No user intervention required  
✅ **Fresh almanac** - Maintains optimal GNSS performance  
✅ **Smart scheduling** - Only when needed (14+ day threshold)  
✅ **Seamless integration** - Works alongside vessel position assistance  

**Result:** Trackers always ready for deployment with minimal battery impact.
