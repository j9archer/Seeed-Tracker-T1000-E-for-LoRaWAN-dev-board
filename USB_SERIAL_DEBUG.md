# Debugging T1000-E via USB Serial Console

## Overview

The T1000-E firmware includes USB CDC (Communications Device Class) support, which allows you to view debug logs via a USB serial connection - **no J-Link required!**

## Quick Start

### 1. Connect USB Cable

Connect your T1000-E to your Mac via USB (make sure it's a data cable, not just power).

### 2. Find the Serial Port

```bash
ls -l /dev/cu.usbmodem*
```

You should see something like:
```
/dev/cu.usbmodem14201
```

### 3. Open Serial Terminal

**Option A: Using `screen` (built-in on macOS)**
```bash
screen /dev/cu.usbmodem14201 115200
```

**Option B: Using `minicom`**
```bash
# Install if needed
brew install minicom

# Connect
minicom -D /dev/cu.usbmodem14201 -b 115200
```

**Option C: Using `cu`**
```bash
cu -l /dev/cu.usbmodem14201 -s 115200
```

### 4. Reset Device

Press the reset button on the T1000-E to see the boot banner:

```
========================================
T1000-E Tracker Firmware v1.0.0-b1
Features: Vessel GNSS assistance + charging almanac
========================================

###### ===== T1000-E Tracker example ==== ######

Application parameters:
  - LoRaWAN uplink Fport = 2
  - Confirmed uplink     = No
```

## What You'll See

### Startup Logs

```
[INFO] Vessel assistance system initialized
[INFO] ========================================
[INFO] T1000-E Tracker Firmware v1.0.0-b1
[INFO] Features: Vessel GNSS assistance + charging almanac
[INFO] ========================================
[INFO] ###### ===== T1000-E Tracker example ==== ######
```

### Vessel Assistance Logs

When a downlink is received:
```
[INFO] Vessel position received: 37.774900, -122.419400 @ 1700000000
```

When GNSS scan starts:
```
[INFO] Applying vessel assistance (EXCELLENT): 37.774900, -122.419400
[INFO] Position assistance sent to AG3335
[INFO] gnss begin, adaptive alarm 10 s
```

### Charging Detection

When plugged in:
```
[INFO] Charging detected - scheduling almanac maintenance
[INFO] gnss begin, adaptive alarm 750 s
```

### GNSS Fix

```
[INFO] GNSS fix obtained: 37.774900, -122.419400
[INFO] Stored own GNSS fix as assistance: 37.774900, -122.419400
```

## Serial Terminal Commands

### Exit `screen`
- Press `Ctrl-A` then `K` (kill)
- Or `Ctrl-A` then `\` (quit)

### Exit `minicom`
- Press `Ctrl-A` then `X` (exit)

### Exit `cu`
- Type `~.` (tilde followed by period)

## Troubleshooting

### No serial port found

**Check USB connection:**
```bash
ls -l /dev/cu.*
```

**Check if device is detected:**
```bash
system_profiler SPUSBDataType | grep -A 10 "T1000"
```

### Port exists but no output

1. **Verify baud rate:** Must be 115200
2. **Try resetting device:** Press reset button
3. **Check USB cable:** Must support data, not just charging
4. **Replug USB:** Sometimes enumeration fails

### Garbage characters

- Wrong baud rate (should be 115200)
- Try 8N1 (8 data bits, no parity, 1 stop bit)

### Permission denied

```bash
sudo chmod 666 /dev/cu.usbmodem*
```

## Advanced: Logging to File

### Using `screen`
```bash
# Start screen with logging
screen -L -Logfile tracker.log /dev/cu.usbmodem14201 115200
```

### Using `cat`
```bash
# Redirect to file
cat /dev/cu.usbmodem14201 > tracker.log &

# Stop logging
killall cat
```

### Using `tee`
```bash
# View AND log simultaneously
cat /dev/cu.usbmodem14201 | tee tracker.log
```

## Automated Connection Script

Create `connect_tracker.sh`:

```bash
#!/bin/bash

# Find T1000-E USB port
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)

if [ -z "$PORT" ]; then
    echo "Error: T1000-E not found"
    echo "Available ports:"
    ls -l /dev/cu.*
    exit 1
fi

echo "Connecting to T1000-E on $PORT at 115200 baud..."
echo "Press Ctrl-A then K to exit"
echo ""

screen "$PORT" 115200
```

Make it executable:
```bash
chmod +x connect_tracker.sh
./connect_tracker.sh
```

## Debug Levels

The firmware uses different log levels:

- `HAL_DBG_TRACE_INFO` - Informational messages
- `HAL_DBG_TRACE_WARNING` - Warnings
- `HAL_DBG_TRACE_ERROR` - Errors
- `HAL_DBG_TRACE_PRINTF` - General debug output

All are enabled by default in the build configuration.

## Monitoring Vessel Assistance

To verify your vessel assistance implementation is working:

1. **Connect serial terminal**
2. **Flash firmware** with `t1000_e_tracker_v1.0.0-b1.uf2`
3. **Watch for initialization:**
   ```
   [INFO] Vessel assistance system initialized
   ```
4. **Send downlink** on port 10 (13 bytes)
5. **Look for confirmation:**
   ```
   [INFO] Vessel position received: XX.XXXXXX, YY.YYYYYY @ timestamp
   ```
6. **Wait for next GNSS scan:**
   ```
   [INFO] Applying vessel assistance (EXCELLENT): XX.XXXXXX, YY.YYYYYY
   [INFO] Position assistance sent to AG3335
   [INFO] gnss begin, adaptive alarm 10 s
   ```

## Real-Time Monitoring

**Watch for charging detection:**
```
[INFO] Charging detected - scheduling almanac maintenance
[INFO] gnss begin, adaptive alarm 750 s
```

This confirms the charging-based almanac maintenance is working!

## Notes

- Baud rate: **115200**
- Data format: **8N1** (8 data bits, no parity, 1 stop bit)
- Flow control: **None**
- USB CDC is enabled automatically when USB is connected
- Debug output works even without J-Link
- All `HAL_DBG_TRACE_*` calls go to USB serial
