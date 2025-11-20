# T1000-E Tracker Firmware Releases

## Latest Version: v1.0.0-b1

### Quick Links
- **Latest HEX**: [t1000_e_tracker_latest.hex](t1000_e_tracker_latest.hex)
- **Latest UF2**: [t1000_e_tracker_latest.uf2](t1000_e_tracker_latest.uf2)

## Building Firmware

Use the automated build script:

```bash
./build_firmware.sh
```

This will:
1. Extract version from `firmware_version.h`
2. Build the firmware using SEGGER Embedded Studio
3. Create versioned `.hex` and `.uf2` files
4. Create symlinks to latest versions

## Flashing Firmware

### Method 1: UF2 Bootloader (Easiest)

1. Double-press reset button to enter bootloader mode
2. Drag and drop `t1000_e_tracker_vX.X.X.uf2` to the USB drive
3. Device will automatically flash and reboot

### Method 2: J-Link/SWD

1. Connect J-Link debugger
2. Use SEGGER Embedded Studio or nrfjprog
3. Flash `t1000_e_tracker_vX.X.X.hex`

## Version History

### v1.0.0-b1 (2025-11-20)

**Features:**
- ✨ Vessel position and time assistance for GNSS
- ✨ 13-byte downlink payload on LoRaWAN port 10
- ✨ Adaptive GNSS scan duration (10-60s based on assistance quality)
- ✨ Charging-based almanac maintenance (zero battery cost)
- ✨ Position assistance via AG3335 PAIR062 commands
- ✨ Own fix storage as fallback assistance

**Performance:**
- Expected TTFF: 10-25 seconds (with vessel assistance)
- Cold start TTFF: 60+ seconds (without assistance)
- Battery savings: 60-70% for GNSS operation

**Files:**
- `t1000_e_tracker_v1.0.0-b1.hex` - Intel HEX format
- `t1000_e_tracker_v1.0.0-b1.uf2` - UF2 bootloader format

## Updating Version

To create a new firmware version:

1. Edit `t1000_e/tracker/inc/firmware_version.h`
2. Update version numbers and changelog
3. Run `./build_firmware.sh`
4. Commit the new firmware files

Example version update:

```c
#define FIRMWARE_VERSION_MAJOR      1
#define FIRMWARE_VERSION_MINOR      1
#define FIRMWARE_VERSION_PATCH      0
#define FIRMWARE_VERSION_BUILD      1

#define FIRMWARE_VERSION_STRING     "1.1.0-b1"
#define FIRMWARE_VERSION_FEATURES   "Your new features here"
```

## File Naming Convention

- `t1000_e_tracker_vMAJOR.MINOR.PATCH-bBUILD.hex` - Versioned HEX file
- `t1000_e_tracker_vMAJOR.MINOR.PATCH-bBUILD.uf2` - Versioned UF2 file
- `t1000_e_tracker_latest.hex` - Symlink to latest HEX
- `t1000_e_tracker_latest.uf2` - Symlink to latest UF2

## Troubleshooting

### Build fails
- Ensure SEGGER Embedded Studio is installed at `/Applications/SEGGER/SEGGER Embedded Studio 8.24/`
- Check that nRF5 SDK is at `/Users/ja/nRF5_SDK_17.1.0_ddde560/`

### UF2 conversion fails
- Ensure Python 3 is installed
- Check that `firmware/uf2conv.py` exists

### Bootloader not detected
- Try double-pressing reset button
- Check USB cable (must support data, not just power)
- Verify bootloader is installed on device
