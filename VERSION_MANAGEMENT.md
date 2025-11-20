# Version Management Quick Reference

## Current Version: v1.0.0-b1

## Updating the Version

### 1. Edit the Version Header

Edit `t1000_e/tracker/inc/firmware_version.h`:

```c
#define FIRMWARE_VERSION_MAJOR      1    // Breaking changes
#define FIRMWARE_VERSION_MINOR      0    // New features
#define FIRMWARE_VERSION_PATCH      0    // Bug fixes
#define FIRMWARE_VERSION_BUILD      1    // Build number

#define FIRMWARE_VERSION_STRING     "1.0.0-b1"
#define FIRMWARE_VERSION_FEATURES   "Brief feature description"
```

Add a changelog entry in the header file comments.

### 2. Build with Version

```bash
./build_firmware.sh
```

This automatically:
- Extracts version from header
- Builds firmware
- Creates `firmware/t1000_e_tracker_v1.0.0-b1.hex`
- Creates `firmware/t1000_e_tracker_v1.0.0-b1.uf2`
- Updates symlinks to latest

### 3. Commit and Tag

```bash
git add -A
git commit -m "Release v1.0.0-b1: Your changes here"
git tag -a v1.0.0-b1 -m "Version 1.0.0 build 1"
git push origin main --tags
```

## Versioning Scheme

### Format: MAJOR.MINOR.PATCH-bBUILD

- **MAJOR**: Incompatible API changes, major new features
- **MINOR**: Backward-compatible new features
- **PATCH**: Backward-compatible bug fixes
- **BUILD**: Build number (increments for each build)

### Examples

- `1.0.0-b1` - First build of version 1.0.0
- `1.1.0-b1` - Added new features (vessel assistance)
- `1.1.1-b1` - Bug fix release
- `2.0.0-b1` - Major rewrite or breaking changes

## Version History Template

When releasing a new version, update `firmware_version.h` with:

```c
/*
 * Version History:
 * 
 * v1.1.0-b1 (2025-XX-XX)
 *   - New feature A
 *   - New feature B
 *   - Fixed bug C
 * 
 * v1.0.0-b1 (2025-11-20)
 *   - Added vessel position and time assistance for GNSS
 *   - Implemented 13-byte downlink payload on port 10
 *   - Added adaptive GNSS scan duration
 *   - Implemented charging-based almanac maintenance
 */
```

## Build Artifacts

After running `./build_firmware.sh`:

```
firmware/
├── t1000_e_tracker_v1.0.0-b1.hex      # Versioned HEX
├── t1000_e_tracker_v1.0.0-b1.uf2      # Versioned UF2
├── t1000_e_tracker_latest.hex         # Symlink → v1.0.0-b1.hex
└── t1000_e_tracker_latest.uf2         # Symlink → v1.0.0-b1.uf2
```

## Boot Banner

The firmware prints version info on startup:

```
========================================
T1000-E Tracker Firmware v1.0.0-b1
Features: Vessel GNSS assistance + charging almanac
========================================
```

## Manual Build (without script)

If you need to build manually:

```bash
# Build
"/Applications/SEGGER/SEGGER Embedded Studio 8.24/bin/emBuild" \
  -config "Release" \
  pca10056/s140/11_ses_lorawan_tracker/t1000_e_dev_kit_pca10056.emProject

# Convert to UF2
python3 firmware/uf2conv.py -c -f 0xADA52840 \
  -o firmware/output.uf2 \
  pca10056/s140/11_ses_lorawan_tracker/Output/Release/Exe/t1000_e_dev_kit_pca10056.hex
```

## Git Workflow

### For feature development:
```bash
# Increment minor version
# Edit firmware_version.h: 1.0.0 → 1.1.0
./build_firmware.sh
git add -A
git commit -m "feat: Add new feature X"
git tag -a v1.1.0-b1 -m "Release 1.1.0-b1"
```

### For bug fixes:
```bash
# Increment patch version
# Edit firmware_version.h: 1.0.0 → 1.0.1
./build_firmware.sh
git add -A
git commit -m "fix: Fix bug in feature Y"
git tag -a v1.0.1-b1 -m "Release 1.0.1-b1"
```

### For rebuilds (same version):
```bash
# Increment build number only
# Edit firmware_version.h: 1.0.0-b1 → 1.0.0-b2
./build_firmware.sh
```
