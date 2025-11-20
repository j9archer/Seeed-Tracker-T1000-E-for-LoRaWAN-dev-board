#!/bin/bash

# T1000-E Firmware Build Script
# Builds the tracker firmware and creates versioned output files

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Paths
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMPROJECT="pca10056/s140/11_ses_lorawan_tracker/t1000_e_dev_kit_pca10056.emProject"
BUILD_OUTPUT="pca10056/s140/11_ses_lorawan_tracker/Output/Release/Exe"
FIRMWARE_DIR="firmware"
VERSION_HEADER="t1000_e/tracker/inc/firmware_version.h"

# SEGGER Embedded Studio path
EMBUILD="/Applications/SEGGER/SEGGER Embedded Studio 8.24/bin/emBuild"

# Check if emBuild exists
if [ ! -f "$EMBUILD" ]; then
    echo -e "${RED}Error: SEGGER Embedded Studio not found at $EMBUILD${NC}"
    exit 1
fi

# Extract version from header file
if [ ! -f "$VERSION_HEADER" ]; then
    echo -e "${RED}Error: Version header not found: $VERSION_HEADER${NC}"
    exit 1
fi

VERSION_STRING=$(grep "#define FIRMWARE_VERSION_STRING" "$VERSION_HEADER" | awk '{print $3}' | tr -d '"')

if [ -z "$VERSION_STRING" ]; then
    echo -e "${RED}Error: Could not extract version string from $VERSION_HEADER${NC}"
    exit 1
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}T1000-E Firmware Build${NC}"
echo -e "${BLUE}Version: $VERSION_STRING${NC}"
echo -e "${BLUE}========================================${NC}"

# Clean old build
echo -e "${GREEN}Cleaning previous build...${NC}"
"$EMBUILD" -config "Release" -clean "$EMPROJECT"

# Build firmware
echo -e "${GREEN}Building firmware...${NC}"
"$EMBUILD" -config "Release" "$EMPROJECT"

if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

# Check if hex file was created
HEX_FILE="$BUILD_OUTPUT/t1000_e_dev_kit_pca10056.hex"
if [ ! -f "$HEX_FILE" ]; then
    echo -e "${RED}Error: Hex file not created${NC}"
    exit 1
fi

# Create firmware directory if it doesn't exist
mkdir -p "$FIRMWARE_DIR"

# Copy and rename hex file
VERSIONED_HEX="$FIRMWARE_DIR/t1000_e_tracker_v${VERSION_STRING}.hex"
cp "$HEX_FILE" "$VERSIONED_HEX"
echo -e "${GREEN}Created: $VERSIONED_HEX${NC}"

# Convert to UF2
VERSIONED_UF2="$FIRMWARE_DIR/t1000_e_tracker_v${VERSION_STRING}.uf2"
echo -e "${GREEN}Converting to UF2...${NC}"
python3 "$FIRMWARE_DIR/uf2conv.py" -c -f 0xADA52840 -o "$VERSIONED_UF2" "$HEX_FILE"

if [ $? -ne 0 ]; then
    echo -e "${RED}UF2 conversion failed!${NC}"
    exit 1
fi

# Get file sizes
HEX_SIZE=$(ls -lh "$VERSIONED_HEX" | awk '{print $5}')
UF2_SIZE=$(ls -lh "$VERSIONED_UF2" | awk '{print $5}')

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Build completed successfully!${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "Version:  ${GREEN}$VERSION_STRING${NC}"
echo -e "HEX file: ${GREEN}$VERSIONED_HEX${NC} ($HEX_SIZE)"
echo -e "UF2 file: ${GREEN}$VERSIONED_UF2${NC} ($UF2_SIZE)"
echo -e "${BLUE}========================================${NC}"

# Create symlinks for "latest" versions
ln -sf "$(basename "$VERSIONED_HEX")" "$FIRMWARE_DIR/t1000_e_tracker_latest.hex"
ln -sf "$(basename "$VERSIONED_UF2")" "$FIRMWARE_DIR/t1000_e_tracker_latest.uf2"

echo -e "${GREEN}Symlinks created for latest version${NC}"
echo -e "  → $FIRMWARE_DIR/t1000_e_tracker_latest.hex"
echo -e "  → $FIRMWARE_DIR/t1000_e_tracker_latest.uf2"
