#!/bin/bash

# T1000-E USB Serial Connection Script
# Automatically finds and connects to T1000-E tracker

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}T1000-E Tracker - USB Serial Debug Console${NC}"
echo -e "${BLUE}==========================================${NC}"
echo ""

# Find T1000-E USB port
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)

if [ -z "$PORT" ]; then
    echo -e "${RED}Error: T1000-E not found${NC}"
    echo ""
    echo "Available serial ports:"
    ls -l /dev/cu.* 2>/dev/null || echo "  (none found)"
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo "  1. Check USB cable is connected"
    echo "  2. Ensure cable supports data (not just power)"
    echo "  3. Try unplugging and replugging USB"
    echo "  4. Press reset button on T1000-E"
    exit 1
fi

echo -e "${GREEN}Found T1000-E on: $PORT${NC}"
echo -e "${GREEN}Baud rate: 115200${NC}"
echo ""

# Check for preferred terminal programs
if command -v picocom &> /dev/null; then
    TERMINAL="picocom"
    echo -e "${YELLOW}Using picocom (best CR/LF handling)${NC}"
    echo -e "${YELLOW}To exit: Press Ctrl-A then Ctrl-X${NC}"
elif command -v minicom &> /dev/null; then
    TERMINAL="minicom"
    echo -e "${YELLOW}Using minicom${NC}"
    echo -e "${YELLOW}To exit: Press Ctrl-A then X${NC}"
else
    TERMINAL="screen"
    echo -e "${YELLOW}Using screen (add 'crlf on' to ~/.screenrc for better CR/LF)${NC}"
    echo -e "${YELLOW}To exit: Press Ctrl-A then K${NC}"
fi

echo -e "${YELLOW}To reset device: Press the reset button${NC}"
echo ""
echo "Connecting in 2 seconds..."
sleep 2

# Configure port with proper settings (convert LF to CRLF)
stty -f "$PORT" 115200 cs8 -cstopb -parenb onlcr

# Connect using best available terminal
case "$TERMINAL" in
    picocom)
        picocom -b 115200 --omap crlf "$PORT"
        ;;
    minicom)
        minicom -D "$PORT" -b 115200
        ;;
    *)
        screen "$PORT" 115200
        ;;
esac
