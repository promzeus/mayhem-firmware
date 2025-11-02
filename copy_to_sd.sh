#!/bin/bash

# Script to copy ALL built apps and firmware to SD card
# Usage: ./copy_to_sd.sh

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Build directory
BUILD_DIR="$(dirname "$0")/build/firmware"

# Source paths
APPS_SRC="$BUILD_DIR/application"
FIRMWARE_FILE="$BUILD_DIR/portapack-mayhem-firmware.bin"

echo -e "${BLUE}=== Portapack Mayhem SD Card Copy Script ===${NC}"
echo ""

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${RED}Error: Build directory not found at $BUILD_DIR${NC}"
    echo "Please build the project first: ./dockerize.sh -j10"
    exit 1
fi

# Count available files
PPMA_COUNT=$(find "$APPS_SRC" -maxdepth 1 -name "*.ppma" 2>/dev/null | wc -l)
PPMP_COUNT=$(find "$BUILD_DIR/standalone" -name "*.ppmp" 2>/dev/null | wc -l)

echo -e "${GREEN}✓ Found build files:${NC}"

if [ -f "$FIRMWARE_FILE" ]; then
    echo "  Firmware: portapack-mayhem-firmware.bin ($(ls -lh "$FIRMWARE_FILE" | awk '{print $5}'))"
else
    echo -e "  ${YELLOW}Warning: Firmware not built yet${NC}"
fi

echo "  External apps (.ppma): $PPMA_COUNT files"
echo "  Standalone apps (.ppmp): $PPMP_COUNT files"
echo ""

# Try to find SD card mount point
SD_CARD=""
if [ -d "/Volumes/NO NAME" ]; then
    SD_CARD="/Volumes/NO NAME"
elif [ -d "/Volumes/PORTAPACK" ]; then
    SD_CARD="/Volumes/PORTAPACK"
elif [ -d "/media/$USER/NO NAME" ]; then
    SD_CARD="/media/$USER/NO NAME"
elif [ -d "/media/$USER/PORTAPACK" ]; then
    SD_CARD="/media/$USER/PORTAPACK"
else
    # Ask user for SD card path
    echo -e "${YELLOW}SD card not auto-detected. Common locations:${NC}"
    echo "  macOS:  /Volumes/NO NAME"
    echo "  Linux:  /media/\$USER/NO NAME"
    echo ""
    read -p "Enter SD card mount point: " SD_CARD

    if [ ! -d "$SD_CARD" ]; then
        echo -e "${RED}Error: Directory $SD_CARD not found${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}✓ SD card found at: $SD_CARD${NC}"
echo ""

# Create APPS directory if it doesn't exist
APPS_DIR="$SD_CARD/APPS"
if [ ! -d "$APPS_DIR" ]; then
    echo -e "${YELLOW}Creating APPS directory...${NC}"
    mkdir -p "$APPS_DIR" 2>/dev/null || sudo mkdir -p "$APPS_DIR"
fi

# Create FIRMWARE directory if it doesn't exist
FIRMWARE_DIR="$SD_CARD/FIRMWARE"
if [ ! -d "$FIRMWARE_DIR" ]; then
    echo -e "${YELLOW}Creating FIRMWARE directory...${NC}"
    mkdir -p "$FIRMWARE_DIR" 2>/dev/null || sudo mkdir -p "$FIRMWARE_DIR"
fi

# Copy firmware file
if [ -f "$FIRMWARE_FILE" ]; then
    echo -e "${YELLOW}Copying firmware to FIRMWARE folder...${NC}"
    if cp "$FIRMWARE_FILE" "$FIRMWARE_DIR/portapack-mayhem.bin" 2>/dev/null; then
        echo -e "${GREEN}✓ Firmware copied successfully${NC}"
    else
        echo -e "${YELLOW}Retrying with sudo...${NC}"
        sudo cp "$FIRMWARE_FILE" "$FIRMWARE_DIR/portapack-mayhem.bin"
        echo -e "${GREEN}✓ Firmware copied successfully (with sudo)${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Skipping firmware (not built)${NC}"
fi
echo ""

# Copy all external apps (.ppma files)
echo -e "${YELLOW}Copying external apps (.ppma) to APPS folder...${NC}"
COPIED_PPMA=0
FAILED_PPMA=0

for ppma_file in "$APPS_SRC"/*.ppma; do
    if [ -f "$ppma_file" ]; then
        filename=$(basename "$ppma_file")
        if cp "$ppma_file" "$APPS_DIR/" 2>/dev/null || sudo cp "$ppma_file" "$APPS_DIR/" 2>/dev/null; then
            echo "  ✓ $filename"
            ((COPIED_PPMA++))
        else
            echo -e "  ${RED}✗ $filename${NC}"
            ((FAILED_PPMA++))
        fi
    fi
done

if [ $COPIED_PPMA -gt 0 ]; then
    echo -e "${GREEN}✓ Copied $COPIED_PPMA external apps${NC}"
else
    echo -e "${YELLOW}⚠ No external apps found${NC}"
fi
echo ""

# Copy all standalone apps (.ppmp files)
echo -e "${YELLOW}Copying standalone apps (.ppmp) to APPS folder...${NC}"
COPIED_PPMP=0
FAILED_PPMP=0

for ppmp_file in "$BUILD_DIR"/standalone/*/*.ppmp; do
    if [ -f "$ppmp_file" ]; then
        filename=$(basename "$ppmp_file")
        if cp "$ppmp_file" "$APPS_DIR/" 2>/dev/null || sudo cp "$ppmp_file" "$APPS_DIR/" 2>/dev/null; then
            echo "  ✓ $filename"
            ((COPIED_PPMP++))
        else
            echo -e "  ${RED}✗ $filename${NC}"
            ((FAILED_PPMP++))
        fi
    fi
done

if [ $COPIED_PPMP -gt 0 ]; then
    echo -e "${GREEN}✓ Copied $COPIED_PPMP standalone apps${NC}"
else
    echo -e "${YELLOW}⚠ No standalone apps found${NC}"
fi
echo ""

# Summary
echo -e "${GREEN}=== Copy Complete ===${NC}"
echo ""
echo -e "${BLUE}Summary:${NC}"
[ -f "$FIRMWARE_DIR/portapack-mayhem.bin" ] && echo "  Firmware: ✓ portapack-mayhem.bin ($(ls -lh "$FIRMWARE_DIR/portapack-mayhem.bin" | awk '{print $5}'))"
echo "  External apps: $COPIED_PPMA files"
echo "  Standalone apps: $COPIED_PPMP files"
[ $FAILED_PPMA -gt 0 ] || [ $FAILED_PPMP -gt 0 ] && echo -e "  ${RED}Failed: $(($FAILED_PPMA + $FAILED_PPMP)) files${NC}"
echo ""

if [ -f "$FIRMWARE_DIR/portapack-mayhem.bin" ]; then
    echo -e "${GREEN}✓ All files copied successfully!${NC}"
    echo ""
    echo -e "${YELLOW}=== NEXT STEPS ===${NC}"
    echo ""
    echo "1. ${GREEN}Flash firmware:${NC}"
    echo "   - Safely eject SD card"
    echo "   - Insert into Portapack H2"
    echo "   - Go to: Settings → Firmware → Flash from SD"
    echo "   - Select: FIRMWARE/portapack-mayhem.bin"
    echo "   - Wait for flash and reboot"
    echo ""
    echo "2. ${GREEN}All apps are now available in the menu!${NC}"
    echo "   - New app: FT8 RX (in RX menu)"
    echo "   - Plus all other external & standalone apps"
else
    echo -e "${YELLOW}⚠ Apps copied, but firmware not available${NC}"
    echo "   Build firmware first: ./dockerize.sh -j10"
fi
echo ""
