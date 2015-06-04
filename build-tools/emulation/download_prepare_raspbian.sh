#!/bin/bash

# Download the Raspbian image, mount it, and do the necessary changes
# for making it work under QEMU.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${COLOUR_PROGRESS}Download and prepare Raspbian image."
echo -e "${COLOUR_PROGRESS}Starting download...${NC}"
wget -q http://downloads.raspberrypi.org/raspbian_latest -c -O raspbian_latest.zip
echo -e "${COLOUR_SUCCESS}Downloaded${NC}"

unzip -o raspbian_latest.zip

echo -e "${COLOUR_PROGRESS}Creating mountpoint directory.${NC}"
mkdir -p raspbian_mount

echo -e "${COLOUR_PROGRESS}Mounting image...${NC}"
sudo ../disk/mount_rpi_image.sh 2015-05-05-raspbian-wheezy.img raspbian_mount

echo -e "${COLOUR_PROGRESS}Updating Raspbian for QEMU...${NC}"
sudo ./update_raspbian_image.sh raspbian_mount/

echo -e "${COLOUR_PROGRESS}Unmounting image...${NC}"
sudo umount raspbian_mount

echo -e "${COLOUR_SUCCESS}Raspbian image downloaded and prepared for emulator.${NC}"
