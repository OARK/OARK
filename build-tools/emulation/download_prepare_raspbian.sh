#!/bin/bash

# Download the Raspbian image, mount it, and do the necessary changes
# for making it work under QEMU.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS=`tput setaf 2`
COLOUR_PROGRESS=`tput setaf 3`
NC=`tput sgr0` # No Color

wget http://downloads.raspberrypi.org/raspbian_latest -c -O raspbian_latest.zip

unzip -o raspbian_latest.zip

echo "${COLOUR_PROGRESS}Creating mountpoint directory.${NC}"
mkdir -p raspbian_mount

echo "${COLOUR_PROGRESS}Mounting image...${NC}"
sudo ../disk/mount_rpi_image.sh 2015-05-05-raspbian-wheezy.img raspbian_mount

echo "${COLOUR_PROGRESS}Updating Raspbian for QEMU...${NC}"
sudo ./update_raspbian_image.sh raspbian_mount/

echo "${COLOUR_PROGRESS}Unmounting image...${NC}"
sudo umount raspbian_mount

echo "${COLOUR_SUCCESS}Success.${NC}"
