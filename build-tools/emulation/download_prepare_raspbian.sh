#!/bin/bash

# Download the Raspbian image, mount it, and do the necessary changes
# for making it work under QEMU.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
GREEN=`tput setaf 2`
ORANGE=`tput setaf 3`
NC=`tput sgr0` # No Color

wget http://downloads.raspberrypi.org/raspbian_latest -c -O raspbian_latest.zip

unzip -o raspbian_latest.zip

echo "${ORANGE}Creating mountpoint directory.${NC}"
mkdir -p raspbian_mount

echo "${ORANGE}Mounting image...${NC}"
sudo ../disk/mount_rpi_image.sh 2015-05-05-raspbian-wheezy.img raspbian_mount

echo "${ORANGE}Updating Raspbian for QEMU...${NC}"
sudo ./update_raspbian_image.sh raspbian_mount/

echo "${ORANGE}Unmounting image...${NC}"
sudo umount raspbian_mount

echo "${GREEN}Success.${NC}"
