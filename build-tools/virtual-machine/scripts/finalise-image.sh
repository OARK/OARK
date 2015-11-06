#!/bin/bash

# Script to finalise the image, so it's ready to be copied to a SD
# card onto a real Pi.

# Copy of Raspbian image is expected in vagrant files dir.
# Hardcoded for 2015-05-05-raspbian-wheezy.img at the moment.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

IMAGE_NAME="2015-05-05-raspbian-wheezy.img"
OARK_IMAGE="oark.img"

cd /home/vagrant/Projects/oark/build-tools/emulation

echo -e "${COLOUR_PROGRESS}Mounting image...${NC}"
# Image needs to be modified before emulator can run it.
# Directory for mounting image.
mkdir -p temp

cd ../disk

sudo ./mount_rpi_image.sh ../emulation/$IMAGE_NAME ../emulation/temp
echo -e "${COLOUR_SUCCESS}Image mounted.${NC}"

cd ../emulation

echo -e "${COLOUR_PROGRESS}Reverting image...${NC}"
sudo ./revert_raspbian_image.sh
echo -e "${COLOUR_SUCCESS}Image reverted.${NC}"

echo -e "${COLOUR_PROGRESS}Unmounting image...${NC}"
sudo umount temp
echo -e "${COLOUR_SUCCESS}Image unmounted.${NC}"

# Copy the image into the vagrant files folder.

echo -e "${COLOUR_PROGRESS}Copy image to files/ directory...${NC}"
NOW=$(date +"%Y-%m-%d")

cp ${IMAGE_NAME} "/vagrant/files/${NOW}-${OARK_IMAGE}"
echo -e "${COLOUR_SUCCESS}Image ${OARK_IMAGE} copied.${NC}"
