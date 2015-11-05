#!/bin/bash

# Script to get the Raspberry Pi QEMU instance up and running.

# Copy of Raspbian image is expected in vagrant files dir.
# Hardcoded for 2015-05-05-raspbian-wheezy.img at the moment.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

IMAGE_NAME="2015-05-05-raspbian-wheezy.img"

echo -e "${COLOUR_PROGRESS}Copy Raspbian image...${NC}"
cd /home/vagrant/Projects/oark/build-tools/emulation

echo -e "${COLOUR_PROGRESS} Mounting image...${NC}"
# Image needs to be modified before emulator can run it.
# Directory for mounting image.
mkdir -p temp

cd ../disk

sudo ./mount_rpi_image.sh ../emulation/$IMAGE_NAME ../emulation/temp

cd ../emulation

echo -e "${COLOUR_PROGRESS}Updating image for emulation...${NC}"
sudo ./update_raspbian_image.sh temp

echo -e "${COLOUR_PROGRESS}Adding SSH key...${NC}"
sudo mkdir -p temp/home/pi/.ssh
sudo cp ~vagrant/.ssh/id_rsa.pub temp/home/pi/.ssh/authorized_keys
sudo chown -R 1000:1000 temp/home/pi/.ssh
sudo chmod 700 temp/home/pi/.ssh

sudo umount temp

./start.sh 2015-05-05-raspbian-wheezy.img
