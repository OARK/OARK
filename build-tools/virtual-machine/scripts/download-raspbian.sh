#!/bin/bash

# Download the latest Raspbian image.
#
# It will check if it's running under Vagrant, and copy the image file
# (if present) from the files directory. It will then try to do a
# resume download of the image from the Raspberry Pi site.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

IMAGE_DATE="2015-05-07"
IMAGE_NAME="2015-05-05-raspbian-wheezy.zip"

VAGRANT_RASPBIAN_FILE=/vagrant/files/${IMAGE_NAME}
EMULATION_DIRECTORY=/home/vagrant/Projects/oark/build-tools/emulation

if [ -e ${VAGRANT_RASPBIAN_FILE} ]; then
  echo -e "${COLOUR_PROGRESS}Copying Raspbian image from vagrant files...${NC}"
  cp ${VAGRANT_RASPBIAN_FILE} ${EMULATION_DIRECTORY}
  echo -e "${COLOUR_SUCCESS}Copy success.${NC}"
fi

cd ${EMULATION_DIRECTORY}

# If we have the Raspbian image in the vagrant files, copy that
# across.

echo -e "${COLOUR_PROGRESS}Downloading Raspbian image.${NC}"
wget -nv -c https://downloads.raspberrypi.org/raspbian/images/raspbian-${IMAGE_DATE}/${IMAGE_NAME}
echo -e "${COLOUR_SUCCESS}Download success.${NC}"

echo -e "${COLOUR_PROGRESS}Unzipping image.${NC}"
unzip ${IMAGE_NAME}
echo -e "${COLOUR_SUCCESS}Unzip completed.${NC}"
