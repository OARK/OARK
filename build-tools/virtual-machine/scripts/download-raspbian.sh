#!/bin/bash

# Download the latest Raspbian image.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

IMAGE_DATE="2015-05-07"
IMAGE_NAME="2015-05-05-raspbian-wheezy.zip"

VAGRANT_RASPBIAN_FILE=/vagrant/files/${IMAGE_NAME}

echo -e "${COLOUR_PROGRESS}Downloading Raspbian image.${NC}"

cd /home/vagrant/Projects/oark/build-tools/emulation

# If we have the Raspbian image in the vagrant files, copy that
# across.

wget -c https://downloads.raspberrypi.org/raspbian/images/raspbian-${IMAGE_DATE}/${IMAGE_NAME}

unzip ${IMAGE_NAME}
