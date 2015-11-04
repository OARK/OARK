#!/bin/bash

# Given a fresh Raspbian image, increase the size of the image, since
# it's a little small to hold everything we need.
#
# The partition will have to be deleted (taking note of the starting
# block) then re-added. resize2fs has to be run after a reboot so the
# new size is picked up.

# Usage: resize_rpi_image.sh <image file>

# Copyright (c) 2015 SEP 17

set -euo pipefail
IFS=$'\n\t'

# Resize the image, just adds 2G
resizeImage() {
    qemu-img-resize $1 +2G
}

resizeImage $1
