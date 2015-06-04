#!/bin/bash

# Given a fresh Raspbian image, update it so it will boot on QEMU.
#
# Usage: update_raspbian_image.sh <mounted image dir>

# The image can be mounted with the mount_rpi_image script in the disk/
# directory.

# By default, root access will be needed for this (since only root can mount).

# Copyright (c) 2015 SEP 17

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

updateLdPreload() {
    sed -i -e "/libcofi_rpi/s/^/#/g" $1/etc/ld.so.preload
}

updateUdevDisks() {
    cat <<EOF > $1/etc/udev/rules.d/90-qemu.rules
KERNEL=="sda", SYMLINK+="mmcblk0"
KERNEL=="sda?", SYMLINK+="mmcblk0p%n"
KERNEL=="sda2", SYMLINK+="root"
EOF
}

# Add SSH keys so we don't need a password to login from the host.
# This is to make scripting the guest VM easier.
addSSHKeys() {
    mkdir -p $1/home/pi/.ssh
    cp ~/.ssh/id_rsa.pub $1/home/pi/.ssh/authorized_keys

    chmod 700 $1/home/pi/.ssh
    chmod 644 $1/home/pi/.ssh/authorized_keys

    # User 1000 is the "pi" user account.
    chown -R 1000:1000 $1/home/pi/.ssh
}

echo -e "${COLOUR_PROGRESS}Updating ld.so.preload...${NC}"
updateLdPreload $1
echo -e "${COLOUR_SUCCESS}Update ld.so.preload, success.${NC}"

echo -e "${COLOUR_PROGRESS}Updating udev disks...${NC}"
updateUdevDisks $1
echo -e "${COLOUR_SUCCESS}Update udev disks, success.${NC}"

echo -e "${COLOUR_PROGRESS}Adding SSH keys to Pi image...${NC}"
addSSHKeys $1
echo -e "${COLOUR_SUCCESS}Update SSH keys to Pi, success.${NC}"
