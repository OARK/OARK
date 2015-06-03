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

updateLdPreload $1
updateUdevDisks $1
