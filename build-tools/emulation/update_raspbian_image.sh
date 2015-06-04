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

addSSHKeys() {
    mkdir -p $1/home/pi/.ssh
    cp ~/.ssh/id_rsa.pub $1/home/pi/.ssh/authorized_keys

    chmod 700 $1/home/pi/.ssh
    chmod 644 $1/home/pi/.ssh/authorized_keys

    chown -R 1000:1000 $1/home/pi/.ssh
}

updateLdPreload $1
updateUdevDisks $1
