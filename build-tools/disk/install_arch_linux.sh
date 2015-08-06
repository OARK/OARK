#!/bin/bash

# Given an image file and an Arch Linux .tgz file, will mount and
# install Arch.

# Usage: install_arch_linux.sh <mount point> <image_file> <arch_archive>

# Copyright (c) 2015 Open Academic Robot Kit

set -euo pipefail
IFS=$'\n\t'

# Our common functions
source ../common-functions.sh
source ./disk_utils.sh

help() {
  echo "install_arch_linux.sh - Install Arch Linux onto image."
  echo
  echo "    install_arch_linux.sh <mount point> <image_file>"
}

# Mount system partition
# $1 - Mount point
# $2 - Image file to mount
mount_system() {
  local sector_size_array
  IFS=' ' read -a sector_size_array <<< \
     "$(disk_utils::get_starting_sectors "$2")"

  echo -e "${COLOUR_PROGRESS}Mounting system partition...${NC}"
  mkdir -p "$1"
  mount -t ext4 -o loop,offset=$(disk_utils::get_byte_offset \
                                        ${sector_size_array[1]}) $2 $1
  echo -e "${COLOUR_SUCCESS}System partition mounted.${NC}"

  echo -e "${COLOUR_PROGRESS}Mounting boot partition...${NC}"
  # Mount the boot partition
  mkdir -p "$1/boot"
  mount -t vfat -o loop,offset=$(disk_utils::get_byte_offset \
                                        ${sector_size_array[0]}) $2 $1/boot
  echo -e "${COLOUR_SUCCESS}Boot partition mounted.${NC}"

  # Download the latest Arch
  # wget http://archlinuxarm.org/os/ArchLinuxARM-rpi-2-latest.tar.gz -c
  tar zxf ArchLinuxARM-rpi-2-latest.tar.gz -C "$1"

  # Bootstrap Arch so it works under QEMU, changes will have to be undone so it will work under a normal Raspberry Pi.
  # Comment out the boot partition, it won't boot otherwise.
  sed -e '/\/dev\/mmcblk0p1.*/ s/^#*/#/' -i "$1/etc/fstab"
  # Copy across "ethernet-dhcp"
  cp "$1/etc/netctl/examples/ethernet-dhcp" "$1/etc/netctl/"
  # Set up the systemd links and netctl links.
  cd "$1/etc/systemd/system/multi-user.target.wants/"
  ln -s "./../../../etc/systemd/system/netctl@ethernet\x2ddhcp.service"

  # Copy across SSH keys.
  cd ../../../../..

  mkdir -p "$1/root/.ssh"
  chmod 700 "$1/root/.ssh"
  cp "id_rsa.pub" "$1/root/.ssh/authorized_keys"

  # Unmount
  umount "$1/boot" "$1"
}

main() {
  if [ $# -eq 0 ]; then
    help
  else
    echo -e "${COLOUR_PROGRESS}Starting"
    # No point to continue unless we're root.
    if [[ $(id -u) != 0 ]]; then
      echo "Must be root." >&2
    else
      if [[ -f $2 ]]; then
        if disk_utils::is_disk_image "$2"; then
          mount_system "$1" "$2"
        else
          echo "$2 is not a disk image."
        fi
      else
        echo "$2 doesn't exist."
      fi
    fi
  fi
}

main "$@"
