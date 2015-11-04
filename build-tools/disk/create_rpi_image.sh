#!/bin/bash

# Creates a Raspberry Pi image.
#
# usage: create_rpi_image <image> <size>
#
# Will create a two partition image, with the boot partition being
# 100M, and the remaining space being the system partition.
#
# size should be an integer of the size of the image, in MB.
#
# Requires access to parted, but not super user access.

# Copyright (c) 2015 Open Academic Robot Kit

set -euo pipefail
IFS=$'\n\t'

# Include the common functions and constants.
source ./disk_utils.sh

# Boot and System partition numbers
readonly BOOT_PART_INDEX=1
readonly SYS_PART_INDEX=2

# For SD cards, you shouldn't always start at sector 0.
# TODO: BOOT_START_SECTOR is not a good name. It's the number of
# alignment sectors for best performance. At the moment, no testing
# has been done for the Raspberry Pi SD cards has been done.
#
# Check: http://rpitc.blogspot.com.au/p/sdcard-tuning.html
# https://thelastmaimou.wordpress.com/2013/05/19/optimizing-arch-linux-image-installation-on-the-raspberry-pi/

readonly BOOT_START_SECTOR=8192
readonly BOOT_END_SECTOR=204799
readonly SYSTEM_START_SECTOR=204800

# Given a filename and size in MB, create an empty disk image file.
function create_disk_image() {
  dd if=/dev/zero of="$1" bs=1M count="$2"
}

# Display the usage text.
help() {
  echo "create_rpi_image.sh - Create a blank disk image."
  echo
  echo "    create_rpi_image.sh <image> <size in MB>"
}

main() {
  # No parameters display the help text.
  if [ $# -eq 0 ]; then
    help
  else
    if [[ $(id -u) != 0 ]]; then
      echo "Must be root." >&2
    else
      # Find the first available loopback device.
      readonly LOOPBACK=$(/sbin/losetup -f)

      create_disk_image "$1" "$2"
      /sbin/parted "$1" mktable msdos

      # Create the boot partition.
      /sbin/parted "$1" mkpart p fat32 "${BOOT_START_SECTOR}s" "${BOOT_END_SECTOR}s"

      # Create the system partition
      /sbin/parted "$1" -- mkpart p ext4 "${SYSTEM_START_SECTOR}s" -4s

      # Mount the Boot partition and format it to FAT32
      disk_utils::mount_image_raw "${LOOPBACK}" "$1" "${BOOT_PART_INDEX}"
      sudo mkfs.vfat -F 32 -n BOOT "${LOOPBACK}"
      sudo /sbin/losetup -d "${LOOPBACK}"

      # Mount the System partition and format it.
      disk_utils::mount_image_raw "${LOOPBACK}" "$1" "${SYS_PART_INDEX}"

      # Also, we don't need huge file support, this is so we can mount
      # the FS on kernels without the option installed.
      # http://kuttler.eu/post/filesystem-with-huge-files-cannot-be-mounted-read-write-without-config_lbdaf/
      sudo mkfs.ext4 -b 4096 -O '^huge_file' -E stride=8,stripe-width=1024 -L ROOT "${LOOPBACK}"
      sudo /sbin/losetup -d "${LOOPBACK}"
    fi
  fi
}

main "$@"
