#!/bin/bash

# Common functions for bash scripts to use for Raspberry Pi based
# disk image files.

# Copyright (c) 2015 Open Academic Robot Kit

set -euo pipefail
IFS=$'\n\t'

# Size of sectors in image
readonly SECTOR_SIZE=512

# Find the starting sectors of each partition in a disk image.
function disk_utils::get_starting_sectors() {
  echo $(file -kb $1 | xargs -n1 -d',' | awk '/startsector/ {print $2}')
}

# Get the total number of sectors in the partition.
function disk_utils::get_total_sectors() {
  echo $(file -kb $1 | xargs -n1 -d',' | awk '/sectors/ {print $1}')
}

# Using a loopback mount command, we need the byte offset into the
# file. Given a sector, this function will return the bytes. Assumes
# sector size of 512.
function disk_utils::get_byte_offset() {
  echo $(($1 * SECTOR_SIZE))
}

# Create loopback device.
# This requires super user access (it will call sudo) and that loop0
# isn't already used.
# $1 - Loopback device to use.
# $2 - Image file name.
# $3 - Which partition to mount (1 or 2)
function disk_utils::mount_image_raw() {
  local starting_sector_array
  IFS=' ' read -a starting_sector_array <<< \
     "$(disk_utils::get_starting_sectors $2)"

  local total_sectors
  IFS=' ' read -a total_sectors <<< \
     "$(disk_utils::get_total_sectors $2)"

  local partition_index
  partition_index=$(($3 - 1))

  local partition_size
  partition_size=$((${total_sectors[partition_index]} * ${SECTOR_SIZE}))

  echo "Partition size: ${partition_size}"

  sudo /sbin/losetup -o $(disk_utils::get_byte_offset \
                            ${starting_sector_array[partition_index]}) \
       --sizelimit ${partition_size} $1 $2
}

# Confirm that the file given is a disk image. Takes a file name,
# returns 0 if it's a partitioned disk image file, 1 otherwise.
#
# It's expected that all file checks have been done, (exists,
# readable, etc)
function disk_utils::is_disk_image() {
  local boot_sector_test
  boot_sector_test="$(file -kb $1 | awk -F ';' '{print $1}')"
  if [[ "${boot_sector_test}" == "DOS/MBR boot sector" ]] ||
     [[ "${boot_sector_test}" == "x86 boot sector" ]]; then
    return 0
  else
    return 1
  fi
}
