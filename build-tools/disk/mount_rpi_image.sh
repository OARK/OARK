#!/bin/bash

# Mounts a Raspberry Pi image.
#
# usage: mount_rpi_image <image> <mount point>

# Copyright (c) 2015 SEP 17

set -euo pipefail
IFS=$'\n\t'

# Include the common functions.
source ./disk_utils.sh

# Displays the help text
help() {
    echo "mount_rpi_image.sh - Mount a Raspberry Pi image."
    echo
    echo "    mount_rpi_image.sh <image> <mount point>"
    echo "    mount_rpi_image.sh help - This help text."
}

main() {
  if [ $# -eq 0 ]; then
    help
  else
    if [[ $(id -u) != 0 ]]; then
      echo "Must be root." >&2
    elif [[ ! -d $2 ]]; then
      echo "Mount point $2 does not exist." >&2
    else
      if [[ -f $1 ]]; then
        if disk_utils::is_disk_image $1; then
          IFS=' ' read -a sectorSizeArray <<< \
             "$(disk_utils::get_starting_sectors $1)"

          if [[ "${#sectorSizeArray[@]}" = 2 ]]; then
            # We only care about the second parition that holds the
            # OS.
            mount -t ext4 -o loop,offset=$(disk_utils::get_byte_offset \
                                             ${sectorSizeArray[1]}) $1 $2
          else
            echo "Not exactly two partitions, is it a standard Pi image?" &>2
          fi
        else
          echo "File is ${boot_sector_test}" &>2
          echo "Doesn't start with DOS/MBR boot sector, not Raspbian image?" &>2
        fi
      else
        echo "$1 isn't a file." &>2
      fi
    fi
  fi
}

main "$@"
