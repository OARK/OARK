#!/bin/bash

# Mounts a Raspberry Pi image.
#
# usage: mount_rpi_image <image> <mount point>

# Copyright (c) 2015 SEP 17

set -euo pipefail
IFS=$'\n\t'

# Get the starting sectors of each partition in the image.
function getSectors()
{
    local result="$(file -b $1 | xargs -n1 -d',' | awk '/startsector/ {print $2}')"
    echo $result
}

# The mount command uses byte offsets, the default sector size is 512.
function getByteOffset()
{
    echo $(expr $1 \* 512)
}

# Displays the help text
help() {
    echo "mount_rpi_image.sh - Mount a Raspberry Pi image."
    echo
    echo "    mount_rpi_image.sh <image> <mount point>"
    echo "    mount_rpi_image.sh help - This help text."
}

if [[ $(id -u) != 0 ]]; then
    echo "Must be root."
elif [[ ! -d $2 ]]; then
    echo "Mount point $2 does not exist."
else
    if [[ -f $1 ]]; then
        if [[ "$(file -b $1 | awk -F ';' '{print $1}')" == "DOS/MBR boot sector" ]]; then

            IFS=' ' read -a sectorSizeArray <<< "$(getSectors $1)"

            if [[ "${#sectorSizeArray[@]}" = 2 ]]; then
                # We only care about the second parition that holds the OS.
                mount -t ext4 -o loop,offset=$(getByteOffset ${sectorSizeArray[1]}) $1 $2
            else
                echo "Not exactly two partitions, is it a standard Pi image?"
            fi
        fi
    else
        echo "$1 isn't a file."
    fi
fi
