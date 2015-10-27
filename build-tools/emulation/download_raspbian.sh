#!/bin/bash

wget http://downloads.raspberrypi.org/raspbian_latest -c -O raspbian_latest.zip

unzip raspbian_latest.zip

mkdir raspbian_mount
sudo ../disk/mount_rpi_image.sh 2015-05-05-raspbian-wheezy.img raspbian_mount
sudo ./update_raspbian_image.sh raspbian_mount/
sudo umount raspbian_mount
