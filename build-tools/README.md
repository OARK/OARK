# Build Tools

Here are the tools for building the project. At the moment it just has some scripts for setting up a QEmu environment that will boot a Raspbian image. Unfortunately it's not a no-brainer setup at the moment. It expects the following things to be already installed:

* Linux system (tested on Arch, but should work on any 64-bit system)
* git
* qemu
* make

Any paths of the scripts that are given, are relative to the `build-tools` directory.

`sudo` access will be necessary because the image will need to be mounted.

## Raspbian image

To boot, we need an operating system to boot too. First, you have to [download the Raspbian image](http://www.raspberrypi.org/downloads/).

Unzip the download, and you will have a *.img file, getting the image to work with QEmu requires modification of the image, so be sure you keep a copy of the unmodified image.

Create a directory, this will be the mount point for the image, then mount the image using:

`sudo disk/mount_rpi_image.sh <image name> <mount point>`

If you examine the mount point directory, you should see the Raspbian image filesystem. To run the necessary update for the Qemu booting:

`sudo emulation/update_raspbian_image.sh <mount point>`

This should update the necessary file. Finally unmount the image:

`sudo umount <mount point>`

That's it, the image should now be bootable with Qemu.

## QEmu Kernel

QEmu will read the kernel from a file that we provide. Since QEmu doesn't support a Raspberry Pi system type, we have to use a modified kernel. Check the `kernel` directory for details. If you haven't got a kernel built on your machine (in the `kernel/build/qemu` directory), then the `start.sh` script will try to download the toolchain and Linux kernel sources to build it.

## Starting emulation

To start emulation:

`emulation/start.sh <image path>`
