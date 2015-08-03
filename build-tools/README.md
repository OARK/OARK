# Build Tools

As the name says, tools for building the project. There are three different ways that the system can be built, using Vagrant, locally on a Linux machine, or remotely onto an actual Raspberry Pi.

# Vagrant

The easiest and most automatic way is to install [Vagrant](https://www.vagrantup.com/) (and either VirtualBox or VMWare), `cd` into the `virtual-machine` directory, and run `vagrant up`. Check the README.md in the `virtual-machine` directory for details.

# Locally

Vagrant just provides a consistent Linux environment for these local install scripts to work. If you have a Linux distribution that is largely compatible, then you can build locally on your machine without needing to run the virtual machine. However, root access will be needed, since the image mounting needs root access. It expects the following things to be installed:

* Linux system (tested on Arch, but should work on any 64-bit system)
* git
* qemu-system-arm
* make

If you are running an Ubuntu or Debian system, then looking at the `virtual-machine/scripts/bootstrap.sh` file will give an indication of the packages needed.

Any paths of the scripts that are given, are relative to the `build-tools` directory.

The scripts use `sudo` for requesting root access, so be sure that is installed and setup for the account that runs the scripts.

## QEmu Kernel

QEmu will read the kernel from a file that we provide. Since QEmu doesn't support a Raspberry Pi system type, we have to use a modified kernel. Check the `kernel` directory for details. If you haven't got a kernel built on your machine (in the `kernel/build/qemu` directory), then the `start.sh` script will try to download the toolchain and Linux kernel sources to build it.

QEmu doesn't support a Raspberry Pi (1 or 2) machine type, so a vexpress machine type is used with an ARMv7 cpu (since that is what Arch Linux supports for the Pi 2). To use this board, a kernel has to be compiled since the supplied kernel will not work from the image. Check the `kernel` directory for details.

## Starting emulation

To start emulation:

`emulation/start.sh <image path>`

# Remote install

`Ansible` is used for scripting any changes needed for building the image from the Pi itself. The same playbooks are used for the QEMU emulated system, or for a remote install to a real Pi. A stock ARM Arch Linux install is expected, which means root access is allowed from SSH using `root` as the password, it will create a user account with password-less sudo for further access.
