# Development Software

To build the software into an image that's usable on the hardware, a
collection of scripts and other software is used to try and make the
process as automated as possible.

These can be found under the ''build-tools''.

There are two approaches, one is to use install Vagrant and either
virtualbox or VMWare and run ''vagrant up'' in the
''build-tools/virtual-machine'' directory.

Or the other is to execute the build scripts locally on your machine,
either using emulation, or a real Raspberry Pi.

## Needed Packages

On Debian/Ubuntu: ''sudo apt-get install git qemu-system-arm''

## Emulation

To remove the dependency on having a hardware Pi setup, QEMU is used
to provide an emulated system. ''qemu-system-arm'' must be installed
on the system.

The script ''start.sh'' takes the filename of the Raspberry Pi image
to boot off. However, since there is no Raspberry Pi emulation
available for QEMU, it relies on a kernel being specifically compiled
for QEMU to use instead of the Raspberry Pi image kernel.

''download_raspbian.sh'' can be run to get the latest release of
Raspbian.

## Kernel

This contains the scripts for building the kernel for use with QEMU,
cd into the ''kernel'' directory and run ''./setup.sh build'', it will
download the toolchain, kernel source, and build the kernel.
