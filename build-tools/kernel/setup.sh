#!/bin/bash

# Initialises the kernel source, downloading and applying any patches as
# necessary.

# The kernel image should be read only, this is because we are using the sources
# to build two kernels. One kernel that is for use with QEMU, and the other that
# is for the actual image to run on Raspberry Pi hardware.

# Copyright (c) 2015 SEP 17

set -euo pipefail
IFS=$'\n\t'

source ../common-functions.sh

script_directory="$(common::get_script_directory)/kernel"

toolchain_directory() {
  echo "$script_directory/toolchain"
}

kernel_directory() {
  echo "$script_directory/linux"
}

build_directory="$script_directory/build/qemu/linux"

# Clone the given Git repo if the directory doesn't exist, otherwise
# try to update to the latest version. Will not consider if the repo
# has been modified, merging it needed, etc.
#
# A hard reset is performed, because to apply the kernel patch, we're
# assuming unmodified sources. We could probably just ignore the error
# raised when patching.

# $1 - Directory to repo.
# $2 - URL to upstream repo.
# $3 - Misc options when pulling updates.
pullGitRepo() {
  if [[ ! -e $1 ]]; then
    if [ $# -eq 3 ]; then
      git clone "$3" "$2" "$1"
    else
      git clone "$2" "$1"
    fi
  else
    git -C "$1" pull
    git -C "$1" reset --hard
  fi
}

# Be sure we have the cross compilation toolchain.
# This assumes no modification to repo from upstream.
ensureToolchain() {
  echo -e "${COLOUR_PROGRESS}Fetching toolchain...${NC}"
  pullGitRepo "$(toolchain_directory)" https://github.com/raspberrypi/tools
}

# When called, will pull the latest Raspberry Pi kernel.
ensurePiKernel() {
  echo -e "${COLOUR_PROGRESS}Fetching kernel source...${NC}"
  pullGitRepo "$(kernel_directory)" https://github.com/raspberrypi/linux --depth=1
}

# For QEMU, we use the Vexpress machine for emulation. It seems that
# compilation for this machine is broken in the Raspberry Pi Linux
# kernel sources. So get the normal Linux source.
#
# Expects the kernel version number to be passed in for which kernel
# to download.
ensureVanillaKernel() {
  echo -e "${COLOUR_PROGRESS}Removing old kernel sources...${NC}"
  rm -fr "$(kernel_directory)"
  echo -e "${COLOUR_PROGRESS}Fetching vanilla kernel source...${NC}"
  wget "http://www.kernel.org/pub/linux/kernel/v3.0/linux-$1.tar.xz" -c \
       -P "$script_directory"
  tar xf "$script_directory/linux-$1.tar.xz" \
      -C "$script_directory"
  mv "$script_directory/linux-$1" "$script_directory/linux"
}

# Setup cross compile path
setup_cross_compile_path() {
  export PATH=$(toolchain_directory)/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
}

# Setup the kernel build process
setup_kernel_build() {
  cd "$(kernel_directory)"
  make O="$build_directory" ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
       vexpress_defconfig
}

# Build the QEMU kernel
buildQEMUKernel() {
  echo -e "${COLOUR_PROGRESS}Building kernel...${NC}"

  cd "$build_directory"
  make "-j$(nproc)" ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

  cp arch/arm/boot/zImage ../qemu-kernel

  make "-j$(nproc)" ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
       INSTALL_MOD_PATH=../modules modules_install
}

# Turn on the settings we require in the kernel config.
#
# $1 - The config file to patch.
#
# Expects to be in the directory of the sed file.
patchKernelConfig() {
  sed -i -rf "$script_directory/vexpress-config.sed" "$1"
}

# Displays the help text.
help() {
    echo "setup.sh - Simple script for building a Raspberry Pi image for QEmu"
    echo
    echo "    setup.sh       - Run without arguments, will download toolchain and Linux kernel."
    echo "    setup.sh build - Will download toolchain and kernel and build the image."
    echo "    setup.sh help  - This help text."
}

# No parameters to script, assume getting build tools only.
if [ $# -eq 0 ]; then
  ensureToolchain
  ensureVanillaKernel 3.18.16
elif [ "$1" = "build" ]; then
  ensureToolchain
  ensureVanillaKernel 3.18.16

  setup_cross_compile_path
  setup_kernel_build

  patchKernelConfig "$build_directory/.config"
  buildQEMUKernel
  echo -e "${COLOUR_SUCCESS}Kernel built successfully.${NC}"
else
  help
fi
