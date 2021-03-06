#!/bin/bash

# Initialises the kernel source, downloading and applying any patches as
# necessary.

# The kernel image should be read only, this is because we are using the sources
# to build two kernels. One kernel that is for use with QEMU, and the other that
# is for the actual image to run on Raspberry Pi hardware.

# Copyright (c) 2015 SEP 17

set -euo pipefail
IFS=$'\n\t'

# I want to be sure that operations are done relative to this scripts directory,
# not from current directory someone may have when running it.
#
# http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in
getScriptDirectory() {
    local SOURCE="${BASH_SOURCE[0]}"
    while [ -h "$SOURCE" ]; do
        # Resolve $SOURCE until the file is no longer a symlink
        local DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
        local SOURCE="$(readlink "$SOURCE")"
         # If $SOURCE was a relative symlink, we need to resolve it relative to
         # the path where the symlink file was located
        [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
    done
    local DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

    echo $DIR
}

toolChainDirectory() {
    echo $(getScriptDirectory)/toolchain
}

kernelDirectory() {
    echo $(getScriptDirectory)/linux
}

# Clone the given Git repo if the directory doesn't exist, otherwise try to
# update to the latest version.
# Will not consider if the repo has been modified, merging it needed, etc.
#
# A hard reset is performed, because to apply the kernel patch, we're assuming
# unmodified sources. We could probably just ignore the error raised when
# patching.

# $1 - Directory to repo.
# $2 - URL to upstream repo.
# $3 - Misc options when pulling updates.
pullGitRepo() {
    if [[ ! -e $1 ]]; then
        if [ $# -eq 3 ]; then
            git clone $3 $2 $1
        else
            git clone $2 $1
        fi
    else
        git -C $1 pull
        git -C $1 reset --hard
    fi
}

# Different commits don't seem to compile well, since we're pulling from the development repo.
# Assumes repo already exists.
#
# $1 - Directory of repo.
# $2 - Commit revision no.
switchGitRepoToCommit() {
  git -C $1 reset --hard
}

# Be sure we have the cross compilation toolchain.
# This assumes no modification to repo from upstream.
ensureToolchain() {
    echo "Toolchain"
    pullGitRepo $(toolChainDirectory) https://github.com/raspberrypi/tools
}

ensureKernel() {
    echo "Kernel"
    pullGitRepo $(kernelDirectory) https://github.com/raspberrypi/linux --depth=1
    switchGitRepoToCommit $(kernelDirectory)
}

# Patch kernel
#
# QEMU doesn't support the Raspberry Pi as a system, so instead we use the
# versatile system. However, the kernel config options don't give us the
# necessary options, this patch enables them.
patchKernel() {
    echo "Patch kernel"
    patch -p1 -d $(kernelDirectory) < $(getScriptDirectory)/linux-arm-qemu.patch
}

# Build the QEMU kernel
buildQEMUKernel() {
    export PATH=$(toolChainDirectory)/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH

    local oldDirectory=$(getScriptDirectory)
    local buildDirectory=$(getScriptDirectory)/build/qemu/linux

    cd $(kernelDirectory)

    make O=$buildDirectory ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- versatile_defconfig

    cp $oldDirectory/qemu-kernel.config $buildDirectory/.config

    cd $buildDirectory

    make -j`nproc` ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

    cp arch/arm/boot/zImage ../qemu-kernel

    make -j`nproc` ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- INSTALL_MOD_PATH=../modules modules_install
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
    ensureKernel
elif [ "$1" = "build" ]; then

    ensureToolchain
    ensureKernel

    patchKernel
    buildQEMUKernel
else
    help
fi
