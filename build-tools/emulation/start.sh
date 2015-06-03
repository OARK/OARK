#!/bin/bash

# Given a Raspbian image, start a QEMU instance and use that image. If the QEmu
# compatible kernel has not been built, it will run kernel/setup.sh script to
# try and build it.
#
# Be warned, this will try to get the toolchain and linux kernel.

set -euo pipefail
IFS=$'\n\t'

# Local port for SSH access into the emulated system.
SSH_PORT=10022

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

# Will check to see if the kernel file exists, if it doesn't, it will try to
# build the kernel.
buildKernel() {
    if [[ ! -f $(getScriptDirectory)/../kernel/build/qemu/qemu-kernel ]]; then
        echo "Building QEMU Kernel"
        $(getScriptDirectory)/../kernel/setup.sh build
    fi
}

buildKernel

qemu-system-arm -machine versatilepb \
                -cpu arm1176 -m 256 -no-reboot -serial stdio \
                -kernel $(getScriptDirectory)/../kernel/build/qemu/qemu-kernel \
                -append "root=/dev/sda2 panic=1 rw" \
                -hda $1 \
                -net user,hostfwd=tcp::${SSH_PORT}-:22 \
                -net nic
