#!/bin/bash

# Given a Raspbian image, start a QEMU instance and use that image. If the QEmu
# compatible kernel has not been built, it will run kernel/setup.sh script to
# try and build it.
#
# Be warned, this will try to get the toolchain and linux kernel.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

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
        echo -e "${COLOUR_PROGRESS}Building QEMU Kernel${NC}"
        $(getScriptDirectory)/../kernel/setup.sh build
    fi
}

# Displays the help text.
help() {
    echo "start.sh - Script for starting a QEMU instance for a Raspberry Pi image."
    echo
    echo "    Port ${SSH_PORT} on localhost will be forwarded to SSH port of guest."
    echo "    VNC port 5905 is available for gfx interface."
    echo
    echo "    start.sh <image file> - Start the emulator instance using image file."
}

# No parameters to script, then show help, since image to use must be
# provided.
if [ $# -eq 0 ]; then
    help
else
    buildKernel

    echo -e "${COLOUR_PROGRESS}Starting emulator...${NC}"

    # Start the emulation, there is no Raspberry Pi system in QEMU, so
    # have to use the VersatilePB system. Doesn't support multicore, or
    # more mem than 256.

    # Local port SSH_PORT is forwarded for SSH access, and VNC server is
    # on local port 5905 for gfx access if needed.

    # We redirect stderr to /dev/null because QEMU can't find some
    # audio devices under the Vagrant guest. This means to causes what
    # looks to be serious errors, but everything is fine.
    # If there are issues starting the emulator, remove the redirection.
    qemu-system-arm -machine versatilepb \
                    -cpu arm1176 -m 256 -no-reboot \
                    -daemonize \
                    -vnc :5 \
                    -kernel $(getScriptDirectory)/../kernel/build/qemu/qemu-kernel \
                    -append "root=/dev/sda2 panic=1 rw" \
                    -hda $1 \
                    -net user,hostfwd=tcp::${SSH_PORT}-:22 \
                    -net nic 2>/dev/null

    echo -e "${COLOUR_SUCCESS}Emulator started.${NC}"
fi
