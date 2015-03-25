#!/bin/bash

# Initialises the kernel source, downloading and applying any patches as
# necessary.

# The kernel image should be read only, this is because we are using the sources
# to build two kernels. One kernel that is for use with QEMU, and the other that
# is for the actual image to run on Raspberry Pi hardware.

# I want to be sure that operations are done relative to this scripts directory,
# not from current directory someone may have when running it.
#
# http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in

set -euo pipefail
IFS=$'\n\t'

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

pullGitRepo() {
    if [[ ! -e $1 ]]; then
        git clone $3 $2 $1
    else
        git -C $1 pull
    fi
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
}

ensureToolchain
ensureKernel
