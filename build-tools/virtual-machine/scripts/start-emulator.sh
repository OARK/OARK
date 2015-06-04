#!/bin/bash

# Script to get the Raspberry Pi QEMU instance up and running.

set -euo pipefail
IFS=$'\n\t'

cd ~/Projects/oark/build-tools/emulation/

./download_prepare_raspbian.sh

./start.sh 2015-05-05-raspbian-wheezy.img
