#!/bin/bash

# Given an OARK installed image running under QEMU, revert the changes
# made so it will work on a real Pi.

set -euo pipefail
IFS=$'\n\t'

revertLdPreload() {
  sed -i -e 's|[#]||g' $1/etc/ld.so.preload
}

revertUdevDisks() {
  rm $1/etc/udev/rules.d/90-qemu.rules
}

revertUdevDisks $1
revertLdPreload $1
