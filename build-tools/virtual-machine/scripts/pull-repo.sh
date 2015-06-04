#!/bin/bash

# Pull in the OARK repo if we haven't already, or update the existing
# one.

set -euo pipefail
IFS=$'\n\t'

mkdir -p Projects
cd Projects

# set -e will exit whole script on command failure.
# This gets around it for this command.
test_for_repo=0
hg status oark || test_for_repo=$?

if [ $test_for_repo -eq 0 ]; then
    cd oark
    hg pull
    hg up
else
    hg clone https://sep-17.kilnhg.com/Code/Open-Academic-Robot-Kit/Group/devel oark
fi
