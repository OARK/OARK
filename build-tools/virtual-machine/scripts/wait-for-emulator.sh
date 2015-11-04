#!/bin/bash

# Script to check for QEMU running.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
export COLOUR_SUCCESS='\033[0;32m'
export COLOUR_PROGRESS='\033[0;33m'
export NC='\033[0m' # No Colour

echo -e "${COLOUR_PROGRESS}Waiting for emulator...${NC}"

# It takes awhile for the emulator to start and for it to be
# responsive, so keep doing a simple looping test on SSH until it
# responds.
until ssh localhost -p 10022 exit
do
    echo -e "${COLOUR_PROGRESS}Not running, sleeping.${NC}"
    sleep 10

    echo -e "${COLOUR_PROGRESS}Checking if emulator image is running...${NC}"
done

echo -e "${COLOUR_SUCCESS}Emulator booted.${NC}"
