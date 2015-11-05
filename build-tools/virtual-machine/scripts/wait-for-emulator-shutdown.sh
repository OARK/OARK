#!/bin/bash

# Script to check for QEMU running.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
export COLOUR_SUCCESS='\033[0;32m'
export COLOUR_PROGRESS='\033[0;33m'
export NC='\033[0m' # No Colour

echo -e "${COLOUR_PROGRESS}Waiting for emulator to shutdown...${NC}"

while pgrep qemu-system-arm
do
  echo -e "${COLOUR_PROGRESS}Emulator still running.${NC}"
  sleep 10
  echo -e "${COLOUR_PROGRESS}Checking if emulator is running...${NC}"
done

echo -e "${COLOUR_SUCCESS}Emulator shutdown.${NC}"
