#!/bin/bash

# Script bootstrap the Raspberry Pi image on the emulator and start
# building the software for the actual project.

set -euo pipefail
IFS=$'\n\t'

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

# Not getting picked up from .bashrc?
# TODO: Must fix properly.
ANSIBLE_INVENTORY=~/.ansible_hosts

# Run a playbook when given just the name. Assumes directory structure
# in homedir.
runAnsiblePlaybook() {
  ansible-playbook -i ${ANSIBLE_INVENTORY} "$HOME/ansible_playbooks/$1"
}

echo -e "${COLOUR_PROGRESS}Installing SSH Key${NC}"
echo "raspberry" | ansible-playbook -k -i ${ANSIBLE_INVENTORY} "$HOME/ansible_playbooks/01-init.yaml"

echo -e "${COLOUR_PROGRESS}Slimming Raspbian image...${NC}"
runAnsiblePlaybook 02-slimline_install.yaml
echo -e "${COLOUR_SUCCESS}Raspbian image slimmed.${NC}"
