#!/bin/bash

# Install ROS and set up the Raspberry Pi for OARK.

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

# Will reboot after install, so wait for it to pop back up.
/vagrant/scripts/wait-for-emulator.sh

echo -e "${COLOUR_PROGRESS}Starting ROS install...${NC}"
runAnsiblePlaybook 03-ros_indigo_install.yaml
echo -e "${COLOUR_SUCCESS}ROS installed.${NC}"

echo -e "${COLOUR_PROGRESS}Starting gstreamer install...${NC}"
runAnsiblePlaybook 04-gstreamer_setup.yaml
echo -e "${COLOUR_SUCCESS}Gstreamer installed.${NC}"

echo -e "${COLOUR_PROGRESS}Starting Camera setup...${NC}"
runAnsiblePlaybook 05-setup_camera.yaml
echo -e "${COLOUR_SUCCESS}Camera setup.${NC}"

echo -e "${COLOUR_PROGRESS}Starting TFT Screen setup...${NC}"
runAnsiblePlaybook 06-setup_tft_screen.yaml
echo -e "${COLOUR_SUCCESS}TFT Screen setup.${NC}"

echo -e "${COLOUR_PROGRESS}Updating RPi firmware...${NC}"
runAnsiblePlaybook 07-update-firmware.yaml
echo -e "${COLOUR_SUCCESS}RPi firmware setup.${NC}"

echo -e "${COLOUR_PROGRESS}Revert kernel...${NC}"
runAnsiblePlaybook 08-revert-kernel.yaml
echo -e "${COLOUR_SUCCESS}Kernel reverted.${NC}"

echo -e "${COLOUR_PROGRESS}Setup wireless AP...${NC}"
runAnsiblePlaybook 09-wireless_ap.yaml
echo -e "${COLOUR_SUCCESS}Wireless AP config set.${NC}"

echo -e "${COLOUR_PROGRESS}Setup config UI...${NC}"
runAnsiblePlaybook 10-config_ui.yaml
echo -e "${COLOUR_SUCCESS}Config UI set.${NC}"

echo -e "${COLOUR_PROGRESS}Install OARK...${NC}"
runAnsiblePlaybook 11-oark_install.yaml
echo -e "${COLOUR_SUCCESS}OARK installed.${NC}"
