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

# Will return 0 if we can log onto SSH.
testSSHServerRunning() {
    echo -e "${COLOUR_PROGRESS}Checking if emulator image is running...${NC}"
    ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no \
        pi@localhost -p 10022 exit
}

# Be sure to clear out known_hosts
rm -f ~/.ssh/known_hosts

test_emulator_running=0
testSSHServerRunning || test_emulator_running=$?

# It takes awhile for the emulator to start and for it to be
# responsive, so keep doing a simple looping test on SSH until it
# responds.
until [ $test_emulator_running -eq 0 ]
do
    echo -e "${COLOUR_PROGRESS}Not running, sleeping.${NC}"
    sleep 10
    testSSHServerRunning || test_emulator_running=$?
done

echo -e "${COLOUR_SUCCESS}Emulator booted.${NC}"

# Emulation image has booted, so now use Ansible to start building our
# software.

# Run a playbook when given just the name. Assumes directory structure
# in homedir.
runAnsiblePlaybook() {
    ansible-playbook -i ${ANSIBLE_INVENTORY} ~/ansible_playbooks/$1
}

echo -e "${COLOUR_PROGRESS}Slimming Raspbian image...${NC}"
runAnsiblePlaybook slimline_install.yaml
echo -e "${COLOUR_SUCCESS}Raspbian image slimmed.${NC}"

echo -e "${COLOUR_PROGRESS}Starting ROS install...${NC}"
runAnsiblePlaybook ros_indigo_install.yaml
echo -e "${COLOUR_SUCCESS}ROS install success.${NC}"
