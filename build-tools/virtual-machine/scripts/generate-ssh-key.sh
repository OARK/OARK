#!/bin/bash

# Generate a SSH keypair

set -euo pipefail
IFS=$'\n\t'

RASPBERRY_IP=localhost

# Colour codes
COLOUR_SUCCESS='\033[0;32m'
COLOUR_PROGRESS='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${COLOUR_PROGRESS}Removing any old ssh keys...${NC}"
rm -f ~/.ssh/id_rsa*

echo -e "${COLOUR_PROGRESS}Generating new ssh keypair...${NC}"
ssh-keygen -q -t rsa -b 2048 -N "" -f ~/.ssh/id_rsa

echo -e "${COLOUR_SUCCESS}SSH keypair generated.${NC}"

# The ssh keys can change often, do this so we don't get warnings and
# SSH is unable to work.
rm -f ~/.ssh/config
echo "Host ${RASPBERRY_IP}" > ~/.ssh/config

cat <<EOF >> ~/.ssh/config
  StrictHostKeyChecking no
  UserKnownHostsFile /dev/null
  User pi
  LogLevel QUIET
EOF

# Be sure to clear out known_hosts
rm -f ~/.ssh/known_hosts
