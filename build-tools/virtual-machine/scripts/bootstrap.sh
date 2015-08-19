#!/bin/bash

set -euo pipefail
IFS=$'\n\t'

# Assume we live in Western Australia
sed -i 's/http:\/\/us\.archive\.ubuntu\.com/http:\/\/ftp.iinet.net.au\/linux/g' /etc/apt/sources.list

# Update package lists
apt-get update

# Add in Ansibles PPA, we want latest releases for bug fixes, etc.
apt-get install -y software-properties-common
apt-add-repository ppa:ansible/ansible

# Bootstrap the development environment
apt-get update

# Build essential is needed because we might want local machine unit testing.
# Unit testing with Arduino hardware is difficult.
apt-get install -y bc build-essential git mercurial unzip

# For installation scripting on the Pi virtual machine.
apt-get install -y ansible sshpass

# For the Ansible hosts file added later.
echo "export ANSIBLE_INVENTORY=~/.ansible_hosts" >> .bashrc
