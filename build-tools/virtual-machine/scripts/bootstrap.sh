#!/bin/bash

# Bootstrap the development environment
apt-get update

# Build essential is needed because we might want local machine unit testing.
# Unit testing with Arduino hardware is difficult.
apt-get install -y bc build-essential git mercurial unzip

# For Pi emulation
apt-get install -y qemu-system-arm

# For installation scripting on the Pi virtual machine.
apt-get install -y ansible sshpass
