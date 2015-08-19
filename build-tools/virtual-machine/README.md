# Virtual Machine

# NOTE

This is currently being reworked. Unfortunately the idea of having a fully software based build system (i.e. without using a real Raspberry Pi) is causing too much problems. This is due to QEMU not providing an environment that seems stable enough to emulate a Pi, or system close enough to the Pi for automation.

As a result, this is currently a work in progress to have a virtual machine that uses Ansible to set up a real Raspberry Pi with a freshly burnt Raspbian image.

# Overview

In order to get a consistent development environment, this directory includes a Vagrant file that can be used to easily create a virtual machine to build the software for the project.

## Requirements

Vagrant is required, then either VirtualBox, or VMWare need to be installed. Vagrant requires a paid licence to use VMWare, so that will be required as well. A 64bit host system is also assumed.

## Usage

The image on the Pi should be a fresh Raspbian install with no modifications at all. For Ansible to work, the IP address of the Pi to be installed to should be entered into the `ansible_hosts` file, in the `files` directory.

Then, just make sure that the `virtual-machine` directory is the current directory, and execute `vagrant up`.
