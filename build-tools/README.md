# Build Tools

As the name says, tools for building the project. There are three different ways that the system can be built, using Vagrant, locally on a Linux machine, or remotely onto an actual Raspberry Pi.

# Vagrant

The easiest and most automatic way is to install [Vagrant](https://www.vagrantup.com/) (and either VirtualBox or VMWare), `cd` into the `virtual-machine` directory, and run `vagrant up`. Check the README.md in the `virtual-machine` directory for details.

# Remote install

`Ansible` is used for scripting any changes needed for building the image from the Pi itself. The same playbooks are used for the QEMU emulated system, or for a remote install to a real Pi. A stock ARM Arch Linux install is expected, which means root access is allowed from SSH using `root` as the password, it will create a user account with password-less sudo for further access.

# Note
This directory used to contain scripts for building a QEMU compatible kernel and installing Arch Linux onto an image file for generating the OARK without need of a real Pi. However, as of 2015-08-17, this has proven to be a bit difficult, with errors occurring in the emulated system, in particular package downloads.

These utilities and their directories have been deleted to reduce confusion. However, if you check the history of the repository, they can be viewed if a reference is necessary for future work. It should be around changeset 225, (hash: d46c937e4da2).
