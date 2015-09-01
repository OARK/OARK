# Build Tools

As the name says, tools for building the project. There are three different ways that the system can be built, using Vagrant, locally on a Linux machine, or remotely onto an actual Raspberry Pi.

# Vagrant

The easiest and most automatic way is to install [Vagrant](https://www.vagrantup.com/) (and either VirtualBox or VMWare), `cd` into the `virtual-machine` directory, and run `vagrant up`. Check the README.md in the `virtual-machine` directory for details.

# Remote install

`Ansible` is used for scripting the changes needed to produce a working system. At least version 1.7.2 is expected, check the package system of your Linux distribution for details on how to install this. For Ansible, an ansible_hosts file is expected, this is the file that associates groups with login details. We use the `pi_emu` group, so the file will have something like the following:

    [pi_emu]
    <ip-address> ansible_connection=ssh ansible_ssh_user=pi

`ip-address` must be changed to the IP address of the Raspberry Pi.

Ansible relies SSH and also relies on a pre-generated SSH key to be installed on the Pi so there's no need to enter a password. If you've generated a SSH key already (check ~/.ssh/id_rsa.pub) then you're fine to go onto the next step, otherwise you can generate one with the command:

    ssh-keygen -t rsa -b 4096 -C "your_email@example.com"

Just keep the defaults.

## Installing the SSH key onto the Pi
Once you've generated the SSH key, then you will have to install it onto the Pi. Be sure you have the IP address of the Pi (logging in and typing `ifconfig eth0` will give the IP address) in the ansible_hosts file. Change your current directory to `build-tools/virtual-machine/files/playbooks`, and issue the command:

    ansible-playbook -i <ansible-hosts_file> 01-init.yaml -k

If will prompt for the SSH password for the Pi, which is `raspberry`, and then install the SSH key so password prompts are no longer needed. Then, building the system is just a case of executing each of the playbooks in the numbered sequence indicated by their filename.

# Note
This directory used to contain scripts for building a QEMU compatible kernel and installing Arch Linux onto an image file for generating the OARK without need of a real Pi. However, as of 2015-08-17, this has proven to be a bit difficult, with errors occurring in the emulated system, in particular package downloads.

These utilities and their directories have been deleted to reduce confusion. However, if you check the history of the repository, they can be viewed if a reference is necessary for future work. It should be around changeset 225, (hash: d46c937e4da2).
