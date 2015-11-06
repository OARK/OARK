# Build Tools

WARNING: As of 2015-11-06, only the Vagrant method is supported, a real hardware Pi can still be used with the Ansible method, but some source file paths have changed. Check the history of commits of the `virtual-machine/files/playbooks` for details.

As the name says, tools for building the project. There are two different ways that the system can be built, both rely on having a real Pi available on a known IP address. One method is to use Vagrant which will download and script a virtual machine to build the software on the Pi, the other is if you run the build scripts locally on a Linux machine.

# Requirements

The scripting expects an untouched Raspbian install. You can download the Raspbian image from the Raspbian site, and copy the image to a SD card, please check the Raspbian site for details. If you're building using Vagrant, this isn't needed.

# Vagrant

The easiest and most automatic way is to install [Vagrant](https://www.vagrantup.com/) (and either VirtualBox or VMWare), then update two configuration files with the IP address of the Pi to use and `cd` into the `virtual-machine` directory, and run `vagrant up`. Check the README.md in the `virtual-machine` directory for details.

# Remote install

`Ansible` is used for scripting the changes needed to produce a working system. At least version 1.7.2 is expected, check the package system of your Linux distribution for details on how to install this. For Ansible, an ansible_hosts file is expected, this is the file that associates groups with login details. We use the `pi_emu` group, so the file will have something like the following:

    [pi_emu]
    <ip-address> ansible_connection=ssh ansible_ssh_user=pi

`ip-address` must be changed to the IP address of the Raspberry Pi.

Ansible relies on SSH and also on a pre-generated SSH key to be installed on the Pi so there's no need to enter a password. If you've generated a SSH key already (check ~/.ssh/id_rsa.pub) then you're fine to go onto the next step, otherwise you can generate one with the command:

    ssh-keygen -t rsa -b 4096 -C "your_email@example.com"

Just keep the defaults.

## Installing the SSH key onto the Pi
Once you've generated the SSH key, then you will have to install it onto the Pi. Be sure you have the IP address of the Pi (logging in and typing `ifconfig eth0` will give the IP address) in the ansible_hosts file. Change your current directory to `build-tools/virtual-machine/files/playbooks`, and issue the command:

    ansible-playbook -i <ansible-hosts_file> 01-init.yaml -k

If will prompt for the SSH password for the Pi, which is `raspberry`, and then install the SSH key so password prompts are no longer needed. Then, building the system is just a case of executing each of the playbooks in the numbered sequence indicated by their filename.

### SSH complains about key mismatch
You may have to remove the ~/.ssh/known_hosts file if you are using the same IP address that had been used with a different Pi.

## Installing OARK

Once Ansible is set up, then execute the different playbooks in order, so `01-init.yaml`, `02-slimline_install.yaml`, `03-ros_indigo_install.yaml` and so forth. The Pi will need to reboot after `02-slimline_install.yaml` so be sure to wait for it to come back up before proceeding.
