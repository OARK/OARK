#!/bin/bash

# Install ROS Indigo on Raspbian.
#
# Copyright (c) 2015 SEP 17

set -euo pipefail
IFS=$'\n\t'

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu wheezy main" > \
    /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | \
    sudo apt-key add -

sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> \
    /etc/apt/sources.list'

wget http://mirrordirector.raspbian.org/raspbian.public.key -O - | \
    sudo apt-key add -

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

wget http://archive.raspberrypi.org/debian/raspberrypi.gpg.key -O - | \
    sudo apt-key add -

sudo apt-get update
sudo apt-get -y upgrade

sudo apt-get -y install python-setuptools python-pip python-yaml python-argparse \
    python-distribute python-docutils python-dateutil python-six

sudo apt-get -y install checkinstall cmake

sudo apt-get -y build-dep console-bridge

sudo pip install rosdep rosinstall_generator wstool rosinstall

sudo rosdep init
rosdep update

mkdir -p ~/Projects/ros_catkin_ws
cd ~/Projects/ros_catkin_ws

rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only \
    --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall

wstool init src indigo-ros_comm-wet.rosinstall

mkdir -p ~/Projects/ros_catkin_ws/external_src

cd ~/Projects/ros_catkin_ws/external_src
apt-get -y source -b console-bridge
sudo dpkg -i libconsole-bridge0.2_*.deb libconsole-bridge-dev_*.deb

# This will take a long time, because of the unit tests.
# TODO, apply patch to remove slowest unit test (fuzzy)
apt-get -y source -b lz4

# # Have to remove -b from above line
# cd lz4-0.0~r122
# export QUILT_PATCHES=debian/patches
# # Push all patches
# quilt push -a
# # Add our own patch
# quilt new 0002-Remove-fuzzy-test.patch
# quilt add programs/Makefile
# # Modify the file in your favourite text editor.
# quilt refresh
# quilt pop -a

# Add info into changelog
# dch -n

# build the package
# debuild -uc -us

sudo dpkg -i liblz4-*.deb

cd ~/Projects/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r \
    --os=debian:wheezy

sudo ./src/catkin/bin/catkin_make_isolated --install \
    -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo
