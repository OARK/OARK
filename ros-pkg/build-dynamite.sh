#!/bin/bash

# Build the dynamite ROS system. Assumes ROS has been installed, as
# per the Raspbain build scripts.

source /opt/ros/indigo/setup.bash

catkin_make
