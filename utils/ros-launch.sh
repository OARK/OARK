#!/bin/bash
#
# This script is necessary to aid in running ROS on startup.
# The ROS setup files need to be sourced before roslaunch is
# run. This script is a simple roslaunch wrapper that sources
# the required files.
#
# This script should be called using the same parameters as
# one would called roslaunch.

PATH=/usr/local/bin:/usr/bin:/usr/bin:/bin
ROS_PATH=/opt/ros/indigo/setup.bash
PROJ_PATH=/opt/oark/ros-pkg/devel/setup.bash

#Flip video
v4l2-ctl --set-ctrl=horizontal_flip=1
v4l2-ctl --set-ctrl=vertical_flip=1

. $ROS_PATH
. $PROJ_PATH

roslaunch oark oark.launch
