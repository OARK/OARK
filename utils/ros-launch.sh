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


. $ROS_PATH
. $PROJ_PATH

#roslaunch $@
roscore
