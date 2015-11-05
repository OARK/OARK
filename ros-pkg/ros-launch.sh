#!/bin/bash
#
# This script is necessary to aid in running ROS on startup.
# The ROS setup files need to be sourced before roslaunch is
# run. This script is a simple roslaunch wrapper that sources
# the required files.
#
# This script runs hand in hand with the oark init.d script.

PATH=/usr/local/bin:/usr/bin:/usr/bin:/bin
ROS_PATH=/opt/ros/indigo
PROJ_PATH=/opt/oark/ros-pkg

# The OARK robot config file. This config file specifies all of the motor
# configurations and the Android inputs.
OARK_ROBOT_CONFIG="$PROJ_PATH/src/oark/emumini2.yaml"


. "$ROS_PATH/setup.bash"
. "$PROJ_PATH/devel/setup.bash"

#Flip video
v4l2-ctl --set-ctrl=horizontal_flip=1
v4l2-ctl --set-ctrl=vertical_flip=1

#Initialise motors
./init_motors.py "$OARK_ROBOT_CONFIG"\
    -m "$ROS_PATH/lib/dynamixel_driver/set_servo_config.py"\
    -p "/dev/ttyAMA0"\
    -b 115200

if [ $? -eq 0 ]; then
    roslaunch oark oark.launch
else
    echo "Could not initialise motors"
fi
