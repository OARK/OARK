#!/bin/bash
#
# A script to initialise all of the motors to the correct
# parameters.

DYN_PATH='/opt/ros/indigo/lib/dynamixel_driver'
BAUD=115200
PORT='/dev/ttyAMA0'

"${DYN_PATH}/set_servo_config.py" --port=$PORT --baud=$BAUD \
	--cw-angle-limit=0 --ccw-angle-limit=0 1 2 3 4

"${DYN_PATH}/set_servo_config.py" --port=$PORT --baud=$BAUD \
	--cw-angle-limit=0 --ccw-angle-limit=1024 5 6 7
