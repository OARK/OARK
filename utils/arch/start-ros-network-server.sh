#!/bin/bash

source /opt/ros/indigo/setup.bash
export PYTHONPATH=/opt/ros/indigo/lib/python2.7/site-packages:$PYTHONPATH
export PKG_CONFIG_PATH="/opt/ros/indigo/lib/pkgconfig:$PKG_CONFIG_PATH"
export PATH="/opt/ros/indigo/bin:$PATH"

# Useful aliases
alias catkin_make="catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/libpython2.7.so"

source /home/aldredmr/Projects/oark/ros-pkg/devel/setup.bash

source /home/aldredmr/Projects/oark/ros-pkg/src/dynamite/scripts/env.sh
python2 /home/aldredmr/Projects/oark/ros-pkg/src/dynamite/src/em2.py
