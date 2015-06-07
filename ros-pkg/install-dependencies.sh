#!/bin/bash

# Installs the dependencies for Dynamite.
rosinstall_generator actionlib --rosdistro indigo --deps --wet-only --exclude roslisp | wstool merge -t src -

wstool update -t src -j4
