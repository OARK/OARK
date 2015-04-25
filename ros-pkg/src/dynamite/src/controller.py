#! /usr/bin/env python
#
# Contains a class to represent a joint
# controller.
#
# This class more or less just defines the common interface
# for the two subclasses: PosController and TorqueController.
#
# It is assumed that all relevant parameters have been pushed
# to the ROS parameter server. This can be changed quite easily
# in software at a later time by giving the option of specifying
# controllers completely.
#

from dynamixel_msgs.msg import MotorState

class Controller:
    def __init__(self, proxy, controller_name):
        raise NotImplementedError

    def get_motor_state(self):
        raise NotImplementedError

    def command(self, value):
        raise NotImplementedError

