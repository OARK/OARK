#! /usr/bin/env python
#
# This is the 'main' module for control
# of the Open Acadmic Robot Kit four
# wheeled car. This module is very much
# a prototype to allow testing of various
# features. As such, something has gone very
# wrong if this is used in production code.
#
# The module makes use of two important packages:
# the 'net' package and the 'control' package.
# The net package handles network communication
# between this software and any software sending commands
# over a network.
# The control package handles physical control of the
# robot. For the time being it is mostly used as a 
# slightly simpler interface to the servo drivers.
#
# This code runs as a ROS node.
#
# Right now a *lot* of this is hard-coded. Most
# notably the controller names and the message
# types. This will hopefully be changed before release.
#


import rospy
import sys
import argparse
from math import radians

import net.cmd_listener as net_listen
from net.msg import *
from control.pos_controller import PosController
from control.torque_controller import TorqueController
import control.manager_proxy as proxy
from utils import rescale


controllers = {}

def msg_received(msg):
    if msg.get_type() == LEFT_GO:
        controllers['fl'].set_torque(msg.get_value())
        controllers['bl'].set_torque(msg.get_value())

    elif msg.get_type() == RIGHT_GO:
        #Value has to be negated because servos oriented differently
        controllers['fr'].set_torque(-msg.get_value())
        controllers['br'].set_torque(-msg.get_value())

    elif msg.get_type() == ARM_GO:
        #Value is between 0 and 127. Arm goes from 200-850 (AX12 Units)
        lower = 200
        upper = 850

        result = rescale(msg.get_value(), 0, 127, 
                         0, (upper - lower) / 1023.0 * radians(300))

        controllers['arm_base'].set_position(result)

    elif msg.get_type() == WRIST_GO:
        #Value is between 0 and 127. Wrist goes from 173 - 820 (AX12 Units)
        lower = 173
        upper = 820

        result = rescale(msg.get_value(), 0, 127, 
                         0, (upper - lower) / 1023.0 * radians(300))

        controllers['arm_wrist'].set_position(result)

    elif msg.get_type() == HAND_GO:
        #Value is between 0 and 127. Hand goes from 150-870 (AX12 Units)
        lower = 150 
        upper = 870 

        result = rescale(msg.get_value(), 0, 127, 
                         0, (upper - lower) / 1023.0 * radians(300))

        controllers['arm_hand'].set_position(result)
    else:
        print str(msg)

def msg_received_test(msg):
    print str(msg)

def client_dc():
    #Stop all motors
    for cont in controllers:
        controllers[cont].command(0)



if __name__ == '__main__':
    port_ns = 'pi_out_port'
    manager_ns = 'dxl_manager'

    #Address of all interfaces
    interfaces = ''

    try:

        rospy.init_node('em2_node')

        man = proxy.ManagerProxy(manager_ns)

        listener = net_listen.CmdListener(interfaces)
        listener.add_data_listener(msg_received)
        listener.add_dc_listener(client_dc)

        controllers['fl'] = TorqueController(man, 'wheel_fl', port_ns)
        controllers['fr'] = TorqueController(man, 'wheel_fr', port_ns)
        controllers['bl'] = TorqueController(man, 'wheel_bl', port_ns)
        controllers['br'] = TorqueController(man, 'wheel_br', port_ns)

        controllers['arm_base'] = PosController(man, 'arm_base', port_ns)
        controllers['arm_wrist'] = PosController(man, 'arm_wrist', port_ns)
        controllers['arm_hand'] = PosController(man, 'arm_hand', port_ns)

        rospy.spin()

        listener.shutdown()

    except rospy.ROSInterruptException, r:
        pass

