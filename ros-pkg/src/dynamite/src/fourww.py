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

import net.cmd_listener as net_listen
from net.msg import *
from control.pos_controller import PosController
import control.manager_proxy as proxy


controllers = {}

#This is not yet fully implemented. That is to say
#that it  doesn't actually do anything close to
#what it says that it does. Most importantly, the 
#duration field of the msg object is completely
#ignored and so stop messages should be sent to
#stop the movement of the robot.
def msg_received(msg):
    if msg.get_type() == msg.ALL_GO:
        controller['front_left'].command(msg.get_value())
        controller['front_right'].command(msg.get_value())
        controller['back_left'].command(msg.get_value())
        controller['back_front'].command(msg.get_value())
    elif msg.get_type() == msg.TURN_CW:
        controller['front_left'].command(msg.get_value())
        controller['front_right'].command(-msg.get_value())
        controller['back_left'].command(msg.get_value())
        controller['back_front'].command(-msg.get_value())
    elif msg.get_type() == msg.TURN_CCW:
        controller['front_left'].command(-msg.get_value())
        controller['front_right'].command(msg.get_value())
        controller['back_left'].command(-msg.get_value())
        controller['back_front'].command(msg.get_value())
    elif msg.get_type() == msg.STOP:
        controller['front_left'].command(0)
        controller['front_right'].command(0)
        controller['back_left'].command(0)
        controller['back_front'].command(0)
    else:
        print str(msg)


def msg_received_test(msg):
    if msg.get_type() == LEFT_GO:
        controllers['wheel_left'].command(msg.get_value())
    if msg.get_type() == RIGHT_GO:
        controllers['wheel_right'].command(msg.get_value())
    else:
        controllers['wheel_left'].command(0.0);
        controllers['wheel_right'].command(0.0);
        print str(msg)


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(description='Four Wheel Wonder control program')

    port_ns = 'pi_out_port'
    manager_ns = 'dxl_manager'

    #Address of all interfaces
    interfaces = ''

    try:
        #parser.add_argument(

        rospy.init_node('forww_node')

        man_proxy = proxy.ManagerProxy(manager_ns)

        listener = net_listen.CmdListener(interfaces)
        listener.add_data_listener(msg_received_test)

        controllers['wheel_left'] = PosController(man_proxy, 'wheel_left', port_ns)
        controllers['wheel_right'] = PosController(man_proxy, 'wheel_right', port_ns)
        #Right now there is no functional difference
        #between position controllers and torque
        #controllers so only a position controller is
        #used.
        #controllers['front_left'] = PosController(proxy, 'front_left', port_ns)
        #controllers['front_right'] = PosController(proxy, 'front_right', port_ns)
        #controllers['back_left'] = PosController(proxy, 'back_left', port_ns)
        #controllers['back_right'] = PosController(proxy, 'back_right', port_ns)
        #controllers['arm_base'] = PosController(proxy, 'arm_base', port_ns)
        #controllers['arm_top'] = PosController(proxy, 'arm_top', port_ns)

        rospy.spin()

        listener.shutdown()

    except rospy.ROSInterruptException, r:
        pass

