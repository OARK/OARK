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
import control.pos_controller as controller
import control.manager_proxy as proxy


FORWARD = 1
BACKWARD = 2
TURNLEFT = 3
TURNRIGHT = 4
STOP = 5

controllers = {}

#This is not yet fully implemented. That is to say
#that it  doesn't actually do anything close to
#what it says that it does. Most importantly, the 
#duration field of the msg object is completely
#ignored and so stop messages should be sent to
#stop the movement of the robot.
def msg_received(message):
    if msg.get_type() == FORWARD:
        controller['front_left'].command(msg.get_value())
        controller['front_right'].command(msg.get_value())
        controller['back_left'].command(msg.get_value())
        controller['back_front'].command(msg.get_value())
    elif msg.get_type() == BACKWARD:
        controller['front_left'].command(msg.get_value())
        controller['front_right'].command(msg.get_value())
        controller['back_left'].command(msg.get_value())
        controller['back_front'].command(msg.get_value())
    elif msg.get_type() == TURNLEFT:
        controller['front_left'].command(msg.get_value())
        controller['front_right'].command(-msg.get_value())
        controller['back_left'].command(msg.get_value())
        controller['back_front'].command(-msg.get_value())
    elif msg.get_type() == TURNRIGHT:
        controller['front_left'].command(-msg.get_value())
        controller['front_right'].command(msg.get_value())
        controller['back_left'].command(-msg.get_value())
        controller['back_front'].command(msg.get_value())
    elif msg.get_type() == STOP:
        controller['front_left'].command(0)
        controller['front_right'].command(0)
        controller['back_left'].command(0)
        controller['back_front'].command(0)

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(description='Four Wheel Wonder control program')

    port_ns = 'pi_out_port'
    manager_ns = 'dxl_manager'
    addr = '192.0.0.2'

    proxy = ManagerProxy(manager_ns, [port_ns])

    try:
        #parser.add_argument(

        rospy.init_node('forww_node')

        listener = CmdListener(addr)
        listener.add_data_listener(msg_received)

        #Right now there is no functional difference
        #between position controllers and torque
        #controllers so only a position controller is
        #used.
        controllers['front_left'] = PosController(proxy, 'front_left', port_ns)
        controllers['front_right'] = PosController(proxy, 'front_right', port_ns)
        controllers['back_left'] = PosController(proxy, 'back_left', port_ns)
        controllers['back_right'] = PosController(proxy, 'back_right', port_ns)
        controllers['arm_base'] = PosController(proxy, 'arm_base', port_ns)
        controllers['arm_top'] = PosController(proxy, 'arm_top', port_ns)

        rospy.spin()

        listener.shutdown()

    except rospy.ROSInterruptException, rie;
        pass

