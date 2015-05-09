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
from control.torque_controller import TorqueController
import control.manager_proxy as proxy


controllers = {}

def msg_received(msg):
    if msg.get_type() == msg.LEFT_GO:
        controller['fl'].set_torque(msg.get_value())
        controller['bl'].set_torque(msg.get_value())
    elif msg.get_type() == msg.RIGHT_GO:
        #Value has to be negated because servos oriented differently
        controller['fr'].set_torque(-msg.get_value())
        controller['br'].set_torque(-msg.get_value())
    elif msg.get_type() == msg.ARM_GO:
        #Value is between 0 and 255. Arm goes from 200-850 (AX12 Units)
        lower = 200
        upper = 850

        ax12_coords = (msg.get_value / 255) * (upper - lower) + upper
        result = (ax12_coords / 1023 - 0.5) * (300 * 3.1415926 / 180)
        controller['arm_base'].set_position(result)

    elif msg.get_type() == msg.WRIST_GO:
        #Value is between 0 and 255. Wrist goes from 173 - 820 (AX12 Units)
        lower = 173
        upper = 820

        ax12_coords = (msg.get_value / 255) * (upper - lower) + upper
        result = (ax12_coords / 1023 - 0.5) * (300 * 3.1415926 / 180)
        controller['arm_wrist'].set_position(result)

    elif msg.get_type() == msg.HAND_GO:
        #Value is between 0 and 255. Hand goes from 570-1023 (AX12 Units)
        lower = 570
        upper = 1023

        ax12_coords = (msg.get_value / 255) * (upper - lower) + upper
        result = (ax12_coords / 1023 - 0.5) * (300 * 3.1415926 / 180)
        controller['arm_hand'].set_position(result)
    else:
        print str(msg)



if __name__ == '__main__':
    #args = rospy.myargv(argv=sys.argv)
    #parser = argparse.ArgumentParser(description='Four Wheel Wonder control program')

    port_ns = 'pi_out_port'
    manager_ns = 'dxl_manager'

    #Address of all interfaces
    interfaces = ''

    try:

        rospy.init_node('forww_node')

        man_proxy = proxy.ManagerProxy(manager_ns)

        listener = net_listen.CmdListener(interfaces)
        listener.add_data_listener(msg_received_test)

        controllers['fl'] = torque_controller.TorqueController(man, 'wheel_fl', 'pi_out_port')
        controllers['fr'] = torque_controller.TorqueController(man, 'wheel_fr', 'pi_out_port')
        controllers['bl'] = torque_controller.TorqueController(man, 'wheel_bl', 'pi_out_port')
        controllers['br'] = torque_controller.TorqueController(man, 'wheel_br', 'pi_out_port')

        controllers['arm_base'] = pos_controller.PosController(man, 'arm_base', 'pi_out_port')
        controllers['arm_wrist'] = pos_controller.PosController(man, 'arm_wrist', 'pi_out_port')
        controllers['arm_hand'] = pos_controller.PosController(man, 'arm_hand', 'pi_out_port')

        rospy.spin()

        listener.shutdown()

    except rospy.ROSInterruptException, r:
        pass

