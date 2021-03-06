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

#TODO: Remove constants

import rospy
import sys
import threading
import time
import signal
from math import radians

import net.cmd_listener as net_listen
from net.msg import *
from control.pos_controller import PosController
from control.torque_controller import TorqueController
import control.manager_proxy as proxy
from utils import rescale


controllers = {}
cmd_mutex = threading.Lock()

def msg_received(msg):
    with cmd_mutex:
        scale_func = lambda val: val * 0.5
        if msg.get_type() == LEFT_GO:
            speed = scale_func(msg.get_value())
            controllers['fl'].set_torque_buffered(speed)
            controllers['bl'].set_torque_buffered(speed)

        elif msg.get_type() == RIGHT_GO:
            #Value has to be negated because servos oriented differently
            speed = -scale_func(msg.get_value())
            controllers['fr'].set_torque_buffered(speed)
            controllers['br'].set_torque_buffered(speed)

        elif msg.get_type() == ARM_GO:
            #Value is between 0 and 127. Arm goes from 200-850 (AX12 Units)
            lower = 200
            upper = 850

            result = rescale(msg.get_value(), 0, 127, 
                             0, (upper - lower) / 1023.0 * radians(300))

            controllers['arm_base'].set_position_buffered(result)

        elif msg.get_type() == WRIST_GO:
            #Value is between 0 and 127. Wrist goes from 173 - 820 (AX12 Units)
            lower = 173
            upper = 820

            result = rescale(msg.get_value(), 0, 127, 
                             0, (upper - lower) / 1023.0 * radians(300))

            controllers['arm_wrist'].set_position_buffered(result)

        elif msg.get_type() == HAND_GO:
            #Value is between 0 and 127. Hand goes from 150-870 (AX12 Units)
            lower = 150 
            upper = 870 

            result = rescale(msg.get_value(), 0, 127, 
                             0, (upper - lower) / 1023.0 * radians(300))

            controllers['arm_hand'].set_position_buffered(result)
        else:
            print 'Unexpected message'
            print str(msg)

def client_dc():
    #Stop all motors
    print 'Client has disconnected!'
    for cont in controllers:
        controllers[cont].command(0)


#Handles interrupts by exiting. Shuts down the network
#connection
def sigint_handler(signal, frame):
    print 'Exiting'
    sys.exit(0)


if __name__ == '__main__':
    #Register handlers
    signal.signal(signal.SIGINT, sigint_handler)


    port_ns = 'pi_out_port'
    manager_ns = 'dxl_manager'

    #Address of all interfaces
    interfaces = ''

    try:

        rospy.init_node('em2_node')

        man = proxy.ManagerProxy(manager_ns)

        listener = net_listen.CmdListener(interfaces)

	rospy.wait_for_service("%s/%s/start_controller"%(manager_ns,port_ns))

        listener.add_data_listener(msg_received)
        listener.add_dc_listener(client_dc)

        controllers['fl'] = TorqueController(man, 'wheel_fl', port_ns)
        controllers['fr'] = TorqueController(man, 'wheel_fr', port_ns)
        controllers['bl'] = TorqueController(man, 'wheel_bl', port_ns)
        controllers['br'] = TorqueController(man, 'wheel_br', port_ns)

        controllers['arm_base'] = PosController(man, 'arm_base', port_ns)
        controllers['arm_wrist'] = PosController(man, 'arm_wrist', port_ns)
        controllers['arm_hand'] = PosController(man, 'arm_hand', port_ns)

        choose_func = lambda q: ([q[-1]] if q else [])
        while not rospy.is_shutdown():
            time.sleep(0.05)

            for name, cont in controllers.items():
                cont.flush_cmd(choose=choose_func)

            if listener.disconnected:
                try:
                    listener.begin_listen()
                except Exception, e:
                    print 'Could not reconnect: ' + str(e)

        listener.shutdown()

    except rospy.ROSInterruptException, r:
        print str(r)
        print 'ROSInterruptException occurred. Exiting...'

