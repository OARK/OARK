#!/usr/bin/env python
# The code in this file needs to be made generic. Config files should be
# made to specify the motors. For now, the values are hardcoded.

"""A module to act as a ROS node and create relevant services/topics
that other ROS nodes can use to interact with the emumini2 motors.
"""

import rospy
import argparse

from manager import DXLManager

__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


class EM2Node(object):
    """A node to receive messages from other nodes and
    control the emu mini 2 based on these messages.
    """

    def __init__(self, man_ns, port_ns):
        self.dxl_mgr = DXLManager(manager_ns, port_ns)
        
        self.create_controller(1, 'wheel_fl', DXLManager.TORQUE_CONTROLLER,
                               min=0, max=4093, init=0, joint_name='wheel_fl')
        self.create_controller(2, 'wheel_fr', DXLManager.TORQUE_CONTROLLER,
                               min=0, max=4093, init=0, joint_name='wheel_fr')
        self.create_controller(3, 'wheel_br', DXLManager.TORQUE_CONTROLLER,
                               min=0, max=4093, init=0, joint_name='wheel_br')
        self.create_controller(4, 'wheel_bl', DXLManager.TORQUE_CONTROLLER,
                               min=0, max=4093, init=0, joint_name='wheel_bl')

        self.create_controller(5, 'arm_elbow', DXLManager.POS_CONTROLLER,
                               min=200, max=850, init=200, joint_name='arm_elbow')
        self.create_controller(6, 'arm_wrist', DXLManager.POS_CONTROLLER,
                               min=173, max=810, init=173, joint_name='arm_wrist')
        self.create_controller(7, 'arm_hand', DXLManager.POS_CONTROLLER,
                               min=150, max=870, init=150, joint_name='arm_hand')

        rospy.Subscriber('command', , self.msg_rcvd)


    def msg_rcvd(self, msg):
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




if __name__ == '__main__':
    #Parser command line
    parser = argparse.ArgumentParser(description='Start the emumini2 control node')
    parser.add_argument('manager_namespace',
                        help='The namespace provided to the dynamixel driver software') 
    parser.add_argument('port_namespace',
                        help='The port name provided to the dynamixel driver software')

    args = parser.parse_args()

