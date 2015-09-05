#!/usr/bin/env python
# The code in this file needs to be made generic. Config files should be
# made to specify the motors. For now, the values are hardcoded.

"""A module to act as a ROS node and create relevant services/topics
that other ROS nodes can use to interact with the emumini2 motors.
"""

import rospy
import sys
import argparse
import threading

from manager import DXLManager
from emumini2.msg import Command

__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


#Temporary. These constants should definitely be moved elsewhere
LEFT_GO = 3
RIGHT_GO = 4
ARM_GO = 6
WRIST_GO = 9
HAND_GO = 10


class EM2Node(object):
    """A node to receive messages from other nodes and
    control the emu mini 2 based on these messages.
    """

    def __init__(self, man_ns, port_ns):
        rospy.loginfo('Creating controllers...')
        d = DXLManager(man_ns, port_ns)
        d.create_controller(1, 'fl', DXLManager.TORQUE_CONTROLLER,
                            min=0, max=4093, init=0, joint_name='wheel_fl')
        d.create_controller(2, 'fr', DXLManager.TORQUE_CONTROLLER,
                            min=0, max=4093, init=0, joint_name='wheel_fr')
        d.create_controller(3, 'br', DXLManager.TORQUE_CONTROLLER,
                            min=0, max=4093, init=0, joint_name='wheel_br')
        d.create_controller(4, 'bl', DXLManager.TORQUE_CONTROLLER,
                            min=0, max=4093, init=0, joint_name='wheel_bl')

        d.create_controller(5, 'elbow', DXLManager.POS_CONTROLLER,
                            min=200, max=850, init=200, joint_name='arm_elbow')
        d.create_controller(6, 'wrist', DXLManager.POS_CONTROLLER,
                            min=173, max=810, init=173, joint_name='arm_wrist')
        d.create_controller(7, 'hand', DXLManager.POS_CONTROLLER,
                            min=150, max=870, init=150, joint_name='arm_hand')
        self.dxl_mgr = d
        rospy.loginfo('Controllers created!')

        #Create a mutex to allow multithreaded subscriber
        self._cmd_mutex = threading.Lock()
        rospy.Subscriber('command', Command, self.msg_rcvd)


    def msg_rcvd(self, msg):
        with self._cmd_mutex:
            scale_func = lambda val: val * 0.5
            if msg.type == LEFT_GO:
                speed = scale_func(msg.value)
                self.dxl_mgr['fl'].set_torque_buffered(speed)
                self.dxl_mgr['bl'].set_torque_buffered(speed)

            elif msg.type == RIGHT_GO:
                #Value has to be negated because servos oriented differently
                speed = -scale_func(msg.value)
                self.dxl_mgr['fr'].set_torque_buffered(speed)
                self.dxl_mgr['br'].set_torque_buffered(speed)

            elif msg.type == ARM_GO:
                #Value is between 0 and 127. Arm goes from 200-850 (AX12 Units)
                lower = 200
                upper = 850

                result = rescale(msg.value, 0, 127, 
                                 0, (upper - lower) / 1023.0 * radians(300))

                self.dxl_mgr['base'].set_position_buffered(result)

            elif msg.type == WRIST_GO:
                #Value is between 0 and 127. Wrist goes from 173 - 820 (AX12 Units)
                lower = 173
                upper = 820

                result = rescale(msg.value, 0, 127, 
                                 0, (upper - lower) / 1023.0 * radians(300))

                self.dxl_mgr['wrist'].set_position_buffered(result)

            elif msg.type == HAND_GO:
                #Value is between 0 and 127. Hand goes from 150-870 (AX12 Units)
                lower = 150 
                upper = 870 

                result = rescale(msg.value, 0, 127, 
                                 0, (upper - lower) / 1023.0 * radians(300))

                self.dxl_mgr['hand'].set_position_buffered(result)
            else:
                rospy.logwarn('Malformed message')
                rospy.logwarn('Message: ' + str(msg))


    def run(self):
        rospy.loginfo('EM2Node running')
        choose_func = lambda q: ([q[-1]] if q else [])
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for name, cont in self.dxl_mgr:
                cont.flush(choose=choose_func)

            rate.sleep()



if __name__ == '__main__':
    #Parser command line
    parser = argparse.ArgumentParser(description='Start the emumini2 control node')
    parser.add_argument('manager_namespace',
                        help='The manager namespace provided to the dynamixel driver software') 
    parser.add_argument('port_namespace',
                        help='The port name provided to the dynamixel driver software')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    try:
        rospy.init_node('em2_node')
        em2_node = EM2Node(args.manager_namespace, args.port_namespace)
        em2_node.run()
    except rospy.ROSInterruptException, rie:
        rospy.logerr('Fatal error occurred. Exiting...')
