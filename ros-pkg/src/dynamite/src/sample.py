#!/usr/bin/env python
#


"""A sample script for creating controllers for the emumini2 and driving
it around for a few seconds.
Written by Tim Peskett
"""

#Import ROS's python bindings
import rospy
import time

#Import our controller creation code
from control.pos_controller import PosController
from control.torque_controller import TorqueController
import control.manager_proxy as proxy





def init_wheels(man, port_ns):
    """Initialise all of the motors by creating a controllers. The
    names of each motor are currently set in a config file. The names
    shouldn't be duplicated here, but they are for the time being. This will
    be changed in the future.

    Returns a dictionary of controller objects.
    """
    wheels = dict()
    wheels['fl'] = TorqueController(man, 'wheel_fl', port_ns)
    wheels['fr'] = TorqueController(man, 'wheel_fr', port_ns)
    wheels['bl'] = TorqueController(man, 'wheel_bl', port_ns)
    wheels['br'] = TorqueController(man, 'wheel_br', port_ns)

    return wheels



def init_arm(man, port_ns):
    """Initialise all of the arm motors and create controllers for them.
    Again, the names of each motor are set elsewhere, and they must be used
    here verbatim for now.

    Returns a dictionary containing controllers for each part of the arm
    """
    arm = dict()
    arm['arm_base'] = PosController(man, 'arm_base', port_ns)
    arm['arm_wrist'] = PosController(man, 'arm_wrist', port_ns)
    arm['arm_hand'] = PosController(man, 'arm_hand', port_ns)

    return arm



if __name__ == '__main__':
    """The entry point for our script. We intialise all of the motors and then
    run them through a simple motion of moving forward for two seconds and then
    moving the arm.
    """

    #Serial port and dynamixel driver software constants
    port_ns = 'pi_out_port'
    manager_ns = 'dxl_manager'

    try:
        #Arbitrary node name for ROS to use
        rospy.init_node('em2_sample')

        #Wait until the dynamixel driver software is running (if it isn't already)
        rospy.wait_for_service('/%s/%s/start_controller'%(manager_ns, port_ns))

        #Initialise all of the motors
        man = proxy.ManagerProxy(manager_ns)
        wheels = init_wheels(man, port_ns)
        arm = init_arm(man, port_ns)
        
        #Start wheels moving forwards
        #One side must be moving clockwise and the other ccw
        #Range on torques is -7 to 7
        speed = 7.0
        wheels['fl'].set_torque(-speed)
        wheels['bl'].set_torque(-speed)
        wheels['fr'].set_torque(speed)
        wheels['br'].set_torque(speed)
        
        #Sleep while the robot moves forwards
        time.sleep(2)

        #Stop all wheels
        for wheel in wheels.values():
            wheel.stop()

        #Move all arm motors to resting point (lying flat)
        for arm_motor in arm.values():
            arm_motor.set_position(0)

        #Do nothing until we are sent the ROS shutdown message
        rospy.spin()

    except rospy.ROSInterruptException, r:
        print str(r)
        print 'ROSInterruptException occurred. Exiting...'
