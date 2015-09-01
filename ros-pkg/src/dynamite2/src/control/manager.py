#!/usr/bin/env python
 
"""A module to provide functions/classes for managing motor objects
"""

import rospy
import threading

from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController

from dynamixel_msgs.msg import MotorStateList, MotorState


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


class DXLManager(object):
    """A manager for a dynamixel bus. It is concerned with all of the motors
    present on a bus. 
    """
    
    #Constants to decide which controller to create
    TORQUE_CONTROLLER = 0
    POS_CONTROLLER    = 1

    def __init__(self, port_ns):
        """Initialises our newly constructed DXLManager object. port_ns
        should be unique for each constructed DXLManager to ensure correct
        operation of motors.
        Params: port_ns - The namespace of the port provided to the
                        dynamixel_motors driver software. This will
                        be provided in the roslaunch config files
        """
        self.port_ns = port_ns
        start_srv_name   = 'start_controller/%s'%(self.port_ns,)
        stop_srv_name    = 'stop_controller/%s'%(self.port_ns,)
        restart_srv_name = 'restart_controller/%s'%(self.port_ns,)

        #Wait for dynamixel_motors driver software to initialise
        rospy.wait_for_service(start_srv_name)
        rospy.wait_for_service(stop_srv_name)
        rospy.wait_for_service(restart_srv_name)

        #Prepare service calls
        self._start_srv   = rospy.ServiceProxy(start_srv_name,
                                                StartController)
        self._stop_srv    = rospy.ServiceProxy(stop_srv_name,
                                                StopController)
        self._restart_srv = rospy.ServiceProxy(restart_srv_name,
                                                RestartController)
 
        #Register for motor state list
        self._motor_state_mutex = threading.Lock()
        with self._motor_state_mutex:
            rospy.Subscriber('motor_states/%s'%(self.port_ns,),
                             MotorStateList,
                             self._motor_state_callback)

        self.cont_list = dict()


    def __getitem__(self, cont_name):
        return self.cont_list[cont_name]


    def _motor_state_callback(self, msl):
        with self._motor_state_mutex:
            self.motor_state_list = msl


    def get_motor_state(self, motor_id, default=None):
        with self._motor_state_mutex:
            for motor in self.motor_state_list:
                if motor_id == motor.id:
                    return motor
            return default


    def create_controller(self, motor_id, name, type, params):
        if name in self.cont_list:
            raise ValueError('A controller by that name already exists')

        if type == TORQUE_CONTROLLER:
            ContType = TorqueController
        elif type == POS_CONTROLLER:
            ContType = PosController
        else:
            raise ValueError('Invalid type suppled. Must be pos or torque controller')

        self.cont_list[name] = ContType(motor_id, name, 
                                        port_ns, params,
                                        this._start_srv, this._stop_srv,
                                        this.restart_srv)


    def destroy_controller(self, name):
        if name not in self.cont_list:
            raise ValueError('No controller by that name exists')

        del self.cont_list[name]
