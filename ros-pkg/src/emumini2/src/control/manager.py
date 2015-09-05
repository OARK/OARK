#!/usr/bin/env python
 
"""A module to provide functions/classes for managing motor objects
"""

import threading
import rospy

from dynamixel_msgs.msg import MotorStateList, MotorState
from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController


from controller import PosController, TorqueController


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

    def __init__(self, man_ns, port_ns):
        """Initialises our newly constructed DXLManager object. port_ns
        should be unique for each constructed DXLManager to ensure correct
        operation of motors.
        Params: 
               man_ns - The namespace of the dynamixel driver software. This
                        is set outside of the software but is required here to properly
                        start the controllers.
               port_ns - The namespace of the port provided to the
                        dynamixel_motors driver software. This will
                        be provided in the roslaunch config files
        """
        self.man_ns = man_ns
        self.port_ns = port_ns
        start_srv_name   = '/%s/%s/start_controller'%(self.man_ns, self.port_ns,)
        stop_srv_name    = '/%s/%s/stop_controller'%(self.man_ns, self.port_ns,)
        restart_srv_name = '/%s/%s/restart_controller'%(self.man_ns, self.port_ns,)

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
            rospy.Subscriber('/motor_states/%s'%(self.port_ns,),
                             MotorStateList,
                             self._motor_state_callback)

        self.cont_list = dict()


    def __getitem__(self, cont_name):
        """An accessor to allow easy access to this class's core functionality."""
        return self.cont_list[cont_name]


    def __iter__(self):
        """A special python method that allows iterating over this object. Actually just
        iterates over the (name, controller) tuples in the object."""
        return self.cont_list.iteritems()


    def _motor_state_callback(self, msl):
        """A callback that for the dynamixel driver software to supply the motor
        state list to. Should not be called from other code.
        """
        with self._motor_state_mutex:
            self.motor_state_list = msl


    def get_motor_state(self, motor_id, default=None):
        """Get the state of a motor. There need not be a motor controller
        instantiated to view these motor states.
        """
        with self._motor_state_mutex:
            for motor in self.motor_state_list:
                if motor_id == motor.id:
                    return motor
            return default


    def create_controller(self, motor_id, name, cont_type, **kwargs):
        """Create a controller for a motor. This will instantiate an
        object inside of the manager that can be used later on to send commands
        to the motor.
        Params: motor_id - ID of the motor that the controller is to control.
                name - A name that you wish the controller to have. It will be
                identified by this name. Should be a string: e.g 'wheel_fr'.
                cont_type - The type of controller that you wish to create. See
                the constants at the top of this class.
                **kwargs - Extra keyword arguments, these can change depending
                on the type of controller you wish to create. For the most part,
                the following will suffice:
                    min - The minimum position/torque for the motor.
                    max - The maximum position/torque for the motor.
                    init - Where the value is measured from for the motor.
                    joint_name - An arbitrary name for the joint_name.
        """
        if name in self.cont_list:
            raise ValueError('A controller by that name already exists')

        if cont_type == DXLManager.TORQUE_CONTROLLER:
            ContType = TorqueController
        elif cont_type == DXLManager.POS_CONTROLLER:
            ContType = PosController
        else:
            raise ValueError('Invalid type suppled. Must be pos or torque controller')

        self.cont_list[name] = ContType(motor_id, name, 
                                        self.port_ns, kwargs,
                                        self._start_srv, self._stop_srv,
                                        self._restart_srv)


    def destroy_controller(self, name):
        """Destroys a controller. The controller should not be referenced
        after this is called. It will return an error if you try to.
        """
        if name not in self.cont_list:
            raise ValueError('No controller by that name exists')

        self.cont_list[name]._destroy()
        del self.cont_list[name]


import time

#A small test harness
if __name__ == '__main__':
    try:
        rospy.init_node('test_node')
        dxl_man = DXLManager('dxl_manager', 'pi_out_port')
        dxl_man.create_controller(1, 'wheel_one', DXLManager.TORQUE_CONTROLLER, joint_name='wheel_joint', min=0, max=4093, init=0)
        dxl_man['wheel_one'].set_torque(1)
        time.sleep(2)
        dxl_man['wheel_one'].set_torque(0)
        while not rospy.is_shutdown():
            print str(dxl_man['wheel_one'].get_joint_state())
            time.sleep(1)
    except Exception, e:
        print 'Error: ', str(e)
