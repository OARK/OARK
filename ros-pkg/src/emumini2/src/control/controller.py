#!/usr/bin/env python

"""A module containing controller objects that can be used to control
motors. These classes should most likely not be instantiated directly,
but should be accessed through a DXLManager object.
"""

import rospy
import threading

import std_msgs
from dynamixel_msgs.msg import JointState

from dynamixel_controllers.srv import SetSpeed


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


class Controller(object):
    """Represents an object that is in charge of controlling a particular
    motor. This class is considered abstract, it should never be
    instantiated directly. Instantiated either TorqueController
    or PosController
    """
    def __init__(self, motor_id, name):
        """Initialise our controller. motor_id should be the hardware
        id of the motor that we wish for this controller to control.
        name should be a string representing the name of the controller.
        """
        self.motor_id = motor_id
        self.name = name
        
        #A mutex to control access to command queue.
        self._cmd_mutex = threading.Lock()
        self._cmd_queue = []

        #A mutex to control access to the joint state
        self._js_mutex = threading.Lock()
        self.joint_state = None
        rospy.Subscriber('/%s/state'%(self.name,),
                         JointState,
                         self._joint_state_callback)


    def _cmd(self, cmd):
        """Publish a command to the dynamixel"""
        raise NotImplementedError('This method is implemented in subclasses')


    def _convert_to_cmd(self, val):
        """Takes a controller-specific value and converts it the appropriate command
        value to be sent to the dynamixel
        """
        raise NotImplementedError('This method is implemented in subclasses')


    #TODO: Add time 
    def queue(self, val):
        """Queue up a command for publishing to the dynamixel"""
        with self._cmd_mutex:
            self._cmd_queue.append(self._convert_to_cmd(val))


    def flush(self, choose=lambda q: q):
        """Flush the command queue. The function to choose which 
        commands are flushed is 'choose'. choose should return a list
        that is a (improper) sublist of q. If you wish all commands to be
        discarded, then use (lambda q: []), for example. By default, all
        commands will be flushed.

        The command queue will be cleared after flushing
        """
        with self._cmd_mutex:
            to_flush = choose(self._cmd_queue)
            for cmd in to_flush:
                #commands were already converted to the correct format when
                #they were add to the queue so we do not have to do that here
                self._cmd(cmd)

            del self._cmd_queue[:]


    def _joint_state_callback(self, js):
        with self._js_mutex:
            self.joint_state = js


    def get_joint_state(self):
        with self._js_mutex:
            return self.joint_state


    def set_pos(self, pos):
        """A stub to make this an 'abstract' class"""
        raise NotImplementedError('Set position not impl. in this class')


    def set_torque(self, torque):
        """A stub to make this an 'abstract' class"""
        raise NotImplementedError('Set torque not impl. in this class')



class PosController(Controller):
    """A class representing a controller that moves a motor to a certain
    position.
    """
    def __init__(self, motor_id, name, port_ns, params, start, stop, restart):
        self._start = start
        self._stop = stop
        self._restart = restart

        cont_ns = '/' + name
        #if rospy.has_param(cont_ns):
            #raise ValueError('A controller by that name is already started')

        rospy.set_param(cont_ns + '/joint_name', params['joint_name'])
        rospy.set_param(cont_ns + '/motor/id', motor_id)
        rospy.set_param(cont_ns + '/motor/init', params['init'])
        rospy.set_param(cont_ns + '/motor/min', params['min'])
        rospy.set_param(cont_ns + '/motor/max', params['max'])

        self._start(port_ns, 'dynamixel_controllers', 'joint_position_controller', 'JointPositionController', name, [])

        #Create the publisher that we can send commands on
        self._publisher = rospy.Publisher('/%s/command'%(name,),
                                          std_msgs.msg.Float64,
                                          latch=True,
                                          queue_size=10)


        super(PosController, self).__init__(motor_id, name)


    def _destroy(self):
        """Destroys the controller. This method should not be called directly
        on the class. The manager will call this method before it is destroyed.
        """
        self._stop(self.name)


    def _cmd(self, cmd):
        """Publish a command to the dynamixel"""
        self._publisher.publish(std_msgs.msg.Float64(cmd))


    def _convert_to_cmd(self, val):
        """Takes a controller-specific value and converts it the appropriate command
        value to be sent to the dynamixel. In this case, converts position value into
        an AX12 position value. This method allows simple conversion of units to be
        implemented later.
        """
        return val


    def set_pos(self, pos):
        """Set the position of a motor. The value that corresponds to a
        given position will depend on _pos_to_cmd.
        """
        self._cmd(self._convert_to_cmd(pos))



class TorqueController(Controller):
    """A class representing a controller that turns at a given speed/torque.
    """
    def __init__(self, motor_id, name, port_ns, params, start, stop, restart):
        self._start = start
        self._stop = stop
        self._restart = restart

        cont_ns = '/' + name
        #if rospy.has_param(cont_ns):
            #raise ValueError('A controller by that name is already started')

        rospy.set_param(cont_ns + '/joint_name', params['joint_name'])
        rospy.set_param(cont_ns + '/motor/id', motor_id)
        rospy.set_param(cont_ns + '/motor/init', params['init'])
        rospy.set_param(cont_ns + '/motor/min', params['min'])
        rospy.set_param(cont_ns + '/motor/max', params['max'])

        self._start(port_ns, 'dynamixel_controllers', 'joint_torque_controller', 'JointTorqueController', name, [])

        #Ensure that controller has started up
        rospy.wait_for_service(cont_ns + '/set_speed')
        self._speed_srv = rospy.ServiceProxy(cont_ns + '/set_speed', SetSpeed)

        super(TorqueController, self).__init__(motor_id, name)


    def _destroy(self):
        """Destroys the controller. This method should not be called directly
        on the class. The manager will call this method before it is destroyed.
        """
        self._stop(self.name)



    def _cmd(self, cmd):
        """Publish a command to the dynamixel"""
        self._speed_srv(cmd)


    def _convert_to_cmd(self, val):
        """Takes a controller-specific value and converts it the appropriate command
        value to be sent to the dynamixel. In this case, converts torque to command.
        """
        return val


    def set_torque(self, torque):
        """Set the torque of a motor. The value that corresponds to a
        given torque will depend on _torque_to_cmd.
        """
        self._cmd(self._convert_to_cmd(torque))
