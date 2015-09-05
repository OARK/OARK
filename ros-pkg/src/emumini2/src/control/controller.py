#!/usr/bin/env python

"""A module containing controller objects that can be used to control
motors. These classes should most likely not be instantiated directly,
but should be accessed through a DXLManager object.
"""

import rospy
import threading

import std_msgs
from dynamixel_msgs.msg import JointState


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

        #Create the publisher that we can send commands on
        self._publisher = rospy.Publisher('/%s/command'%(self.name,),
                                          std_msgs.msg.Float64,
                                          latch=True,
                                          queue_size=10)

        #A mutex to control access to the joint state
        self._js_mutex = threading.Lock()
        self.joint_state = None
        rospy.Subscriber('/%s/state'%(self.name,),
                         JointState,
                         self._joint_state_callback)



    def _cmd(self, val):
        """Publish a command to the dynamixel"""
        self._publisher.publish(std_msgs.msg.Float64(val))


    #TODO: Add time 
    def _queue_cmd(self, cmd):
        """Queue up a command for publishing to the dynamixel"""
        with self._cmd_mutex:
            self._cmd_queue.append(cmd)


    def _flush_cmd(self, choose=lambda q: q):
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

        #Ensure that controller has started up
        rospy.wait_for_service(cont_ns + '/set_speed')

        super(PosController, self).__init__(motor_id, name)


    def _destroy(self):
        """Destroys the controller. This method should not be called directly
        on the class. The manager will call this method before it is destroyed.
        """
        self._stop(self.name)


    def _pos_to_cmd(self, pos):
        """Converts a value for a position into a value for the AX12. This
        allows simple conversion of units to be implemented later.
        """
        return pos


    def set_pos(self, pos):
        """Set the position of a motor. The value that corresponds to a
        given position will depend on _pos_to_cmd.
        """
        self._cmd(self._pos_to_cmd(pos))


    def set_pos_buffered(self, pos):
        """Queue a command to set the position of the motor."""
        self._queue_cmd(pos)


    def flush(self, choose=lambda q: q):
        """Flush the commands in the command queue. See Controller._flush_cmd
        """
        self._flush_cmd(choose)



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

        super(TorqueController, self).__init__(motor_id, name)


    def _destroy(self):
        """Destroys the controller. This method should not be called directly
        on the class. The manager will call this method before it is destroyed.
        """
        self._stop(self.name)


    def _torque_to_cmd(self, torque):
        """Converts a value for a torque into a value for the AX12. This
        allows simple conversion of units to be implemented later.
        """
        return torque


    def set_torque(self, torque):
        """Set the torque of a motor. The value that corresponds to a
        given torque will depend on _torque_to_cmd.
        """
        self._cmd(self._torque_to_cmd(torque))


    def set_torque_buffered(self, torque):
        """Queue a command to set the torque of the motor."""
        self._queue_cmd(torque)


    def flush(self, choose=lambda q: q):
        """Flush the commands in the command queue. See Controller._flush_cmd
        """
        self._flush_cmd(choose)

