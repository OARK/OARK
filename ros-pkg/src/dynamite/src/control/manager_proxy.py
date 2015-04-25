#! /usr/bin/env python
#
# Author: SEP Team 17
# 
# The lowest level. Provides an interface to perform
# all remote procedure calls between this node and 
# the dynamixel_controllers controller_manager node.
# This class is very naive and does very little
# input validation, it is just a proxy. It should
# be used with care.
#
# This class is just a prototype and should not be
# used in production code or projects requiring
# reliability.
#
# Any of the functions within this class that require
# ROS params will assume that those params have already
# been set on the ROS parameter server
#
# Right now some of the functions seem redundant
# but they are here to facilitate dynamic creation
# of controllers. 


import rospy
from dynamixel_controllers.controller_spawner import manage_controller

#Import services
from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController

#Import messages
from dynamixel_controllers.msg import MotorState


class ManagerProxy:
    def __init__(self, manager_ns, port_ns_vect, timeout=0.5):
        """Initiates the controller manager.
            Arguments:
                manager_ns -- A string containing the namespace
                                    of the manager
                port_ns_vect -- A list of strings, each containing a 
                                    port namespace that is used by the
                                    dynamixel controller manager.
                timeout -- The time to wait for dynamixel_controllers
                            controller manager to spawn.
        """

        #Ensure that provided arguments are non-empty
        if not manager_ns or not port_ns_vect:
            raise ValueError('bad namespace(s)')

        self._manager_ns = manager_ns
        self._ports = []
        self._timeout = timeout

        #Caches the controller command publishers
        self._cmds = {}

        #Service proxies for our controller managing calls
        self._start = {}
        self._stop = {}
        self._restart = {}
        

    #Assumes that the parameters have already been pushed to the
    #parameter server
    def _cmd_controller(self, controller_name, port_ns, controller_type, command, deps):

        #Check that we have initialised our service proxies for the 
        #given port
        if command.lower() not in ['start', 'restart', 'stop']:
            raise ValueError('command unrecognised')

        #Create appropriate service proxies if we haven't already
        if port_ns not in self._ports:
            start_name = '%s/%s/start_controller'%(self._manager_ns, port_ns)
            stop_name = '%s/%s/stop_controller'%(manager_namespace, port_ns)
            restart_name = '%s/%s/restart_controller'%(manager_namespace, port_ns)
            
            rospy.wait_for_service(start_name, timeout=self._timeout)
            rospy.wait_for_service(stop_name, timeout=self._timeout)
            rospy.wait_for_service(restart_name, timeout=self._timeout)

            self._start[port_ns] = rospy.ServiceProxy(start_name,  
                                                      StartController)
            self._stop[port_ns] = rospy.ServiceProxy(stop_name,
                                                     StopController)
            self._restart[port_ns] = rospy.ServiceProxy(restart_name,
                                                        RestartController)

            self._ports.append(port_ns)
        
        manage_controller(  controller_name, port_ns, controller_type,
                            command, deps, self._start[port_ns], 
                            self._stop[port_ns], self._restart[port_ns])


    def start_torque_cont(self, controller_name, port_ns):
        self._cmd_controller(controller_name, port_ns,
                             'simple', 'start', '') 

    def start_pos_cont(self, controller_name, port_ns):
        self._cmd_controller(controller_name, port_ns,
                             'simple', 'start', '') 

    def stop_torque_cont(self, controller_name, port_ns):
        self._cmd_controller(controller_name, port_ns,
                             'simple', 'stop', '') 

    def stop_pos_cont(self, controller_name, port_ns):
        self._cmd_controller(controller_name, port_ns,
                             'simple', 'stop', '') 

    def restart_torque_cont(self, controller_name, port_ns):
        self._cmd_controller(controller_name, port_ns,
                             'simple', 'restart', '') 

    def restart_pos_cont(self, controller_name, port_ns):
        self._cmd_controller(controller_name, port_ns,
                             'simple', 'restart', '') 


    def start_meta_cont(self, controller_name, port_ns, deps):
        self._cmd_controller(controller_name, port_ns,
                             'meta', 'start', deps) 


    def stop_meta_cont(self, controller_name, port_ns, deps):
        self._cmd_controller(controller_name, port_ns,
                             'meta', 'stop', deps) 

    def restart_meta_cont(self, controller_name, port_ns, deps):
        self._cmd_controller(controller_name, port_ns,
                             'meta', 'restart', deps) 

    def command(self, controller_name, value):
        #Create and cache publisher
        if controller_name not in self._cmds:
            self._cmds[controller_name] = rospy.Publisher(controller_name +'/command', std_msgs.msg.Float64, queue_size=10)

        self._cmds[controller_name].publish( float( value ) )

    def set_state_callback(self, controller_name, callback):
        rospy.Subscriber(controller_name + '/state', , callback)
        

