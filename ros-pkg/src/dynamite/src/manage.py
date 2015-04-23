#!/usr/bin/env python

#Manages joint controllers. Allows dynamic creation, destruction, and 
#grouping of joint controllers.
#
# THIS FILE IS CURRENTLY OBSOLETE AS IT IS DOWN A DESIGN PATH THAT I CHOSE
# NOT TO PURSUE. IT IS BEING KEPT FOR ARCHIVAL PURPOSES

import rospy

from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController

from dynamixel_controllers.controller_spawner import manage_controller


class ControllerManager:
    """Managers all of the joint controllers for the robot.
        Allows them to be added, destroyed and grouped
        arbitrarily at runtime."""

    def __init__( self, manager_namespace, port_namespaces ):
        """Initiates the controller manager.
            Arguments:
                manager_namespace -- A string containing the namespace
                                    of the manager
                port_namespaces -- A list of strings, each containing a 
                                    port namespace that is used by the
                                    dynamixel controller manager.
        """

        if not port_namespaces:
            raise ValueError

        self._start = {}
        self._stop = {}
        self._restart = {}
        
        for port_ns in port_namespaces:
            start_service_name = '%s/%s/start_controller' % (manager_namespace, port_ns )
            stop_service_name = '%s/%s/stop_controller' % (manager_namespace, port_ns )
            restart_service_name = '%s/%s/restart_controller' % (manager_namespace, port_ns )
            
            rospy.wait_for_service( start_service_name )
            rospy.wait_for_service( stop_service_name )

            rospy.wait_for_service( restart_service_name )

            self._start[port_ns] = rospy.ServiceProxy( start_service_name,  
                                                       StartController )
            self._stop[port_ns] = rospy.ServiceProxy( stop_service_name,
                                                      StopController )
            self._restart[port_ns] = rospy.ServiceProxy( restart_service_name,
                                                         RestartController )

    def __destroy__( self ):
        pass

    
    def create_pos( self, name, motor_id, init = 512, min_angle = 0, max_angle = 1023 ): 
        pass

    def create_torque( self, name, motor_id, init = 0, min_torque = 0, max_torque = 4095 ):
        pass

    def destroy( self, name ):
        pass

    def set_

    def group( self, names ):
        pass


