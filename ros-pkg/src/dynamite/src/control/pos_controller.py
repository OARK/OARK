#! /usr/bin/env python
#
# Contains a class to represent a position controller.
# A position controller controls a joint and moves it 
# to a desired angle.
#
# For now we assume that all of the controller parameters
# are already on the parameter server. This will be changed
# in future to allow easy dynamic controller creation.
#
# The prototypical nature of this class means that it
# currently has many thorns. It should be used with
# caution lest the user be pricked.

import rospy
import controller
import threading


class PosController(controller.Controller):
    def __init__(self, proxy, controller_name, port_ns, wait=True):
        #Initialise fields of parent class
        controller.Controller.__init__(self, proxy, controller_name, port_ns)

        #Create controller
        proxy.start_pos_cont(controller_name, port_ns)
        if wait:
            rospy.wait_for_service('/%s/set_speed'%(controller_name,))

        self.hardware_init = True

    def _pos_to_cmd(self, position):
        """Translates a position value to a controller command. The format
        of this method will determine the units that the torque is
        supplied in."""
        return position

    #TODO - Specify position in a sensible unit
    def set_position(self, position):
        self.command(self._pos_to_cmd(position))

    def set_position_buffered(self, position):
        """Buffers a command to set the position of the controller.
        The command will not be executed until flush_cmd is called."""
        self.queue_cmd(self._pos_to_cmd(position))

    #TODO: Implement meaningfully
    def stop(self):
        pass


#Used only for testing
import time

#Cute little test harness.
if __name__ == '__main__':
    rospy.init_node('other_node')
    man_proxy = manager_proxy.ManagerProxy('dxl_manager')
    pos = PosController(man_proxy, 'wheel_fl', 'pi_out_port')
    time.sleep(3)
    pos.command(2.0)
    time.sleep(3)
    pos.command(0.0)
    time.sleep(3)
    pos.command(-2.0)
    time.sleep(3)
    pos.command(0.0)
    rospy.spin()

