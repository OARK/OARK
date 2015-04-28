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
import time
import manager_proxy


class PosController(controller.Controller):
    def __init__(self, proxy, controller_name, port_ns):
        #Initialise fields of parent class
        controller.Controller.__init__(self, proxy, controller_name, port_ns)

        #Create controller
        proxy.start_pos_cont(controller_name, port_ns)
        
    def destroy(self):
        self.proxy.stop_pos_cont(self.controller_name, self.port_ns)

    def command(self, value):
        print "Holler back at ya dawg : " + str(self.controller_name)
        print "Value:", value
        self.proxy.command(self.controller_name, value)

    def set_position(self, position):
        self.proxy.command(self.controller_name, position)



#Cute little test harness.
if __name__ == '__main__':
    rospy.init_node('other_node')
    man_proxy = manager_proxy.ManagerProxy('dxl_manager')
    pos = PosController(man_proxy, 'wheel_cont', 'pi_out_port')
    time.sleep(3)
    pos.command(2.0)
    time.sleep(3)
    pos.command(0.0)
    time.sleep(3)
    pos.command(-2.0)
    time.sleep(3)
    pos.command(0.0)
    rospy.spin()

