#! /usr/bin/env python
#
# Contains a class to represent a torque controller.
# A torque controller controls a joint and applies
# a constant torque to it until otherwise desired.
#
# For now we assume that all of the controller parameters
# are already on the parameter server. This will be changed
# in future to allow easy dynamic controller creation.

# The prototypical nature of this class means that it
# currently has many thorns. It should be used with
# caution lest the user be pricked.



import controller
import threading
import manager_proxy


class TorqueController(controller.Controller):
    def __init__(self, proxy, controller_name, port_ns):
        #Initialise fields of parent class
        #Creates object fields: self.proxy, self.controller_name, and
        # self.port_ns
        controller.Controller.__init__(self, proxy, controller_name, port_ns)

        #Create controller
        proxy.start_torque_cont(controller_name, port_ns)

    def destroy(self):
        self.proxy.stop_torque_cont(self.controller_name, self.port_ns)

    def command(self, value):
        self.proxy.command(self.controller_name, value)

    #TODO - Specify torque in sensible unit
    def set_torque(self, torque):
        self.command(torque)

    def stop(self):
        self.set_torque(0)



#Used only for testing
import rospy
import time

#Cute little test harness.
if __name__ == '__main__':
    rospy.init_node('torque_test_node')
    man_proxy = manager_proxy.ManagerProxy('dxl_manager')
    pos = TorqueController(man_proxy, 'wheel_fl', 'pi_out_port')
    time.sleep(3)
    pos.command(2.0)
    time.sleep(3)
    pos.command(0.0)
    time.sleep(3)
    pos.command(-2.0)
    time.sleep(3)
    pos.command(0.0)
    rospy.spin()

