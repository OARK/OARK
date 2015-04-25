#! /usr/bin/env python
#
# Contains a class to represent a torque controller.
# A torque controller controls a joint and applies
# a constant torque to it until otherwise desired.
#
# The prototypical nature of this class means that it
# currently has many thorns. It should be used with
# caution lest the user be pricked.


import controller
import threading


class TorqueController(controller.Controller):
    def __init__(self, proxy, controller_name):
        #Initialise fields of parent class
        controller.Controller.__init__(self)

        #Create controller
        proxy.start_torque_cont(controller_name, port_ns)

    def destroy(self):
        proxy.stop_torque_cont(self.controller_name, self.port_ns)

    def command(self, value):
        self.proxy.command(self.controller_name, value)

    def set_torque(self, position):
        self.proxy.command(self.controller_name, torque)
