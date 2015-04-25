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

import controller
import threading


class PosController(controller.Controller):
    def __init__(self, proxy, controller_name, port_ns):
        #Initialise fields of parent class
        controller.Controller.__init__(self)

        #Create controller
        proxy.start_pos_cont(controller_name, port_ns)
        
    def destroy(self):
        proxy.stop_pos_cont(self.controller_name, self.port_ns)

    def command(self, value):
        self.proxy.command(self.controller_name, value)

    def set_position(self, position):
        self.proxy.command(self.controller_name, position)
