# !/usr/bin/env python
# msg.py
# Contains a very basic class that represents
# a message. The message need not be a command
# for a controller but usually is. Any fields
# that are unused for a particular command will
# be initialised to zero.
#
#
# The current (test) message format is as follows:
#
# Start Length      Field
# ----- ------      -----
#    0       1      Message type
#    1       1      Controller ID. ID of the controller to send the
#                   command to. 0 if unused.
#    2       1      Value to send to the controller. This could
#                   be either a position of torque.
#    3       4      Time to send to the controller. This is the
#                   time (in milliseconds). This should be an 32-bit value
#                   formatted in network order.

import struct

class Msg:
    def __init__(self, net_data):
        (self.type,
         self.id, 
         self.val, 
         self.duration) = struct.unpack('!BBBi', net_data)

    def get_type(self):
        return self.type

    def get_id(self):
        return self.id

    def get_value(self):
        return self.val

    def get_duration(self):
        return self.duration

    def __str__(self):
        return "Type:%d Id:%d Value:%d Duration:%d"%(self.type, self.id, self.val, self.duration)
