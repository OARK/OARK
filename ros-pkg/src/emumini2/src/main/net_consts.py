#!/usr/bin/env python

"""Defines constants and other data that are required for the networking
oark package. It is better to have these constants separated out from the
data so that they can be easily changed and identified if required.
"""

from emumini2.msg import Command, Input
from emumini2.srv import InputRequestRequest, InputRequestResponse

#The length of the header in the message. First byte is always the 
#message type and the following bytes make up the length in network
#byte order (Big endian)
MSG_HEADER_LEN = 3


#Defines the mapping between message type numbers and the ros message
#that they instantiate. This is needed so that the message type can be
#sent over the network.
MSGS = {
            1: Command,
            2: InputRequestRequest,
            3: InputRequestResponse,
       }

#Create the reverese association for easy lookup
MSG_TYPES = dict(map(lambda m: (m[1], m[0]), MSGS.items()))
