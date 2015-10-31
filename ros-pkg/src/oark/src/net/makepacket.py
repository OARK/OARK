#!/usr/bin/env python

"""A module used to make a packet accepted by cmd_listener.py
so that it can be piped into netcat to test the functionality
of the network node.
"""

import StringIO
import sys

import net_consts

from emumini2.msg import Command, Input
from emumini2.srv import InputRequestRequest, InputRequestResponse


def make_command(values):
    """Returns a serialized string that contains the command packet
    with values inside of it
    """
    cmd = Command(values=map(float, values))
    return wrap_data(net_consts.MSG_TYPES[Command], msg_to_string(cmd))
make_command.usage = 'float1 float2 float3 float4 ...'


def make_inputrequest(request):
    """Make a sample input request packet.
    """

    inputrequest = InputRequestRequest()
    return wrap_data(net_consts.MSG_TYPES[InputRequestRequest], msg_to_string(inputrequest))
make_inputrequest.usage = ""


def make_fake_inputresponse(inputs):
    """Makes a fake input response packet. Serialises it as a string.
    inputs should be of the form
    ['input1_name', 'input1_type', 'input1_title', 'input1_axes',
     'input2_name', ...
    ]
    """
    input_list = list()
    for i in range(0, len(inputs), 4):
        input_list.append(Input(name=inputs[i], type=inputs[i+1], title=inputs[i+2], axes=inputs[i+3]))
    out_string = msg_to_string(InputRequestResponse(inputs=input_list))
    return wrap_data(net_consts.MSG_TYPES[InputRequestResponse], out_string)
make_fake_inputresponse.usage = 'input1_name input1_type input1_title input1_axes input2_name ...'
    

def wrap_data(msg_type, buffer):
    """Wraps data and makes it into a packet ready to be sent over the network.
    """
    #Calculate length of packet
    msg_len_rev = []
    for i in range(net_consts.MSG_HEADER_LEN-1):
        msg_len_rev.append((len(buffer) >> (i*8)) % 256)

    #Create message to send
    return chr(msg_type) + ''.join(map(chr, reversed(msg_len_rev))) + buffer


def msg_to_string(msg):
    """Serialises a message and returns it as a python string.
    """
    buff = StringIO.StringIO()
    msg.serialize(buff)

    #Assumes that message is less than 255 bytes
    out_msg = ''.join([chr(ord(a)) for a in buff.getvalue()])
    return out_msg



if __name__ == '__main__':
    """Entry point. Handles user input and output
    """
    #Copy so that we are iterating over an active dictionary
    old_globals = dict(globals())

    functions = dict()
    for fun_name, fun in old_globals.iteritems():
        if fun_name.find('make_', 0) != -1:
            packet_name = fun_name.lstrip('make').lstrip('_')
            functions[packet_name] = fun

    if len(sys.argv) < 2:
        print sys.argv[0]
        print 'Synthesises packets for testing oark software'
        print 'Available packet types:'
        for packet_name in functions:
            print '\t', packet_name
        print 'Use %s packet_name -h to get usage for a particular packet'%(sys.argv[0],)
        exit(1)

    try:
        packet_func = functions[sys.argv[1]]
    except KeyError, ke:
        print 'No such packet found: ', sys.argv[1]
        exit(1)

    if len(sys.argv) > 2 and sys.argv[2] == '-h':
        print 'Usage: ', sys.argv[0], sys.argv[1], packet_func.usage
        exit(0)

    try:
        result = packet_func(sys.argv[2:])
        sys.stdout.write(result)
        sys.stdout.flush()
    except Exception, e:
        print "Error occurred: ", str(e)
