#!/usr/bin/env python
#A file used to make a packet accepted by cmd_listener.py
#so that it can be piped into netcat to test the functionality
#of the network node.

import StringIO
import sys

from emumini2.msg import Command


MSG_TYPE = 1


if __name__ == '__main__':
    if len(sys.argv) <= 7:
        print 'Usage: makepacket.py left_analog_x left_analog_y right_analog_x etc'
        exit(1)

    values = map(float, sys.argv[1:8])
    cmd = Command(values=values)

    buff = StringIO.StringIO()
    cmd.serialize(buff)

    buff_list = [ord(a) for a in buff.getvalue()]
    #Assumes that message is less than 255 bytes
    out_msg = ''.join(map(chr, [ MSG_TYPE, 0, len(buff_list) ] + buff_list))
    sys.stdout.write(out_msg)

