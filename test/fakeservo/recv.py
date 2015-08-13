#!/usr/bin/env python
#
# The entry point of the application.


#from serial import Serial
import pty
import os
import time
import sys

if len(sys.argv) < 2:
    print 'Need tty as filename'
    exit(0)

#peer = serial.Serial()
tty_fd = os.open(sys.argv[1], os.O_RDONLY)
while 1:
    print hex(ord(os.read(tty_fd, 1)))
