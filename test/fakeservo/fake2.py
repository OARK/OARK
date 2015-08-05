#!/usr/bin/env python
#
# The entry point of the application.


#from serial import Serial
import pty
import os
import time
import termios
import sys





if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Provide a path to place the new serial port'
        exit(0)

    #Open ther pseudo terminal that will become our serial port
    master_fd, slave_fd = pty.openpty()
    os.symlink(os.ttyname(slave_fd), sys.argv[1])

    #Configure terminal to make it like a serial port
    attr = termios.tcgetattr(slave_fd)
    print 'Attributes ', attr
    attr[IFLAG] = attr[IFLAG] & ~RM_IFLAG

    attr[OFLAG] = 0

    attr[LFLAG] = attr[LFLAG] & ~RM_LFLAG

    attr[CFLAG] = attr[CFLAG] & ~RM_CFLAG
    attr[CFLAG] = attr[CFLAG] | AD_CFLAG

    attr[CCSPEED][termios.VMIN] = 1
    attr[CCSPEED][termios.VTIME] = 0

    termios.tcsetattr(slave_fd, termios.TCSADRAIN, attr)
    
    while 1:
        print os.write(master_fd, 'M')
        time.sleep(1)

