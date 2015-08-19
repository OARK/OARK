#!/usr/bin/env python
#
# Useful functions to handle ttys like serial ports


import termios


#Constants for setting terminal attributes
IFLAG  = 0
OFLAG  = 1
CFLAG  = 2
LFLAG  = 3
ISPEED = 4
OSPEED = 5
CCSPEED= 6

RM_IFLAG =(termios.IGNBRK |
           termios.BRKINT |
           termios.ICRNL  |
           termios.INLCR  |
           termios.PARMRK |
           termios.INPCK  |
           termios.ISTRIP |
           termios.IXON)

RM_LFLAG = (termios.ECHO   |
            termios.ECHONL |
            termios.ICANON |
            termios.IEXTEN |
            termios.ISIG)

RM_CFLAG = (termios.CSIZE  |
            termios.PARENB)

AD_CFLAG = (termios.CS8)



def cfmakeraw(tty):
    """Applies the correct settings to a terminal to make it appear
    as if it is a serial port
    """
    attr = termios.tcgetattr(tty)
    
    #Apply the correct attributes
    attr[IFLAG] = attr[IFLAG] & ~RM_IFLAG
    attr[OFLAG] = 0
    attr[LFLAG] = attr[LFLAG] & ~RM_LFLAG
    attr[CFLAG] = attr[CFLAG] & ~RM_CFLAG
    attr[CFLAG] = attr[CFLAG] | AD_CFLAG
    attr[CCSPEED][termios.VMIN] = 1
    attr[CCSPEED][termios.VTIME] = 0

    termios.tcsetattr(tty, termios.TCSADRAIN, attr)
