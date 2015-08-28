#!/usr/bin/env python
#
# The entry point of the application.

#TODO: Go through and change strings to bytearrays

import pty
import os
import time
import sys
import threading
import yaml

import serial
import ax12
import const


def full_read(tty, num_bytes):
    """Reads exactly num_bytes from the given tty
    """
    input = []

    while len(input) < num_bytes:
        input.extend(list(os.read(tty, num_bytes - len(input))))

    return map(ord, input)

def full_write(tty, out_list, num_bytes):
    """Writes exactly num_bytes to the given tty
    """
    num_read = 0

    while num_read < num_bytes:
        num_read = num_read + os.write(tty, str(bytearray(out_list[num_read:])))



def emulate_and_send(**kwargs):
    """Emulate motor running with packet and output status packet to
       tty (if applicable) after getting mutex.
       It is assumed that this method will be run in a different thread.
    """
    motor = kwargs['motor']
    tty = kwargs['tty']
    mutex = kwargs['mutex']
    packet = kwargs['packet']

    status = motor.receive(packet)
    if status is not None:
        with mutex:
            full_write(tty, status, len(status))



if __name__ == '__main__':
    if len(sys.argv) < 3:
        print ('Usage: %s <serial port name> <motor file>\n\
                Emulates the motors specified in <motor file> over the \
                virtual serial port <serial port name>.'%sys.argv[0])
        exit(0)

    mutex = threading.Lock()

    #Load motor configuration
    m_file = open(sys.argv[2], 'r')
    motor_cfg = yaml.load(m_file.read())

    #Create motors
    motor_dict = {cfg['id']: ax12.AX12(cfg['id'], name) for name, cfg in motor_cfg.iteritems()}
    print motor_dict

    #Open ther pseudo terminal that will become our serial port
    master_fd, slave_fd = pty.openpty()
    os.symlink(os.ttyname(slave_fd), sys.argv[1])

    #Configure terminal to make it like a serial port
    serial.cfmakeraw(slave_fd)
    serial.cfmakeraw(master_fd)
    
    while 1:
        input = full_read(master_fd, 1)
        
        #Read first byte of header
        if input == [0xFF]:
            input.extend(full_read(master_fd, 1))
            if input == [0xFF, 0xFF]:
                #Header found. Read instruction and length
                input.extend(full_read(master_fd, 2))
                id = input[2]
                length = input[3]

                #Read the rest of the packet
                input.extend(full_read(master_fd, length))

                print 'RECEIVED A MESSAGE', input

                #Emulate motors in new thread and send results back
                if id == const.ID_BROADCAST:
                    for motor in motor_dict.values():
                        threading.Thread(
                                target=emulate_and_send,
                                kwargs={'motor': motor, 
                                        'tty': master_fd,
                                        'mutex': mutex,
                                        'packet': input}
                        ).start()
                else:
                    threading.Thread(
                                target=emulate_and_send,
                                kwargs={'motor': motor_dict[id], 
                                        'tty': master_fd,
                                        'mutex': mutex,
                                        'packet': input}
                        ).start()
