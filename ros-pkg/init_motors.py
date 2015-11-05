#!/usr/bin/env python

"""A short script to read an OARK robot YAML configuration file and
configure the AX12 motors appropriately. This configuration occurs
*inside* the motor EEPROM rather than any of the Raspberry Pi or
OpenCM software.

The script that is intended to be used to set the motor configs is
set_servo_config.py from the dynamixel_motor ROS stack. This file
does not export very useful return statuses, and so it's difficult
for this script to return useful return statuses.
"""

import sys
import yaml
import argparse
import subprocess

__author__ = 'Tim Peskett'
__copyright__ = 'Copyright 2015, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


def config_motor(motor_script, data, baud, serial):
    """Initialises a single AX12 motor to the values specified
    in data.
    Params:
        motor_script - The address of the script to use to configure
                       the motors. Should be set_servo_config.
        data         - The dictionary to use to write the motor config.
                       Should have at least the following 
                       fields: id, min, max, type.
        baud         - The baud rate of the serial link.
        serial       - The filename of the serial link.
    Returns nothing.

    Throws OSError if motor_script not found.
    Throws ValueError if values in data are invalid
    """
    #Set appropriate (counter)clockwise angle
    cw_angle = 0
    ccw_angle = 0
    if data['type'].lower() == 'torque':
        cw_angle = ccw_angle = 0
    elif data['type'].lower() == 'position':
        cw_angle = data['min']
        ccw_angle = data['max']

    #Synthesise correct command
    cmd = [motor_script, 
           '-b', str(baud),
           '-p', str(serial),
           '--cw-angle-limit=' + str(cw_angle),
           '--ccw-angle-limit=' + str(ccw_angle),
           str(data['id'])]

    #Call process
    subprocess.call(cmd)


def init_motors(motor_script, config_fname, baud, serial):
    """Initialises the AX12 motors to the values specified
    in config_file. If an error is encountered, the motors will
    be left in an unknown state.
    Params:
        motor_script - The address of the script to use to configure
                       the motors. This should be the set_servo_config
                       script of the dynamixel_motor stack.
        config_fname - The file path of the configuration file
                       containing the motor configurations.
        baud         - The baud rate of the serial link to write over.
        serial       - The filename of the serial port to write over.
    Returns nothing
    
    Throws KeyError if bad config and IOError if bad file
    """
    config_file = open(config_fname, 'r')
    motor_data = yaml.load(config_file.read())
    config_file.close()

    for name, data in config_file['controllers'].iteritems():
        config_motor(motor_script, data, baud, serial)


if __name__ == '__main__':
    #Parse command line
    parser = argparse.ArgumentParser(description='Initialise motors using an OARK config file')
    parser.add_argument('config_filename',
                        help='The file path of the OARK configuration file') 
    parser.add_argument('-m', metavar='SCRIPT', dest='script',
                        default='/opt/ros/indigo/lib/dynamixel_driver/set_servo_config.py',                        
                        help='The file path of the script used to configure the motors. (Usually set_servo_config.py) [default: %(default)s]')
    parser.add_argument('-p', metavar='PORT', dest='port', 
                        default='/dev/ttyAMA0', help='the port that the AX12s are connected on [default: %(default)s]')
    parser.add_argument('-b', metavar='BAUD', type=int, dest='baud',
                        default=115200, help='the baud rate of the AX12 port [default: %(default)s]')
    args = parser.parse_args()


    retval = 0
    try:
        init_motors(args.script,
                    args.config_filename,
                    args.baud,
                    args.port)
    except (KeyError, ValueError) as e:
        print 'Config file malformed (possibly missing fields?):', str(e)
        #Return fail
        retval = 1
    except IOError as e:
        print 'Could not open file: ', str(e)
        #Return fail
        retval = 1
    except OSError as e:
        print 'Could not find motor_script file: ', str(e)
        #Return fail
        retval = 1
 
    sys.exit(retval)
