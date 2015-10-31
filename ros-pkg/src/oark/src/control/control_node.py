#!/usr/bin/env python
# The code in this file needs to be made generic. Config files should be
# made to specify the motors. For now, the values are hardcoded.

"""A module to act as a ROS node and create relevant services/topics
that other ROS nodes can use to interact with the motors.
"""

import rospy
import sys
import argparse
import threading
import yaml
import math

from manager import DXLManager

from oark.msg import Command
from oark.srv import GetInputs, GetInputsResponse
from oark.srv import Connected, ConnectedResponse
from oark.msg import Input
from std_srvs.srv import Empty, EmptyResponse


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2015, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']

NODE_NAME = 'oark_control_node'
#Update motor states 20 times a second
UPDATE_RATE = 20


class OARKNode(object):
    """A node to receive messages from other nodes and
    control the motors based on these messages.
    """

    def __init__(self, node_ns, man_ns, port_ns, config_filename):
        """Create the OARKNode object. The object will receive messages
        from the networking node and control the motors based on these
        messages.
        Params:
            node_ns - The name of this node.
            man_ns  - The namespace of the dynamixel manager (specified in the roslaunch file)
            port_ns - The namespace of the dynamixel port (specified in the roslaunch file)
            config_filename - The filename of the config file
        """
        #Read the controller configuration from a file
        config_file = open(config_filename, 'r')
        self.config = yaml.load(config_file.read())
        config_file.close()

        #Save config file in this object
        self.input_list = self.config['inputs']
        self.cont_dict = self.config['controllers']
        self._has_video = self.config.get('video', False)

        self.dxl_mgr = DXLManager(man_ns, port_ns)
        self._create_controllers(self.cont_dict)

        #Initialise the evaluation function symbol table
        #Updated before each evaluation
        self._sym_table = dict(math.__dict__)

        #Compile the evaluation function for each controller
        self.eval_funcs = dict()
        try:
            for name, data in self.cont_dict.iteritems():
                self.eval_funcs[name] = compile(data['value'], '<string>', 'eval')
        except KeyError, ke:
            raise ConfigException('\'value\' field not present on controller %s'%(name,))
        except (SyntaxError, TypeError) as e:
            raise ConfigException('\'value\' field not valid python expression on controller %s'%(name,))

        #Create a mutex to allow multithreaded subscriber
        self._cmd_mutex = threading.Lock()
        rospy.Subscriber('/%s/command'%(node_ns,), Command, self.cmd_rcvd)

        #Wait for video node if available
        if self._has_video:
            rospy.wait_for_service('/oark_vid_node/start_stream')

        #Create services to allow caller to retrieve data
        rospy.Service('/%s/connected'%(node_ns,), Connected, self.connected)
        rospy.Service('/%s/disconnected'%(node_ns,), Empty, self.disconnected)
        rospy.Service('/%s/get_inputs'%(node_ns,), GetInputs, self.get_inputs)
        #rospy.Service('/%s/get_motor_state'%(node_ns,), , self.get_motor_state)

        #Create service proxies with which to call other services
        self.stop_stream = rospy.ServiceProxy('/oark_vid_node/stop_stream',
                                              Empty)
        self.start_stream = rospy.ServiceProxy('/oark_vid_node/start_stream',
                                               Connected)


    def _create_controllers(self, cont_dict):
        """A helper function to take the yaml-specified motor configuration data
        and create the controllers for them. The point of this method is really
        just to keep the constructor clean.
        """
        rospy.loginfo('Creating controllers...')
        try:
            for name, data in cont_dict.iteritems():
                #Validate data
                if data['type'].lower() == 'torque':
                    cont_type = DXLManager.TORQUE_CONTROLLER
                elif data['type'].lower() == 'position':
                    cont_type = DXLManager.POS_CONTROLLER
                else:
                    raise ConfigException('Invalid type (%s) for controller %s'%(data['type'], name,))

                #Create controller
                self.dxl_mgr.create_controller(data['id'],
                                               name,
                                               cont_type,
                                               min=data['min'], max=data['max'],
                                               init=(data['max'] + data['min'])/2.0,
                                               joint_name=data['joint_name'])
        except KeyError, ke:
            raise ConfigException('Field not found', ke)
        rospy.loginfo('Controllers created!')



    def run(self):
        """The main loop for the node. This method will flush the commands to
        the motors a few times a second until the node is shutdown.
        """
        rospy.loginfo('Control node running')
        choose_func = lambda q: ([q[-1]] if q else [])
        rate = rospy.Rate(UPDATE_RATE)
        while not rospy.is_shutdown():
            for name, cont in self.dxl_mgr:
                cont.flush(choose=choose_func)

            rate.sleep()


    def cmd_rcvd(self, cmd):     
        """Called by ROS whenever a command is received from the network
        node. cmd is a Command message object. The Command is parsed and
        the motors are moved appropriately.
        """
        with self._cmd_mutex:
            #Associate each input value with the name of the
            #input to which it corresponds
            syms = dict()
            i = 0
            for input in self.input_list:
                if input['type'] == 'analog':
                    syms[input['name'] + '_x'] = cmd.values[i]
                    syms[input['name'] + '_y'] = cmd.values[i+1]
                    i = i + 2
                else:
                    syms[input['name']] = cmd.values[i]
                    i = i + 1


            self._sym_table.update(syms)


            #Evaluate all functions and supply values to controllers
            for cont_name, eval_func in self.eval_funcs.iteritems():
                try:
                    out_value = float(eval(eval_func, self._sym_table))
                    self.dxl_mgr[cont_name].queue(out_value)
                except ValueError, ve:
                    rospy.logwarn('Invalid python expression for controller ' + str(cont_name))
                except AttributeError, ae:
                    rospy.logwarn('Unrecognised attribute in expression for controller ' + str(cont_name))
                    rospy.logwarn(str(ae))
                except NameError, ne:
                    rospy.logwarn('Unrecognised name in python expression for controller ' + str(cont_name))
                    rospy.logwarn(str(ne))


    def connected(self, req):
        """Called when a client connects to the robot.
        Req contains the IP address of the connected robot.
        """
        rospy.loginfo('Client connected')
        if self._has_video:
            return self.start_stream(req)
        else:
            #Nothing to do, return success
            return ConnectedResponse(1)

    
    def disconnected(self, req):
        """Called when a client disconnects from the robot.
        Req is the Empty message.
        """
        if self._has_video:
            return self.stop_stream(req)
        else:
            return EmptyResponse()

    
    def get_inputs(self, req):
        """Called by ROS when the networking node receives a request
        for the inputs. The inputs usually map to widgets on the
        android device. Each input is a float value between -1.0 and 1.0, and
        each motor uses these values to set its own value.
        """
        #Create Input objects out of input yaml dictionaries
        out_inputs = map(lambda i: Input(**i), self.input_list)
        return GetInputsResponse(inputs=out_inputs)


    #def get_motor_state(self, req): 
        #pass


#An exception thrown when the configuration file is malformed
class ConfigException(Exception):
    pass




if __name__ == '__main__':
    """The entry point for the application. Parses command line arguments and
    then starts an OARKNode
    """
    #Parser command line
    parser = argparse.ArgumentParser(description='Start the oark control node')
    parser.add_argument('manager_namespace',
                        help='The manager namespace provided to the dynamixel driver software') 
    parser.add_argument('port_namespace',
                        help='The port name provided to the dynamixel driver software')
    parser.add_argument('controller_file',
                        help='A file containing configuration data for the controllers and controls')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    try:
        rospy.init_node(NODE_NAME)
        node = OARKNode(NODE_NAME,
                        args.manager_namespace,
                        args.port_namespace,
                        args.controller_file)
        node.run()
    except ConfigException, ce:
        rospy.logerr('Error with configuration file occured:')
        rospy.logerr(str(ce))
    except rospy.ROSInterruptException, rie:
        rospy.loginfo('Node exiting...')
