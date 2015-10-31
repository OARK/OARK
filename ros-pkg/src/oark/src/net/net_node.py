#!/usr/bin/env python

"""A node to receive messages from the network and coordinate
these messages/events with other running nodes.
"""

import sys
import rospy
import argparse
import StringIO

from cmd_listener import CmdListener
import net_consts

from oark.msg import Command
from oark.srv import StartStream
from oark.srv import GetInputs, GetInputsRequest, GetInputsResponse
from std_srvs.srv import Empty


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2015, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


NODE_NAME='oark_net_node'


class NetNode(object):
    """A network node. Creates methods used as callbacks from
    CmdListener. CmdListener will receive network messages
    and send them to this node. This node will then process them
    and send them on to the control node.
    """

    def __init__(self, node_name, port):
        """Create the NetNode object. Registers the methods in
        this NetNode instance as callbacks for the CmdListener
        object. This is an 'inversion of control' in some respects.
        Params:
            node_name - The name of the ROS node.
            port      - The port to send/receive commands on
        """
        #Create network communication object
        self.listener = CmdListener('', port)

        #Add listeners
        self.listener.add_connect_listener(self.on_connect)
        self.listener.add_data_listener(self.on_data)
        self.listener.add_dc_listener(self.on_dc)

        self.node_name = node_name

        #Create publishers/service proxies for communicating with the
        #control node.
        self.get_inputs_srv = rospy.ServiceProxy('/oark_control_node/get_inputs',
                                                 GetInputs)
        self._command_pub = rospy.Publisher('/oark_control_node/command',
                                            Command,
                                            latch=True,
                                            queue_size=10)

        self.stop_stream = rospy.ServiceProxy('/oark_vid_node/stop_stream',
                                              Empty)
        self.start_stream = rospy.ServiceProxy('/oark_vid_node/start_stream',
                                               StartStream)


    def on_connect(self, addr):
        """Called when a connection is received. Will start the video
        stream if available
        Params:
            addr - The IP address of the connected device.
        """
        rospy.loginfo('Connected to ' + str(addr))

        #The first member of the addr tuple is the IP address
        self.start_stream(str(addr[0]))


    def on_data(self, msg_type, msg):
        """Receives a message from the caller, turns it into the appropriate
        ROS message and then forwards it to another function to handle the message.
        This method is assumed to be called in a separate thread and so
        is allowed to block if necessary.
        Params:
            msg_type - An integer identifier of the message. All network
                       messages have a message type. The mapping for this
                       can be found in net_consts.
            msg      - A serialized string containing the message data.
        """
        rospy.loginfo('Got data')

        #Deserialise msg
        #print 'Packet', [ord(a) for a in msg]
        ros_msg = net_consts.MSGS[msg_type]()
        ros_msg.deserialize(msg)

        try:
            #Send message to its own handler function
            msg_name = net_consts.MSGS[msg_type].__name__

            print 'Received message ', msg_name, ' of type ', msg_type

            msg_func = getattr(self, 'on_' + msg_name.lower())
            msg_func(ros_msg)
        except KeyError, ke:
            rospy.logwarn(msg_type, ' is not a valid message type')
        except AttributeError, ae:
            rospy.logwarn('No function to handle ROS msg <' + msg_name + '>')


    def on_command(self, cmd):
        """Handles ROS Command messages. These command messages consist of a
        variable length array of floats. The exact format of this message
        does not need to be known here as it is just pushed off to the 
        control node
        """
        self._command_pub.publish(cmd)


    def on_getinputsrequest(self, inputrequest):
        """Handles ROS GetInputsRequest messages. When this command is received
        the node will call the appropriate service in the control node, and then
        send the result back over the network.
        Params:
            inputrequest - A GetInputsRequest ROS message.
        """

        #Call the control node
        result = self.get_inputs_srv(inputrequest)
        #Send the result of the call over the network
        self.listener.send_str(net_consts.MSG_TYPES[GetInputsRequest], self._msg_to_string(result))


    def on_dc(self):
        """Called when a client disconnects.
        """
        rospy.loginfo('Disconnected')
        self.stop_stream()

    
    def shutdown(self):
        self.listener.shutdown()

    def _msg_to_string(self, msg):
        """Takes a ROS message and converts it to a serialised string. 
        Essentially just wraps msg.serialize()
        Params:
            msg - A ROS message.
        """
        buffer = StringIO.StringIO()
        msg.serialize(buffer)
        #Serialise from StringIO to string
        out_buf = ''.join([chr(ord(a)) for a in buffer.getvalue()])
        return out_buf



if __name__ == '__main__':
    """The entry point for the network node. Creates a network node object
    on a specified port and then lets that node take over."""

    #Parse command line arguments
    parser = argparse.ArgumentParser(description='Start the oark network node')
    parser.add_argument('port',
                        type=int,
                        default=net_consts.NET_DEFAULT_PORT,
                        help='The port to communicate commands on')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node(NODE_NAME)

    #Wait for camera node to initialise
    rospy.wait_for_service('/oark_vid_node/start_stream')

    try:
        #Create network node
        n = NetNode(NODE_NAME, args.port)

        rospy.spin()
    except Exception, e:
        rospy.logerr('Error occurred: ' + str(e))
    finally:
        rospy.loginfo('Exiting node %s...'%(NODE_NAME,))
        n.shutdown()
