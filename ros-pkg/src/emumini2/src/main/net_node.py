#!/usr/bin/env python

"""A node to receive messages from the network and coordinate
these messages/events with other running nodes.
"""

from cmd_listener import CmdListener
import net_consts

import rospy
import StringIO

from emumini2.msg import Command
from emumini2.srv import StartStream
from emumini2.srv import InputRequest, InputRequestRequest, InputRequestResponse
from std_srvs.srv import Empty


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


NODE_NAME='em2_net_node'


class NetNode(object):
    def __init__(self, node_name):
        #Create network communication object
        self.listener = CmdListener('')

        #Add listeners
        self.listener.add_connect_listener(self.on_connect)
        self.listener.add_data_listener(self.on_data)
        self.listener.add_dc_listener(self.on_dc)

        self.node_name = node_name
        self._command_pub = rospy.Publisher('/em2_control_node/command',
                                            Command,
                                            latch=True,
                                            queue_size=10)


    def on_connect(self, addr):
        rospy.loginfo('Connected to ' + str(addr))
        start_stream = rospy.ServiceProxy('/em2_vid_node/start_stream',
                                          StartStream)

        #The first member of the addr tuple is the IP address
        start_stream(str(addr[0]))



    def on_data(self, msg_type, msg):
        """Receives a message from the caller, turns it into the appropriate
        ROS message and then forwards it to another function to handle the message.
        The calling of the function may need to be run in another thread at a later
        point if efficiency issues are encountered.
        """

        rospy.loginfo('Got data')

        print 'Packet', [ord(a) for a in msg]
        ros_msg = net_consts.MSGS[msg_type]()
        ros_msg.deserialize(msg)

        msg_name = net_consts.MSGS[msg_type].__name__

        #try:
        msg_func = getattr(self, 'on_' + msg_name.lower())
        msg_func(ros_msg)
        #except AttributeError, ae:
            #rospy.logwarn('No function to handle ROS msg <' + msg_name + '>')


    def on_command(self, cmd):
        self._command_pub.publish(cmd)


    def on_inputrequestrequest(self, inputrequest):
        inputrequest_srv = rospy.ServiceProxy('/em2_control_node/get_inputs',
                                              InputRequest)

        result = inputrequest_srv(inputrequest)
        print 'Result of input request: '
        print str(result)
        self.listener.send(net_consts.MSG_TYPES[InputRequestResponse], self._msg_to_string(result))


    def on_dc(self):
        rospy.loginfo('Disconnected')
        stop_stream = rospy.ServiceProxy('/em2_vid_node/stop_stream',
                                         Empty)

        stop_stream()

    def _msg_to_string(self, msg):
        """Takes a ROS message and converts it to a serialised string. Essentially
        just wraps msg.serialize()
        """
        buffer = StringIO.StringIO()
        msg.serialize(buffer)
        #Serialise from StringIO to string
        out_buf = ''.join([chr(ord(a)) for a in buffer.getvalue()])
        return out_buf



if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    #Wait for camera node to initialise
    rospy.wait_for_service('/em2_vid_node/start_stream')

    #Create a listener on all interfaces
    #try:
    n = NetNode(NODE_NAME)

    rospy.spin()
    #except Exception, e:
        #rospy.logerr('Error occurred: ' + str(e))
    #finally:
        #rospy.loginfo('Exiting node %s...'%(NODE_NAME,))
        #listener.shutdown()

