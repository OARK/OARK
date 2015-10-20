#!/usr/bin/env python

"""A node to receive messages from then network and coordinate
these messages/events with other running nodes.
"""

import rospy

from cmd_listener import CmdListener

from emumini2.msg import Command
from emumini2.srv import StartStream
from std_srvs.srv import Empty


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


#Defines the mapping between message type numbers and the ros message
#that they instantiate. This is needed so that the message type can be
#sent over the network.
MSGS = {
        1: Command,
       }



class NetNode(object):
    def __init__(self):
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
        ros_msg = MSGS[msg_type]()
        ros_msg.deserialize(msg)

        msg_name = MSGS[msg_type].__name__

        try:
            msg_func = getattr(self, 'on_' + msg_name.lower())
            msg_func(ros_msg)
        except AttributeError, ae:
            rospy.logwarn('No function to handle ROS msg <' + msg_name + '>')


    def on_command(self, cmd):
        self._command_pub.publish(cmd)


    def on_dc(self):
        rospy.loginfo('Disconnected')
        stop_stream = rospy.ServiceProxy('/em2_vid_node/stop_stream',
                                         Empty)

        stop_stream()



if __name__ == '__main__':
    rospy.init_node('em2_net_node')

    #Wait for camera node to initialise
    rospy.wait_for_service('/em2_vid_node/start_stream')

    #Create a listener on all interfaces
    listener = CmdListener('')
    n = NetNode()
    try:
        listener.add_connect_listener(n.on_connect)
        listener.add_data_listener(n.on_data)
        listener.add_dc_listener(n.on_dc)

        rospy.spin()
    except Exception, e:
        rospy.logerr('Error occurred: ' + str(e))
    finally:
        rospy.loginfo('Exiting node em2_net_node...')
        listener.shutdown()

