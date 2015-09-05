#!/usr/bin/env python

"""A node to receive messages from then network and coordinate
these messages/events with other running nodes.
"""

import rospy

from cmd_listener import CmdListener
from emumini2.msg import Command


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


class NetNode(object):
    def __init__(self):
        self._command_pub = rospy.Publisher('/em2_control_node/command',
                                            Command,
                                            latch=True,
                                            queue_size=10)

    def on_connect(self, addr):
        rospy.loginfo('Connected to ' + str(addr))


    def on_data(self, msg):
        rospy.loginfo('Got data')

        #Convert our own message to a ROS message
        c = Command(type=msg.get_type(), value=msg.get_value())
        self._command_pub.publish(c)


    def on_dc(self):
        rospy.loginfo('Disconnected')



if __name__ == '__main__':
    rospy.init_node('em2_net_node')

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

