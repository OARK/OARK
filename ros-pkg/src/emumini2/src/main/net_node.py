#!/usr/bin/env python

"""A node to receive messages from then network and coordinate
these messages/events with other running nodes.
"""

import rospy

from net import CmdListener


__author__    = 'Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']


def on_connect(addr):
    rospy.loginfo('Connected to ' + addr)


def on_data(msg):
    rospy.loginfo('Got data')


def on_dc():
    rospy.loginfo('Disconnected')


if __name__ == '__main__':
    rospy.init_node('em2_net_node')

    #Create a listener on all interfaces
    listener = CmdListener('')
    try:
        listener.add_connect_listener( )
        listener.add_data_listener( )
        listener.add_dc_listener( )

        rospy.spin()
    except Exception, e:
        rospy.logerror('Error occurred: ' + str(e))
    finally:
        listener.shutdown()

