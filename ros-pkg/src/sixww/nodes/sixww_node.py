#!/usr/bin/env python
#This file is a proof of concept ROS node to control Raymond Sheh's
#six wheel wonder. The messages should be sent to this node's
# /sixww_node/command topic. The messages should consist of a 
# string and a float32. Possible message types are:
#
#	"go" -- velocity
#	"turn_front" -- angle
#	"turn_back" -- angle
#	"elevate" -- angle
#
# I would consider this a very non-optimal way to achieve its goal.
# If anything this code exists more to test the control of the
# dynamixel ax12 servos.
#
# If this code is run, take *very* special care to make sure that either
# the min/max parameters are properly set in launch/controllers.yaml or
# the argument sent to this node is of an appropriate magnitude. Failure
# to follow this advice could break the robot.

import roslib
roslib.load_manifest( 'sixww' )

import rospy
from sixww.msg import CarCommand
import std_msgs


class CarController():

	def __init__( self, nodename ):
		self._nodename = nodename
		rospy.init_node( self._nodename )
		
		rospy.loginfo( "Started node %s" % self._nodename )
		rospy.loginfo( "Waiting for controllers to start" )
		rospy.wait_for_service( "/susp_cont/set_speed" )

		rospy.Subscriber( "/%s/command" % self._nodename, CarCommand, self._cmdReceived )

		self._controllers = {
			"front_left" : rospy.Publisher( '/wheel_fl_cont/command', std_msgs.msg.Float64 ),
			"front_right" : rospy.Publisher( '/wheel_fr_cont/command', std_msgs.msg.Float64 ),
			"center_left" : rospy.Publisher( '/wheel_cl_cont/command', std_msgs.msg.Float64 ),
			"center_right" : rospy.Publisher( '/wheel_cr_cont/command', std_msgs.msg.Float64 ),
			"back_left" : rospy.Publisher( '/wheel_bl_cont/command', std_msgs.msg.Float64 ),
			"back_right" : rospy.Publisher( '/wheel_br_cont/command', std_msgs.msg.Float64 ),

			"axel_front" : rospy.Publisher( '/axel_f_cont/command', std_msgs.msg.Float64 ),
			"axel_back" : rospy.Publisher( '/axel_b_cont/command', std_msgs.msg.Float64 ),
			"susp_cont" : rospy.Publisher( '/susp_cont/command', std_msgs.msg.Float64 )
			}


	def _cmdReceived( self, msg ):
		""" This is such a terrible way to implement this functionality
			but this is only intended as proof of concept """

		if msg.command.lower() == "go":
			rospy.loginfo( "Going with 'velocity' %f" % msg.numarg )
			self._sendMsg( 'front_left', msg.numarg )
			self._sendMsg( 'front_right', msg.numarg )
			self._sendMsg( 'back_left', msg.numarg )
			self._sendMsg( 'back_right', msg.numarg )
			self._sendMsg( 'center_left', msg.numarg )
			self._sendMsg( 'center_right', msg.numarg )
		elif msg.command.lower() == "turn_front":
			rospy.loginfo( "Turning front axle to 'angle' %f" % msg.numarg )
			self._sendMsg( 'axel_front', msg.numarg )
		elif msg.command.lower() == "turn_back":
			rospy.loginfo( "Turning back axle to 'angle' %f" % msg.numarg )
			self._sendMsg( 'axel_back', msg.numarg )
		elif msg.command.lower() == "elevate":
			rospy.loginfo( "Elevating front by 'angle' %f" % msg.numarg )
			self._sendMsg( 'susp_cont', msg.numarg )
		else:
			rospy.logerr( "Unrecognised command: %s" % msg.command.lower() )

	def _sendMsg( self, controller_name, value ):
		self._controllers[controller_name].publish( std_msgs.msg.Float64( value ) )


	def run( self ):
		rospy.spin()
		


if __name__ == '__main__':
	car = CarController( "sixww_node" )

	car.run()
