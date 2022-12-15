#!/usr/bin/env python3

#==============================================================================
# File name          : mp0.py                                                                 
# Description        : MP0 for CS588                                                                                                                        
# Usage              : rosrun mp0 mp0.py                                                                                                                           
#==============================================================================
from __future__ import print_function

#Python Headers
import math
import os

# ROS Headers
import rospy

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt

class Node():

	def __init__(self):

		self.rate = rospy.Rate(10)
		
		self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size = 1)
		self.turn_cmd = PacmodCmd()
		self.turn_cmd.ui16_cmd = 1

		# self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size = 1)
		# self.steer_cmd = PositionWithSpeed()
		# self.steer_cmd.angular_position = 0.75 # radians, -: clockwise, +: counter-clockwise
		# self.steer_cmd.angular_velocity_limit = 2.0 # radians / second

	def run(self):

		sos = [2, 2, 1, 0, 0, 1, 2, 2, 1, 1, 1]

		while not rospy.is_shutdown():

			# self.steer_pub.publish(self.steer_cmd)
			for signal in sos:
				self.turn_cmd.ui16_cmd = signal
				self.turn_pub.publish(self.turn_cmd)
				self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('sos_node', anonymous=True)
	node = Node()
	node.run()
