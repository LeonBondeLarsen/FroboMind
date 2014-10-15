#!/usr/bin/env python
#/****************************************************************************
# FroboMind cmd_vel_converter.py
# Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#	  names of its contributors may be used to endorse or promote products
#	  derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg._PoseStamped import PoseStamped
from nav_msgs.msg._Odometry import Odometry

class Simulator():
	"""
		Simulates indoor positioning system
	"""
	def __init__(self):
		# Init node
		input_topic = rospy.get_param("~pose_sub", "/base_pose_ground_truth")
		output_topic = rospy.get_param("~pose_pub", "/fmInformation/ips")

		self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
		self.subscriber = rospy.Subscriber(input_topic, Odometry, self.onInputMessage )
		
		self.boundary_north = rospy.get_param("~boundary_north", 10.0)
		self.boundary_south = rospy.get_param("~boundary_south", -10.0)
		self.boundary_east = rospy.get_param("~boundary_east", 10.0)
		self.boundary_west = rospy.get_param("~boundary_west", -10.0)
		
	def onInputMessage(self,msg):
		if self.insideSquare(msg.pose.pose.position.x, msg.pose.pose.position.y) :
			self.publisher.publish(msg)		
		
	def insideSquare(self,x,y):
		return ( x > self.boundary_west and x < self.boundary_east and y > self.boundary_south and y < self.boundary_north )
			

if __name__ == '__main__':
	rospy.init_node('ips_simulator')
	node = Simulator()
	rospy.spin()
	
