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
import rospy, os, csv
from visualization_msgs.msg import Marker, MarkerArray
import math

class RSDVisualizer(object):
	def __init__(self):
		self.publisher = rospy.Publisher("/visualizer", MarkerArray, queue_size=10)
		self.markerArray = MarkerArray()
		
		self.waypoints = []
		with open(os.path.dirname(os.path.realpath(__file__))+'/waypoints.csv', 'rb') as wptfile:
			reader = csv.reader(wptfile, delimiter=',', quotechar='|')
			for row in reader:
				self.waypoints.append([float(row[1]),float(row[2])])


# 		self.waypoints = [
#                                      [0.2,-4.3],      # Inside box 
#                                      [0.7,-4.3],
#                                      [0.7,-5.0],
#                                      [-0.4,-5.25],
#                                      [-0.54239938613,-4.87560666965], #dispenser
#                                      [-0.2,-4.3],
#                                      [-0.2,-2.0],       # Box out
#                                      [-1.70,0.70],       # Line crosses
#                                      [-2.73,0.72],
#                                      [-2.75,1.72],
#                                      [-1.51,1.76],
#                                      [-0.95,1.78],
#                                      [1.25,1.82],
#                                      [1.93,1.82],
#                                      
#                                      [0.25,0.0],        # Step...
#                                      [0.25,-1.5]        # Box in
#                                  ]
		
	def getMarker(self,x,y):
		marker = Marker()
		marker.header.frame_id = "world"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = 0
		
		return marker
		
	def spin(self):	
		for x,y in self.waypoints :
			marker = self.getMarker(x,y)
			self.markerArray.markers.append(marker)
		
		id = 0
		for m in self.markerArray.markers:
			m.id = id
   			id += 1      
   			
		while not rospy.is_shutdown(): 	
		   	self.publisher.publish(self.markerArray)
		   	rospy.sleep(0.1)

if __name__ == '__main__':
	rospy.init_node('register')
	node = RSDVisualizer()
	node.spin()
		
