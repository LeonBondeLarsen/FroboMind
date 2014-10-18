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
import rospy, smach, smach_ros, sys, threading, signal
from msgs.msg import StringStamped
from rsd_smach.ressources import gui # generated with 'pyuic4 MainWindow.ui > gui.py'
from PyQt4 import QtGui, QtCore

class MainWindow(QtGui.QMainWindow, gui.Ui_MainWindow):
	"""
		Connections between ROS interface and Qt GUI
	"""
	def __init__(self, idle_cb, job1_cb, job2_cb, parent=None):
		QtGui.QMainWindow.__init__(self, parent)
		self.setupUi(self)

		self.buttonIdle.clicked.connect(idle_cb)
		self.buttonExecuteJob1.clicked.connect(job1_cb)
		self.buttonExecuteJob2.clicked.connect(job2_cb)


class GuiNode():
	"""
		GuiNode for using FroboMind with stage. 
		Takes Odometry message from stage and published PoseStamped on /fmKnowledge/pose
	"""
	def __init__(self):
		output_topic = rospy.get_param("~task_publisher", "/fmDecisionMaking/task")

		self.pub = rospy.Publisher(output_topic, StringStamped, queue_size=10)
		self.task_msg = StringStamped()
		
		self.main_window = MainWindow(self.activateIdleState, self.activateJob1State, self.activateJob2State)
		self.main_window.show()
		
	def activateIdleState(self):
		rospy.loginfo(rospy.get_name() + ": Manual mode selected")
		self.task_msg.data = 'IDLE'
		self.pub.publish(self.task_msg)
 
	def activateJob1State(self):
		rospy.loginfo(rospy.get_name() + ": Task 1 selected")
		self.task_msg.data = 'JOB1'
		self.pub.publish(self.task_msg)
        
	def activateJob2State(self):
		rospy.loginfo(rospy.get_name() + ": Task 2 selected")
		self.task_msg.data = 'JOB2'
		self.pub.publish(self.task_msg)
			

if __name__ == '__main__':
    rospy.init_node('gui_node')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)
    node = GuiNode()
    sys.exit(app.exec_())
	
