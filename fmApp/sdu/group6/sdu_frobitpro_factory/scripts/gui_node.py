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
	def __init__(self, manual_mode_cb,command_wait_cb,command_abort_cb,command_tip_cb,command_navigate_dispenser_cb,command_navigate_in_box_cb,
				command_navigate_line_cb,command_navigate_station_1_cb,command_navigate_station_2_cb,command_navigate_station_3_cb,
				command_navigate_ramp_out_cb,command_navigate_ramp_in_cb,command_navigate_floor_out_cb,command_navigate_floor_in_cb,
				command_navigate_load_on_1_cb,command_navigate_load_off_1_cb,command_navigate_load_on_2_cb,command_navigate_load_off_2_cb,
				command_navigate_load_off_3_cb,command_navigate_load_on_3_cb, parent=None):
		QtGui.QMainWindow.__init__(self, parent)
		self.setupUi(self)
		
		self.robotStateFeedback.setText('Receiving robot state...')
		
		self.manualMode.clicked.connect(manual_mode_cb)
		self.commandWait.clicked.connect(command_wait_cb)
		self.commandAbort.clicked.connect(command_abort_cb)
		self.commandTip.clicked.connect(command_tip_cb)

		self.commandNavigateDispenser.clicked.connect(command_navigate_dispenser_cb)

		self.commandNavigateInBox.clicked.connect(command_navigate_in_box_cb)
		self.commandNavigateLine.clicked.connect(command_navigate_line_cb)
		
		self.commandNavigateStation1.clicked.connect(command_navigate_station_1_cb)
		self.commandNavigateStation2.clicked.connect(command_navigate_station_2_cb)
		self.commandNavigateStation3.clicked.connect(command_navigate_station_3_cb)

		self.commandNavigateRampOut.clicked.connect(command_navigate_ramp_out_cb)
		self.commandNavigateRampIn.clicked.connect(command_navigate_ramp_in_cb)

		self.commandNavigateFloorOut.clicked.connect(command_navigate_floor_out_cb)
		self.commandNavigateFloorIn.clicked.connect(command_navigate_floor_in_cb)

		self.commandNavigateLoadOn1.clicked.connect(command_navigate_load_on_1_cb)
		self.commandNavigateLoadOff1.clicked.connect(command_navigate_load_off_1_cb)

		self.commandNavigateLoadOn2.clicked.connect(command_navigate_load_on_2_cb)
		self.commandNavigateLoadOff2.clicked.connect(command_navigate_load_off_2_cb)

		self.commandNavigateLoadOff3.clicked.connect(command_navigate_load_off_3_cb)
		self.commandNavigateLoadOn3.clicked.connect(command_navigate_load_on_3_cb)		
		
		

class GuiNode():
	"""
		GuiNode for using FroboMind with stage. 
		Takes Odometry message from stage and published PoseStamped on /fmKnowledge/pose
	"""
	def __init__(self):
		output_topic = rospy.get_param("~task_publisher", "/fmDecisionMaking/task")
		input_topic = rospy.get_param("~state_subscriber", "/fmInformation/state")

		self.pub = rospy.Publisher(output_topic, StringStamped, queue_size=10)
		self.task_msg = StringStamped()
		
		self.sub = rospy.Subscriber(input_topic, StringStamped, self.onStateFeedback)
		
		self.main_window = MainWindow(
			self.activateManualModeState ,self.activateWaitState ,self.activateAbortState ,self.activateTipState ,self.activateNavigateDispenserState ,
			self.activateNavigateInBoxState ,self.activateNavigateLineState ,self.activateNavigateStation1State ,self.activateNavigateStation2State ,
			self.activateNavigateStation3State ,self.activateNavigateRampOutState ,self.activateNavigateRampInState ,self.activateNavigateFloorOutState ,
			self.activateNavigateFloorInState ,self.activateNavigateLoadOn1State ,self.activateNavigateLoadOff1State ,self.activateNavigateLoadOn2State ,
			self.activateNavigateLoadOff2State ,self.activateNavigateLoadOff3State ,self.activateNavigateLoadOn3State) 

		self.main_window.show()
		
	def activateManualModeState(self):
		rospy.loginfo(rospy.get_name() + ": Manual mode activated")
		self.task_msg.data = 'MANUAL'
		self.pub.publish(self.task_msg)
 
	def activateWaitState(self):
		rospy.loginfo(rospy.get_name() + ": Wait state activated")
		self.task_msg.data = 'WAIT'
		self.pub.publish(self.task_msg)
        
	def activateAbortState(self):
		rospy.loginfo(rospy.get_name() + ": Abort state activated")
		self.task_msg.data = 'ABORT'
		self.pub.publish(self.task_msg)
		
	def activateTipState(self):
		rospy.loginfo(rospy.get_name() + ": Tip state activated")
		self.task_msg.data = 'TIP'
		self.pub.publish(self.task_msg)
		
	def activateNavigateDispenserState(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to dispenseer")
		self.task_msg.data = 'NAVIGATE_DISPENSER'
		self.pub.publish(self.task_msg)

	def activateNavigateInBoxState(self):
		rospy.loginfo(rospy.get_name() + ": Navigate in box")
		self.task_msg.data = 'NAVIGATE_IN_BOX'
		self.pub.publish(self.task_msg)
		
	def activateNavigateLineState(self):
		rospy.loginfo(rospy.get_name() + ": Navigate line")
		self.task_msg.data = 'NAVIGATE_LINE'
		self.pub.publish(self.task_msg)

	def activateNavigateStation1State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to station 1")
		self.task_msg.data = 'NAVIGATE_STATION_1'
		self.pub.publish(self.task_msg)

	def activateNavigateStation2State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to station 2")
		self.task_msg.data = 'NAVIGATE_STATION_2'
		self.pub.publish(self.task_msg)

	def activateNavigateStation3State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to station 3")
		self.task_msg.data = 'NAVIGATE_STATION_3'
		self.pub.publish(self.task_msg)
		
	def activateNavigateRampInState(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to ramp in")
		self.task_msg.data = 'NAVIGATE_RAMP_IN'
		self.pub.publish(self.task_msg)

	def activateNavigateRampOutState(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to ramp out")
		self.task_msg.data = 'NAVIGATE_RAMP_OUT'
		self.pub.publish(self.task_msg)
		
	def activateNavigateFloorInState(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to floor in")
		self.task_msg.data = 'NAVIGATE_FLOOR_IN'
		self.pub.publish(self.task_msg)

	def activateNavigateFloorOutState(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to floor out")
		self.task_msg.data = 'NAVIGATE_FLOOR_OUT'
		self.pub.publish(self.task_msg)
			
	def activateNavigateLoadOn1State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to load on 1")
		self.task_msg.data = 'NAVIGATE_LOAD_ON_1'
		self.pub.publish(self.task_msg)

	def activateNavigateLoadOff1State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to load off 1")
		self.task_msg.data = 'NAVIGATE_LOAD_OFF_1'
		self.pub.publish(self.task_msg)

	def activateNavigateLoadOn2State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to load on 2")
		self.task_msg.data = 'NAVIGATE_LOAD_ON_2'
		self.pub.publish(self.task_msg)

	def activateNavigateLoadOff2State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to load off 2")
		self.task_msg.data = 'NAVIGATE_LOAD_OFF_2'
		self.pub.publish(self.task_msg)
		
	def activateNavigateLoadOn3State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to load on 3")
		self.task_msg.data = 'NAVIGATE_LOAD_ON_3'
		self.pub.publish(self.task_msg)

	def activateNavigateLoadOff3State(self):
		rospy.loginfo(rospy.get_name() + ": Navigate to load off 3")
		self.task_msg.data = 'NAVIGATE_LOAD_OFF_3'
		self.pub.publish(self.task_msg)
		
	def onStateFeedback(self, msg):
		self.main_window.robotStateFeedback.setText( msg.data )
		
if __name__ == '__main__':
    rospy.init_node('gui_node')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)
    node = GuiNode()
    sys.exit(app.exec_())
	
