#/****************************************************************************
# FroboMind wii_interface.py
# Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
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
import math
from msgs.msg import StringStamped, BoolStamped
from sensor_msgs.msg import Joy,JoyFeedback,JoyFeedbackArray
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
import smach
import smach_ros

class GamepadInterface():
    """
        Wiimote interface.
        Button RB is deadman button
        Button B enters automode
        Button X exits automode
        When not in automode, remote controlled driving is active.
    """
    def __init__(self):
        # Setup parameters
        rospy.loginfo("gamepad interface initialized")
        self.automode = False
        self.deadman = Bool(False)
        self.linear = 0.0
        self.angular = 0.0
        self.twist = TwistStamped()

	self.JoyLeftFR  = 0
	self.JoyLeftRL  = 0

	self.JoyRightFR = 0
	self.JoyRightRL = 0

	self.scale = 0
        # Callbacks
        self.button_A_cb = self.no_callback_registered
        self.button_up_cb = self.no_callback_registered
        self.button_down_cb = self.no_callback_registered

        # Get parameters
        self.max_linear_velocity = rospy.get_param("~max_linear_velocity",2)
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity",4)
        self.publish_frequency = rospy.get_param("~publish_frequency",10) 
	self.scale = rospy.get_param("~scale",10)          
        
        # Get topic names
        self.deadman_topic = rospy.get_param("~deadman_topic",'deadman')
        self.automode_topic = rospy.get_param("~automode_topic",'automode')
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic",'cmd_vel')
        self.feedback_topic = rospy.get_param("~feedback_topic",'joy/set_feedback') 
        self.joy_topic = rospy.get_param("~joy_topic",'/joy')
        self.status_topic = rospy.get_param("~all_ok_topic",'/fmSafety/all_ok')     

        # Setup topics
        self.deadman_pub = rospy.Publisher(self.deadman_topic, Bool)
        self.automode_pub = rospy.Publisher(self.automode_topic, Bool)
        self.twist_pub = rospy.Publisher(self.cmd_vel_topic, TwistStamped)
        self.fb_pub = rospy.Publisher(self.feedback_topic, JoyFeedbackArray)
        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self.onJoy )
        self.status_sub = rospy.Subscriber(self.status_topic, BoolStamped , self.onAllOk)

    def no_callback_registered(self):
        rospy.loginfo("Button pressed but no corresponding callback was registered")
        
    def register_callback_button_A(self,cb):
        rospy.loginfo("Callback registered on button A")
        self.button_A_cb = cb
        
    def register_callback_button_up(self,cb):
        rospy.loginfo("Callback registered on button Up")
        self.button_up_cb = cb
        
    def register_callback_button_down(self,cb):
        rospy.loginfo("Callback registered on button Down")
        self.button_down_cb = cb
        
    def onJoy(self,msg):
        """
            Callback method handling Joy messages
            Index   Button   Usage
            0       1        Enter automode
            1       2        Exit automode
            2       A        Callback
            3       B        Deadman
            4       Plus
            5       Minus
            6       Left
            7       Right
            8       Up
            9       Down
            10      HOME
        """
        # Handle automode buttons
        if msg.buttons[1] == 1 :
            self.automode = True
        if msg.buttons[2] == 1 :
            self.automode = False
        
        # Handle button callback
#        if msg.buttons[2] == 1 :
#            self.button_A_cb()
#        if msg.buttons[8] == 1 :
#            self.button_up_cb()
#        if msg.buttons[9] == 1 :
#            self.button_down_cb()
        # Handle deadman button
        self.deadman = msg.buttons[5]
	print "deadman: ", msg.buttons[5]
	# Handle joys
	self.JoyLeftFR  = msg.axes[1]
	self.JoyLeftRL  = msg.axes[0]

	self.JoyRightFR = msg.axes[4]
	self.JoyRightRL = msg.axes[3]

    def onAllOk(self,msg):
        """
            Callback method to translate All OK signal into user feedback
        """
        if msg.data :
            self.warning = False
        else:
            self.warning = True
                        
    def publishCmdVel(self): 
        """
            Method to average and publish twist from wiimote input
        """
	self.linear  = self.JoyLeftFR
	self.angular = self.JoyLeftRL + self.JoyRightRL

        # Publish twist message
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = self.linear
        self.twist.twist.angular.z = self.angular
        self.twist_pub.publish(self.twist)
      
    def publishZeroCmdVel(self):
        # Publish zero twist message
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = 0
        self.twist.twist.angular.z = 0
        self.twist_pub.publish(self.twist)
          
    def publishDeadman(self):
        """
            Method to publish Bool message on deadman topic
        """
        self.deadman_pub.publish(self.deadman)
        
    def publishAutomode(self):
        """
            Method to publish Bool message on automode topic
        """
        self.automode_pub.publish(self.automode)
