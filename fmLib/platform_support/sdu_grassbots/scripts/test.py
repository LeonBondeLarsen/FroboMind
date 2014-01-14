#!/usr/bin/env python
#/****************************************************************************
# FroboMind init_serial_nmea.py
# Copyright (c) 2011-2013, Leon Bonde Larsen <leon@bondelarsen.dk>
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
"""
    This script is for testing the Pichi system without a roboteq controller present.
"""
import rospy
import math
from msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

class TestInterface():
    def __init__(self):
        # Init node and setup topics
        rospy.init_node('test_interface')
        self.publisher = rospy.Publisher("/fmData/robot_rx", serial)
        
        self.enc_left_pub = rospy.Publisher("/fmInformation/enc_left",encoder)
        self.enc_right_pub = rospy.Publisher("/fmInformation/enc_right",encoder)
        self.deadman_pub = rospy.Publisher("/fmSignals/deadman",Bool)
        self.vel_publ = rospy.Publisher("/fmSignals/cmd_vel_left",TwistStamped)
        self.vel_pubr = rospy.Publisher("/fmSignals/cmd_vel_right",TwistStamped)
        
        self.vel_msg = TwistStamped()
        self.vel_msg.twist.linear.x = 0.1
        
        self.tx_sub = rospy.Subscriber("/fmData/robot_tx", serial, self.onTx)
        self.rx_sub = rospy.Subscriber("/fmData/robot_rx", serial, self.onRx)
        self.msg = serial()
        self.flag = False
        self.enc = 0.0
        
        self.dead = Bool()
        self.dead.data = True
        
        self.enc_msg = encoder()
        # Spin
        try:
            while not rospy.is_shutdown():
                self.msg.header.stamp = rospy.Time.now()
#                self.msg.data = "CB=125:125"
                self.msg.data = "CB=" + str(int(self.enc)) + ":" + str(-int(self.enc))
                self.enc += 25.7
                self.enc_msg.encoderticks = int(self.enc)
                self.publisher.publish(self.msg)
                self.enc_left_pub.publish(self.enc_msg)
                self.enc_right_pub.publish(self.enc_msg)
                self.deadman_pub.publish(self.dead)
                
                self.vel_msg.header.stamp = rospy.Time.now()
                self.vel_publ.publish(self.vel_msg)
                self.vel_pubr.publish(self.vel_msg)
                
                rospy.sleep(0.02)
        except rospy.ROSInterruptException:
            pass
    
    def onRx(self,msg):
        print(msg.data)
        return 0
            
    def onTx(self,msg):
        print(msg.data)
        if not self.flag and "?FID" in msg.data :
            self.flag = True
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "FID=Roboteq blah blah"
            self.publisher.publish(self.msg)
        elif "?V" in msg.data :
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "V=412:412:5000"
            self.publisher.publish(self.msg)
        elif "?BA" in msg.data :
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "BA=300:300"
            self.publisher.publish(self.msg)
        elif "?A" in msg.data :       
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "A=412:412"
            self.publisher.publish(self.msg)
        elif "?T" in msg.data :        
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "T=85:10:15"
            self.publisher.publish(self.msg)
        elif "?FF" in msg.data :        
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "FF=1"
            self.publisher.publish(self.msg)
        elif "?FS" in msg.data :        
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = "FS=2"
            self.publisher.publish(self.msg)

if __name__ == '__main__':
    node = TestInterface()
    rospy.spin()
    



    