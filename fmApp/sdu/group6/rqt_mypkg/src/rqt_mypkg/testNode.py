#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class TestNode():
    def __init__(self):
        rospy.loginfo("Test Node Started")
        rospy.init_node('testNode', anonymous=False)
        self.testDataTopic = rospy.get_param("~testData","/fmTest/testData")
        self.pub = rospy.Publisher(self.testDataTopic,String)
        
        while not rospy.is_shutdown():
            self.pub.publish("Testing")
            rospy.sleep(60)

if __name__ == '__main__':
#     try:
        TestNode()
#     except:
#         rospy.loginfo("Node Terminated")