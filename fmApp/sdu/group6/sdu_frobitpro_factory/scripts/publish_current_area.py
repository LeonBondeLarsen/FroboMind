#!/usr/bin/env python

import rospy
from msgs.msg import StringStamped
from tf import TransformListener

areas = {
    'FLOOR_OUT': [-2.00,-2.15,   -0.15, 1.00],
    'FLOOR_IN':  [-0.15,-2.15,    2.50, 1.00],
    'RAMP_OUT':  [-1.00,-4.70,   -0.15,-2.15],
    'RAMP_IN':   [-0.15,-4.10,    0.40,-2.15],
    'IN_BOX':    [-0.15,-5.80,    0.35,-4.10],
    'DISPENSER': [-1.00,-5.80,   -0.15,-4.70],
    'STATION_1': [ 0.35,-4.65,    1.15,-4.10],
    'STATION_2': [ 0.35,-5.20,    1.15,-4.65],
    'STATION_3': [ 0.35,-5.80,    1.15,-5.20],
    'LINE_LEFT': [-4.00, 0.00,   -2.00, 1.95],
    'LINE_MID':  [-2.00, 1.00,    2.50, 1.95],
    'LINE_RIGHT':[ 2.50, -1.00,    6.50, 1.95],
    'LOAD_OFF1': [ 3.40, 1.95,    4.00, 3.00],
    'LOAD_ON1':  [ 4.00, 1.95,    4.60, 3.00],
    'LOAD_OFF2': [ 0.75, 1.95,    1.35, 3.00],
    'LOAD_ON2':  [ 1.35, 1.95,    1.95, 3.00],
    'LOAD_OFF3': [-2.20, 1.95,   -1.60, 3.00],
    'LOAD_ON3':  [-1.60, 1.95,   -1.00, 3.00]
}


class AreaPublisher():
    def __init__(self):
        rospy.init_node('rsd_area_node')
        rospy.loginfo(rospy.get_name() + ": Area Evaluater Initialised")
        
        self.position_topic = rospy.get_param("~rsd_area_topic", '/fmKnowledge/rsd_area')
        self.position_pub = rospy.Publisher(self.position_topic, StringStamped, queue_size=10)
        
        self.listener = TransformListener()
        self.odom_frame = rospy.get_param("~odom_frame", "world")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.rsd_area = StringStamped()
        self.rsd_area.data = 'UNKNOWN'
        rospy.Timer(rospy.Duration(2.0), self.update_area)
        rospy.loginfo("ending initialisation")

    def update_area(self, event):
        #rospy.loginfo("updating area")
        (self.position, self.heading) = self.listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        x = self.position[0]
        y = self.position[1]
        
        for area, sides in areas.iteritems():
            self.rsd_area.data = 'UNKNOWN'
            #rospy.loginfo("checking with sides: " + str(sides[0]) + " " + str(sides[1]) + " " + str(sides[2]) + " " + str(sides[3]))
            if sides[0] < x and x < sides[2] and sides[1] < y and y < sides[3]:
                self.rsd_area.data = area
                if area == 'LINE_LEFT' or area == 'LINE_RIGHT' or area == 'LINE_MID':
                    self.rsd_area.data = 'LINE'
                break
        rospy.loginfo("updating area: " + str(x) + " " + str(y) + " is " + self.rsd_area.data)
        
        self.position_pub.publish(self.rsd_area)


if __name__ == '__main__':
    node = AreaPublisher()
    rospy.spin()
