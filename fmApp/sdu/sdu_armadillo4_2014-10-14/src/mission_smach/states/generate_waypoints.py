import rospy
import smach
import smach_ros
import tf
import numpy as np
from geometry_msgs.msg._Quaternion import Quaternion
from nav_msgs.msg._Odometry import Odometry

class generate(smach.State):
    """
        Temporary implementation. State giving the next position goal based on hard-coded list
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['waypoints'])
        self.ptr = 0
        self.position_list = [] #[[3,0],[3,-3],[0,-3],[0,0]]
        
        self.odom_topic = rospy.get_param("~odom_topic",'/fmKnowledge/odometry')
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.onOdometry )
        self.latest_position = np.empty((2, ), dtype=np.float64)
        self.latest_orientation = np.empty((4, ), dtype=np.float64)

    def execute(self, userdata):
        userdata.waypoints = self.generateWaypointList()
        return 'succeeded'       

     
    def onOdometry(self,msg):
        """    
            Callback method to save current position
        """
        
        # Extract the orientation quaternion
        self.latest_orientation[0] = msg.pose.pose.orientation.x
        self.latest_orientation[1] = msg.pose.pose.orientation.y
        self.latest_orientation[2] = msg.pose.pose.orientation.z
        self.latest_orientation[3] = msg.pose.pose.orientation.w
        
        # Extract the position vector
        self.latest_position[0] = msg.pose.pose.position.x
        self.latest_position[1] = msg.pose.pose.position.y
        
    def generateWaypointList(self):
        rospy.loginfo(rospy.get_name() + "Generating waypoint")
        rospy.loginfo(rospy.get_name() + "  Current position is (%f,%f) ",self.latest_position[0], self.latest_position[1])
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(self.latest_orientation)
        rospy.loginfo(rospy.get_name() + "  Yaw is %f ",yaw)
        point_x = self.latest_position[0] + 2.0*np.cos(yaw)
        point_y = self.latest_position[1] + 2.0*np.sin(yaw)
        rospy.loginfo(rospy.get_name() + "  Waypoint is (%f,%f) ",point_x,point_y)
        return [[point_x,point_y]]

    
    
    