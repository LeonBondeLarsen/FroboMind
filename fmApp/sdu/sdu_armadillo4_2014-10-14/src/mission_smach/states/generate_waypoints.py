import rospy
import smach
import smach_ros
import tf
import numpy as np
from tf import TransformListener
from simple_2d_math.vector import Vector
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
        
        self.base_frame = rospy.get_param("~base_frame","base")
        self.odom_frame = rospy.get_param("~odom_frame","world")
        
        self.position = Vector(0,0)
        self.heading = Vector(0,0)
        
        self.__listen = TransformListener()
#        self.odometry_topic = rospy.get_param("~odometry_topic","/fmKnowledge/odom")
#        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.onOdometry )
#        self.latest_position = np.empty((2, ), dtype=np.float64)
#        self.latest_orientation = np.empty((4, ), dtype=np.float64)

    def execute(self, userdata):
        userdata.waypoints = self.generateWaypointList()
        return 'succeeded'       

     
#     def onOdometry(self,msg):
#         """    
#             Callback method to save current position
#         """
#         
#         # Extract the orientation quaternion
#         self.latest_orientation[0] = msg.pose.pose.orientation.x
#         self.latest_orientation[1] = msg.pose.pose.orientation.y
#         self.latest_orientation[2] = msg.pose.pose.orientation.z
#         self.latest_orientation[3] = msg.pose.pose.orientation.w
#         
#         # Extract the position vector
#         self.latest_position[0] = msg.pose.pose.position.x
#         self.latest_position[1] = msg.pose.pose.position.y
        
    def generateWaypointList(self):
        (pos_x, pos_y, yaw) = self.get_current_position()
        point_x = pos_x + ( 1.0*np.cos(yaw) )
        point_y = pos_y + ( 1.0*np.sin(yaw) )
        
        rospy.loginfo(rospy.get_name() + " Generating waypoint: ")
        rospy.loginfo(rospy.get_name() + "   Current position is (%f,%f) ",pos_x, pos_y)
        rospy.loginfo(rospy.get_name() + "   Heading is %f ",yaw)
        rospy.loginfo(rospy.get_name() + "   Waypoint is (%f,%f) ",point_x,point_y)
        
        return [[point_x,point_y]]

    def get_current_position(self):
        """
            Get current position from tf
        """
        try:
            (position,heading) = self.__listen.lookupTransform( self.odom_frame,self.base_frame,rospy.Time(0)) # The transform is returned as position (x,y,z) and an orientation quaternion (x,y,z,w).
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(heading)
            return (position[0], position[1], yaw)
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo(rospy.get_name() + " : could not locate vehicle "+str(err))
            return (0,0,0)       
    
    