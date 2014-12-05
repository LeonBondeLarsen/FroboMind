#!/usr/bin/env python
import pickle, rospy, tf, cv2
import numpy as np
from line_follower_transformed_image import line_and_cross_detector
from nav_msgs.msg import Odometry


class LineDetector(object):
    def __init__(self):
        # Init line detector
        self.camera = cv2.VideoCapture(1)
        self.f = file("/home/leon/code/roswork/build/fmApp/sdu/group6/line_detector/catkin_generated/installspace/transformMatrix", 'rb')
        self.transformMatrix = pickle.load(self.f)
        
        destination = np.array([(100,500),(100,100),(500,100),(500,500)],np.float32)
        source = np.array([(31,468),(105,28),(506,21),(590,454)],np.float32)
        self.transformMatrix = cv2.getPerspectiveTransform(source,destination)
        
        self.odometry_msg = Odometry()
        self.listener = tf.TransformListener() 
        
        # Init node
        output_topic = rospy.get_param("~pose_pub", "/fmInformation/line_pose")
        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
        rospy.Timer(rospy.Duration(0.005), self.onCameraTimer)
        rospy.Timer(rospy.Duration(1), self.onPublishTimer)
        
        # Init TF listener
        self.tf_listener = tf.TransformListener()
        self.quaternion = np.empty((4, ), dtype=np.float64)
                       
    def onCameraTimer(self,event):
        ret, image = self.camera.read()
        image = cv2.warpPerspective(image,self.transformMatrix,(600,600))
        detector = line_and_cross_detector(1,1)
        self.value = detector.analyze_image(image)      
        self.updateOdometryMessage(self.value)

    def onPublishTimer(self,event):
        self.publisher.publish(self.odometry_msg)

    def getNearestPoint(self, x, y):
        #TODO: 
        return (0,0.95)
        
    def updateOdometryMessage(self,value): 
        self.odometry_msg.header.stamp = rospy.Time.now()
        if self.value is not None:
            # Get estimated robot position
            (tf_x,tf_y,yaw) = self.get_current_position()
            
            # Unpack state
            (x, y), theta1, theta2 = value
            sensed_x = (x*4 -300.)/1000. # [m]
            sensed_y = (300 - y*4) /1000. + 0.5  # [m]
            
            # Discern correct angle
            theta = theta1 #TODO: implement for real
            
            # Move point to robot frame
            point_in_robot_frame_x = sensed_x + np.sqrt(0.5**2)*cos(theta)
            point_in_robot_frame_y = sensed_y + np.sqrt(0.5**2)*sin(theta)
            
            # Estimate position of sensed point in world frame
            point_in_world_frame_x = tf_x + point_in_robot_frame_x
            point_in_world_frame_y = tf_y + point_in_robot_frame_y
            
            # Get nearest real point
            (known_x, known_y) = self.getNearestPoint(point_in_world_frame_x, point_in_world_frame_y)
                    
            # Calculate robot position
            robot_x = known_x - point_in_robot_frame_x
            robot_y = known_y - point_in_robot_frame_y
            
            # Update odometry message
            self.odometry_msg.pose.pose.position.x = robot_x
            self.odometry_msg.pose.pose.position.y = robot_y
            self.odometry_msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, theta)

            # Set covariance to good quality
            self.odometry_msg.pose.covariance[0] = 0.00001 # variance x
            self.odometry_msg.pose.covariance[7] = 0.00001 # variance y
            self.odometry_msg.pose.covariance[35] = 0.00001 # variance theta
        
        else:
            # Set covariance to poor quality
            self.odometry_msg.pose.covariance[0] = 1 # variance x
            self.odometry_msg.pose.covariance[7] = 1 # variance y
            self.odometry_msg.pose.covariance[35] = 1 # variance theta

    def get_current_state(self):
        try:
            (position,heading) = self.listener.lookupTransform( self.line_frame, self.base_frame, rospy.Time(0) )
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(heading)
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo(rospy.get_name() + " : could not locate vehicle "+str(err))
        return (position[0],position[1],yaw)
                    
if __name__ == '__main__':
    rospy.init_node('line_detector')
    node = LineDetector()
    rospy.spin()