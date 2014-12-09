# -*- coding: utf-8 -*-
"""
Created on Fri Dec  5 17:58:25 2014

@author: frederik
"""

#!/usr/bin/env python
import rospy, tf, cv2
import numpy as np
from line_follower_transformed_image import line_and_cross_detector
from nav_msgs.msg import Odometry


class LineDetector(object):
    def __init__(self):
        # Init line detector
        self.camera = cv2.VideoCapture(0)
#        self.f = file("/home/leon/code/roswork/build/fmApp/sdu/group6/line_detector/catkin_generated/installspace/transformMatrix", 'rb')
#        self.transformMatrix = pickle.load(self.f)
        
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

    def getNearestPoint(self, x, y, theta):
        def find_nearest_cross(crosses, robot_pose):
        
            def distance(cross, camera_point):
                return np.sqrt( (cross[0] - camera_point[0])**2 + (cross[1] - camera_point[1])**2)
        
            def create_lines(cross):
                list_of_lines = []
                for angle in [0, np.pi/2, np.pi, np.pi*1.5]:  
                    list_of_lines.append( (cross[0]  + np.cos(angle)/10, cross[1] + np.sin(angle)/10 ) )
                return list_of_lines
        
        
            robot_angle = np.pi * ( robot_pose[1] + 90 ) /180
            camera_point = (robot_pose[0][0] - np.cos(robot_angle)/2, robot_pose[0][1] + np.sin(robot_angle)/2 )
#            print camera_point    
            dist_index = 0
            for index, cross in enumerate( crosses ):
                if distance(cross, camera_point) < distance(crosses[dist_index],camera_point):
                    dist_index = index
            
#            print distance(crosses[dist_index], camera_point)
#            print crosses[dist_index]
            possible_lines = create_lines(crosses[dist_index])
#            print possible_lines
        
            line_index = 0
            for index, line in enumerate( possible_lines ):
                if distance(line, robot_pose[0]) < distance(possible_lines[line_index],robot_pose[0]):
                    line_index = index
                    
#            print line_index
            return crosses[dist_index], [0, np.pi/2, np.pi, np.pi*1.5][line_index]

        crosses = [(-2,2), (-2,3), (2,3)]
        robot_pose = [(x, y), theta]
        
        return find_nearest_cross(crosses,robot_pose)

    
    

    def line_intersection(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here
    
        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]
    
        div = det(xdiff, ydiff)
        if div == 0:
           raise Exception('lines do not intersect')
    
        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y
        
    def updateOdometryMessage(self,value): 
        self.odometry_msg.header.stamp = rospy.Time.now()
        if self.value is not None:
            # Get estimated robot position
            (tf_x,tf_y,yaw) = self.get_current_position()
            
            
                        
            # Unpack state
            (x, y), theta1, theta2 = value
            sensed_x = (x*4 -300.)/1000. # [m]
            sensed_y = (300 - y*4) /1000. + 0.5  # [m]
            

            
            # Move point to robot frame
            point_in_robot_frame_x = sensed_x
            point_in_robot_frame_y = sensed_y
            
            # + np.sqrt(0.5**2)*cos(theta)            
#             + np.sqrt(0.5**2)*sin(theta)
            
            # Estimate position of sensed point in world frame
            point_in_world_frame_x = tf_x + np.cos(yaw) * point_in_robot_frame_x + point_in_robot_frame_y * np.sin(yaw)
            point_in_world_frame_y = tf_y - np.sin(yaw) * point_in_robot_frame_x + point_in_robot_frame_y * np.cos(yaw)
            
            # Get nearest real point
            (known_x, known_y, closest_line_angle) = self.getNearestPoint(point_in_world_frame_x, point_in_world_frame_y, yaw)
               

            # Discern correct angle'
            if theta1 > 90:
                theta1 = -180 + theta1               
            if theta2 > 90:
                theta2 = -180 + theta2               
            if abs(theta1) < abs(theta2):
                  actual_angle_of_line_to_robot = closest_line_angle + theta1
            else:
                  actual_angle_of_line_to_robot = closest_line_angle + theta2                   
             
                     
                     
                    
            distance_to_crossing = np.sqrt( point_in_robot_frame_x**2 + point_in_robot_frame_y**2) 
            
            point_in_robot_frame_x_angled = point_in_robot_frame_x * np.cos( actual_angle_of_line_to_robot ) + point_in_robot_frame_y * np.sin(actual_angle_of_line_to_robot)
            point_in_robot_frame_y_angled = -point_in_robot_frame_x * np.sin( actual_angle_of_line_to_robot ) + point_in_robot_frame_y * np.cos(actual_angle_of_line_to_robot)
            crossing_with_zero_degree_line = self.line_intersection(((0.0, 0.0), (0, 0.5)), ((point_in_robot_frame_x, point_in_robot_frame_y), (point_in_robot_frame_x_angled, point_in_robot_frame_y_angled)))      
            
            distance_zero_degree_line =  np.sqrt( crossing_with_zero_degree_line[0]**2 + crossing_with_zero_degree_line[1]**2)               

            theta_of_cross_to_robot = np.arccos(  np.cos(actual_angle_of_line_to_robot) / (distance_to_crossing * distance_zero_degree_line  ) )

            # Calculate robot position
            robot_x = known_x + point_in_robot_frame_x * np.cos( theta_of_cross_to_robot ) + point_in_robot_frame_y * np.sin(theta_of_cross_to_robot)
            robot_y = known_y -point_in_robot_frame_x * np.sin( theta_of_cross_to_robot ) + point_in_robot_frame_y * np.cos(theta_of_cross_to_robot)
            robot_theta = 180 - actual_angle_of_line_to_robot - theta_of_cross_to_robot
            # Update odometry message
            self.odometry_msg.pose.pose.position.x = robot_x
            self.odometry_msg.pose.pose.position.y = robot_y
            self.odometry_msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, robot_theta)

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