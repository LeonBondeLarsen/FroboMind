#!/usr/bin/env python 
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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
import math


class LineDetector(object):
    def __init__(self):
        # Init line detector
        self.camera = cv2.VideoCapture(0)
#        self.f = file("/home/leon/code/roswork/build/fmApp/sdu/group6/line_detector/catkin_generated/installspace/transformMatrix", 'rb')
#        self.transformMatrix = pickle.load(self.f)

        self.crosses = [
                            #[-1.70,0.70],
                            [-2.73,0.72],
                            [-2.75,1.72],
                            [-1.51,1.76],
                            [-0.95,1.78],
                            [1.25,1.82],
                            [1.93,1.82]
                        ]
                
        destination = np.array([(100,500),(100,100),(500,100),(500,500)],np.float32)
        source = np.array([(31,468),(105,28),(506,21),(590,454)],np.float32)
        self.transformMatrix = cv2.getPerspectiveTransform(source,destination)
        
        self.odometry_msg = Odometry()
        self.listener = tf.TransformListener() 
        
        # Init node
        output_topic = rospy.get_param("~pose_pub", "/fmInformation/line_pose")
        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)

        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.z = 0.0
        self.marker_publisher = rospy.Publisher("/line_cross_marker", Marker, queue_size=10)
        
        self.r_marker = Marker()
        self.r_marker.header.frame_id = "world"
        self.r_marker.type = self.r_marker.ARROW
        self.r_marker.action = self.r_marker.ADD
        self.r_marker.scale.x = 0.2
        self.r_marker.scale.y = 0.2
        self.r_marker.scale.z = 0.2
        self.r_marker.color.a = 1.0
        self.r_marker.color.r = 1.0
        self.r_marker.color.g = 1.0
        self.r_marker.color.b = 0.0
        self.r_marker.pose.orientation.w = 1.0
        self.r_marker.pose.position.z = 0.0
        self.r_marker_publisher = rospy.Publisher("/line_robot_marker", Marker, queue_size=10)
    
        # Init TF listener
        self.tf_listener = tf.TransformListener()
        self.quaternion = np.empty((4, ), dtype=np.float64)
        
        rospy.Timer(rospy.Duration(0.005), self.onCameraTimer)
        rospy.Timer(rospy.Duration(0.1), self.onPublishTimer)
                       
    def onCameraTimer(self,event):
        ret, image = self.camera.read()
        image = cv2.warpPerspective(image,self.transformMatrix,(600,600))
        detector = line_and_cross_detector(1,1)
        self.value = detector.analyze_image(image)      
        self.updateOdometryMessage(self.value)

    def onPublishTimer(self,event):
        self.publisher.publish(self.odometry_msg)
        #self.cross_publisher.publish(self.cross_marker)
        

    def getNearestPoint(self,x_r,y_r,  x, y, theta):
        def find_nearest_cross(crosses, robot_pose):
        
            def distance(cross, camera_point):
                return np.sqrt( (cross[0] - camera_point[0])**2 + (cross[1] - camera_point[1])**2)
        
            def create_lines(cross):
                list_of_lines = []
                for angle in [0, np.pi/2, np.pi, np.pi*1.5]:  
                    list_of_lines.append( (cross[0]  + np.cos(angle)/10, cross[1] - np.sin(angle)/10 ) )
                    
                print "list_of_lines", list_of_lines
                return list_of_lines
                
        
#            robot_angle = np.pi * ( robot_pose[1] + 90 ) /180
#            camera_point = (robot_pose[0][0] - np.cos(robot_angle)/2, robot_pose[0][1] + np.sin(robot_angle)/2 )
#            print "camera_point", camera_point    
            dist_index = 0
            for index, cross in enumerate( crosses ):
                if distance(cross, (x,y) ) < distance(crosses[dist_index],(x,y)):
                    dist_index = index
            
#            print distance(crosses[dist_index], camera_point)
#            print crosses[dist_index]
            possible_lines = create_lines(crosses[dist_index])
#            print possible_lines
        
            line_index = 0
            for index, line in enumerate( possible_lines ):
                if distance(line, (x_r, y_r)) < distance(possible_lines[line_index],(x_r, y_r)):
                    line_index = index
            
            print "possible_lines", possible_lines[line_index]
#            print line_index
            #return crosses[dist_index], [180, 270, 0, 90][line_index]
            return crosses[dist_index], [0, 90, 180, 270][line_index]
        
        robot_pose = [(x, y), theta]
        
        return find_nearest_cross(self.crosses,robot_pose)

    
    

    def line_intersection(self,line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here
    
        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]
    
        div = det(xdiff, ydiff)
        if div == 0:
           return None
    
        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y
        
    def updateOdometryMessage(self,value): 
        self.odometry_msg.header.stamp = rospy.Time.now()
        if value is not None:
            # Get estimated robot position
            #(tf_x,tf_y,yaw) = (0.95,2.8,np.pi/2 )      
            #(tf_x,tf_y,yaw) = (0.5,0,np.pi ) 
            #(tf_x,tf_y,yaw) = (0,0.5,np.pi*1.5 ) 
            (tf_x,tf_y,yaw) = self.get_current_state()
            
            print "(tf_x,tf_y,yaw)", (tf_x,tf_y,yaw)
                        
            # Unpack state
            (x, y), theta1, theta2 = value
            sensed_x = (x*4 -300.)/1000. # [m]
            sensed_y = (300 - y*4) /1000. + 0.5  # [m]
            
            print sensed_x, sensed_y, theta1, theta2
            
            # Move point to robot frame
            point_in_robot_frame_x = sensed_x
            point_in_robot_frame_y = sensed_y
            
            # + np.sqrt(0.5**2)*cos(theta)            
#             + np.sqrt(0.5**2)*sin(theta)
            
            # Estimate position of sensed point in world frame
            #point_in_world_frame_x = tf_x + np.cos(yaw + np.pi/2) * point_in_robot_frame_x + point_in_robot_frame_y * np.sin(yaw + np.pi/2)
            #point_in_world_frame_y = tf_y - np.sin(yaw + np.pi/2) * point_in_robot_frame_x + point_in_robot_frame_y * np.cos(yaw + np.pi/2)

            point_in_world_frame_x = tf_x + np.cos(-yaw ) * point_in_robot_frame_y - point_in_robot_frame_x * np.sin(-yaw )
            point_in_world_frame_y = tf_y - np.sin(-yaw ) * point_in_robot_frame_y - point_in_robot_frame_x * np.cos(-yaw )

            self.marker.pose.position.x = point_in_world_frame_x
            self.marker.pose.position.y = point_in_world_frame_y 
            self.marker_publisher.publish(self.marker)

            print "point_in_world_frame", point_in_world_frame_x, point_in_world_frame_y
            
            # Get nearest real point
            (known_x, known_y), closest_line_angle = self.getNearestPoint(tf_x, tf_y, point_in_world_frame_x, point_in_world_frame_y, yaw)
               

            # Discern correct angle'
            if theta1 > 90:
                theta1 = -180.0 + theta1               
            if theta2 > 90:
                theta2 = -180.0 + theta2               
            if abs(theta1) < abs(theta2):
                  actual_angle_of_line_to_robot = float(closest_line_angle) - theta1
                  closest_angle_to_line = theta1
            else:
                  actual_angle_of_line_to_robot = float(closest_line_angle) - theta2                   
                  closest_angle_to_line = theta2
                     
            print "theta", closest_angle_to_line
            print "point position to robot", float(closest_line_angle), known_x, known_y
                    
            distance_to_crossing = np.sqrt( point_in_robot_frame_x**2 + point_in_robot_frame_y**2) 
            
            print "distance_to_crossing", distance_to_crossing
            
            point_in_robot_frame_x_angled = point_in_robot_frame_x + ( 0 * np.cos( closest_angle_to_line * np.pi/180.0 ) + 1 * np.sin(closest_angle_to_line * np.pi/180.0 ) )
            point_in_robot_frame_y_angled = point_in_robot_frame_y + ( 0 * np.sin( closest_angle_to_line * np.pi/180.0 ) + 1 * np.cos(closest_angle_to_line * np.pi/180.0 ) )
            print "point_in_robot_frame_x_angled", point_in_robot_frame_x_angled, point_in_robot_frame_y_angled


            crossing_with_zero_degree_line = self.line_intersection(((0.0, 0.0), (0.5, 0.0)), ((point_in_robot_frame_x, point_in_robot_frame_y), (point_in_robot_frame_x_angled, point_in_robot_frame_y_angled)))      
            if crossing_with_zero_degree_line is None:
                print "lines do not intersect"
                return -1
            print "crossing_with_zero_degree_line", crossing_with_zero_degree_line
            
            
            distance_zero_degree_line =  np.sqrt( crossing_with_zero_degree_line[0]**2 + crossing_with_zero_degree_line[1]**2)     
            distance_line_to_cross = np.sqrt( (crossing_with_zero_degree_line[0] - point_in_robot_frame_x )**2 + ( crossing_with_zero_degree_line[1] - point_in_robot_frame_y)**2 )          
            print "distance_zero_degree_line", distance_zero_degree_line
            print "distance_zero_to_cross", distance_line_to_cross

            #if not (abs(point_in_robot_frame_x) > abs(crossing_with_zero_degree_line[0]) and math.copysign(1, crossing_with_zero_degree_line[0]) == math.copysign(1, point_in_robot_frame_x)):
            if math.copysign(1, crossing_with_zero_degree_line[0]) == math.copysign(1, closest_angle_to_line):
                angle_in_triangle = 90.0 + abs(closest_angle_to_line)
            else:
                angle_in_triangle = 90.0 - abs(closest_angle_to_line)
            
            print angle_in_triangle
            theta_of_cross_to_robot = np.arcsin(  np.sin( ( angle_in_triangle)  * np.pi/180.0 ) * distance_zero_degree_line / (distance_to_crossing   ) )
            print theta_of_cross_to_robot * 180.0/np.pi
            
            theta_of_cross_to_robot = math.copysign(theta_of_cross_to_robot, crossing_with_zero_degree_line[0])
                    #print "test", np.cos(closest_angle_to_line * np.pi/180.0 ) * distance_zero_degree_line / (distance_to_crossing   ) + float(closest_line_angle)
            print "theta_of_cross_to_robot", theta_of_cross_to_robot * 180.0/np.pi
            print "angle to line", theta_of_cross_to_robot * 180.0/np.pi + closest_line_angle
            
            
            theta_of_cross_to_robot = np.arccos( (distance_to_crossing**2 + distance_line_to_cross**2 - distance_zero_degree_line**2)/(2*distance_line_to_cross*distance_to_crossing)  )
            theta_of_cross_to_robot = math.copysign(theta_of_cross_to_robot, crossing_with_zero_degree_line[0])
            print "theta_of_cross_to_robot", theta_of_cross_to_robot * 180.0/np.pi

            # Calculate robot position
            #robot_x = known_x + point_in_robot_frame_y * np.cos( theta_of_cross_to_robot + ( closest_line_angle) * np.pi/180.0 +  np.pi/2) - point_in_robot_frame_x * np.sin( theta_of_cross_to_robot + ( closest_line_angle) * np.pi/180.0 +  np.pi/2 )
            #robot_y = known_y -point_in_robot_frame_y * np.sin( theta_of_cross_to_robot + ( closest_line_angle) * np.pi/180.0 + np.pi/2) - point_in_robot_frame_x * np.cos( theta_of_cross_to_robot  +  (closest_line_angle) * np.pi/180.0 +  np.pi/2  )
#            robot_theta = 180.0 - angle_in_triangle - theta_of_cross_to_robot * 180.0/np.pi_            
            robot_theta = -closest_angle_to_line + (closest_line_angle + 180) % 360
            robot_x = known_x + distance_to_crossing * np.cos( theta_of_cross_to_robot + ( closest_line_angle) * np.pi/180.0 ) + 0 * np.sin( theta_of_cross_to_robot + ( closest_line_angle) * np.pi/180.0  )
            robot_y = known_y - distance_to_crossing * np.sin( theta_of_cross_to_robot + ( closest_line_angle) * np.pi/180.0 ) + 0 * np.cos( theta_of_cross_to_robot  +  (closest_line_angle) * np.pi/180.0  )            
            #robot_x = known_x + point_in_robot_frame_y * np.cos( ( -robot_theta) * np.pi/180.0 ) + point_in_robot_frame_x * np.sin( ( -robot_theta) * np.pi/180.0  )
            #robot_y = known_y - point_in_robot_frame_y * np.sin( ( -robot_theta) * np.pi/180.0 ) + point_in_robot_frame_x * np.cos( ( -robot_theta) * np.pi/180.0  )            

            print robot_x, robot_y, robot_theta  
            # Update odometry message


            
            # Update odometry message
            self.odometry_msg.pose.pose.position.x = robot_x
            self.odometry_msg.pose.pose.position.y = robot_y
            q = tf.transformations.quaternion_from_euler(0, 0,- robot_theta * np.pi / 180)
            self.odometry_msg.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
  
            # Set covariance to good quality
            self.odometry_msg.pose.covariance[0] = 0.00001 # variance x
            self.odometry_msg.pose.covariance[7] = 0.00001 # variance y
            self.odometry_msg.pose.covariance[35] = 0.00001 # variance theta

            self.r_marker.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            self.r_marker.pose.position.x = robot_x
            self.r_marker.pose.position.y = robot_y 
            self.r_marker_publisher.publish(self.r_marker)
        else:
            pass
            # Set covariance to poor quality
            self.odometry_msg.pose.covariance[0] = 1 # variance x
            self.odometry_msg.pose.covariance[7] = 1 # variance y
            self.odometry_msg.pose.covariance[35] = 1 # variance theta
            
    def get_current_state(self):
        
        try:
            (position,heading) = self.listener.lookupTransform("world", "base",  rospy.Time(0) )
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(heading)
            return (position[0],position[1],yaw)
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo(rospy.get_name() + " : could not locate vehicle "+str(err))
            return (0.0,0.0,0.0)


import time
                    
if __name__ == '__main__':
    rospy.init_node('line_detector')
    node = LineDetector()
    rospy.spin()