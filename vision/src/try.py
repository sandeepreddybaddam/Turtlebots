#10/15 Local PD

print("start")
#import pyrealsense2.pyrealsense2 as rs
from rosgraph import xmlrpc
import rospy
import numpy as np
import cv2
import math #need sqrt for tag width
import struct #bytes -> numbers
import rospy #python library for ROS
from geometry_msgs.msg import Twist  #moving bot
import time #fps calculation
import matplotlib.pyplot as plt
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry #odometry
from math import radians, copysign, sqrt, pow, pi, atan2
import sys, signal
from sensor_msgs.msg import LaserScan
import itertools as it
from apriltags.msg import AprilTagDetections
from sensor_msgs.msg import Range

#function to make program stop correctly

class turtlebot:
    def __init__(self,start = time.time()):
        # Params
        # Start time
        self.prev_time = start

        # Start Count
        self.fpscount=0
        self.count=0 
        self.lidar_count=0

        # Start Move Commands
        self.movelinearx=0
        self.moveangularz=0
        self.front_dist = 1.5

        # Leader Bot Goals
        self.distance_x=5
        self.distance_y=-3

        self.obstacle_depth_r = 0
        self.obstacle_offset_r = 0
        self.obstacle_depth_l = 0
        self.obstacle_offset_l = 0

        self.front_object = 0
        self.front_obstacle_dist = 1.5

        # Initialize the Subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarcallback)
        self.TOF_sub = rospy.Subscriber('vl53l1x/range',Range, self.TOFcallback)


        # Initialize the Publisher 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.move = Twist()
    
    def angle_conversion(self):
        # Converts the turtlebot -pi to pi --> 0 to 2pi ----> NEEDS WORK
        if self.previous_rotation >pi-.1 and self.rotation <=0:
            self.rotation =2*pi + self.rotation
        elif self.previous_rotation < -pi+.1 and self.rotation >0:
            self.rotation = -2*pi + self.rotation
            
    

    def controller(self, distance, total_distance, diff_distance, path_angle, total_angle, diff_angle):
        """
            This will actually run the bot.
        """
        print("------Move Command Count ------",self.count)

        # Gains
        kp_distance = 1 #1
        ki_distance = 0.01 #unused
        kd_distance = 0.5 #.5
        # Angle PID Gain
        kp_angle = 1 #1
        ki_angle = 0.03 #unused
        kd_angle = 0.5 #.05 
        
        # The distance from the goal that it stops ---> Should be very small
        stop_distance = .3

        # move to goal
        if distance > stop_distance:
            # PD control, integral not set up                               
            controlx = kp_distance*(distance-stop_distance) + ki_distance*total_distance + kd_distance*diff_distance 
            controlz =  (kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle-self.obstacle_offset_l+self.obstacle_offset_r)  #whole thing negative... why?
            print('controlz', controlz)
            # Set max speeds for X and Z when no object blocking
            if self.front_dist < .1:
                print('-----slow speed control------')
                self.movelinearx = min(controlx,.08) #sets max speed
                if controlz > 0:                                
                    self.moveangularz= min(controlz,.25) #.5
                else:
                    self.moveangularz= max(controlz,-.25) #.5
            else:
                print('------ normal speed control-----')
                self.movelinearx = min(controlx,.13) #sets max speed
                if controlz-self.rotation > 0:                      
                    self.moveangularz= min(controlz-self.rotation,.275) #.5
                else:
                    self.moveangularz= max(controlz-self.rotation,-.275) #.5
                self.front_object = 0
                
            # Print variables for analysis
            print("Start Pose x,y",self.start_posx,self.start_posy)
            #print("Lidar X,Y,depth (avg)",self.lidar_x,self.lidar_y,self.lidar_depth)
            print("final goal", self.final_x, self.final_y)
            print ("pos x y", self.posx, self.posy)
            print("Front Distance", self.front_dist)
            #print("_________Front Obstacle Distance", self.front_obstacle_dist)
            #print("Front Obstacle angle", self.xuk_front[1])
            print("Obstacle right", self.obstacle_depth_r)

        # stop if within range
        else:
            self.movelinearx = 0
            self.moveangularz= 0
            print("Close to Tag")
        
        # integral control (not variables wokring)
        # total_distance= total_distance + distance #not working, canceled in controlx eq
        # total_angle= total_angle + path_angle #not working, cancled in controlz eq
        print('Velocity', self.movelinearx)
        print('Angular Velocity', self.moveangularz)

        # publish move command
        self.move.linear.x = self.movelinearx
        self.move.angular.z = self.moveangularz
        self.pub.publish(self.move)
    
   
        

    def callback(self, msg):

        # Read current position from the odometer ---> This does not reset to zero unless bringup is restarted
        self.posx = msg.pose.pose.position.x
        self.posy = msg.pose.pose.position.y
            
        # Read current rotation from the odometer ---> This does not reset to zero unless bringup is restarted
        rot = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        rotation_array = euler_from_quaternion(rot)
        self.rotation = rotation_array[2]
        
        print("TB Rot:", self.rotation)

        # Initialize variables during the first loop through.
        if self.count == 0:
            # total_angle and total_distance are meant for the Integral control ---> currently not working
            total_angle = 0 ##rotation
            total_distance = 0 ##sqrt(pow(self.lidar_x,2)+pow(self.lidar_y,2))

            # Save the initial starting (x,y) location and rotation 
            self.start_posx = msg.pose.pose.position.x
            self.start_posy = msg.pose.pose.position.y
            self.rotation_start = self.rotation # getting .05ish initial rotation when should be 0...

            
            self.final_x =  self.start_posx + self.distance_x
            self.final_y =  self.start_posy + self.distance_y

            # Initialize the previous variables for future use
            self.previous_distance = 0
            self.previous_angle = 0
            self.previous_rotation = 0

            # Add to the count so it doesn't repeat the initialization
            self.count += 1       

        else: 

            # Set the distance between the bot and the final location to be the goal ---> we want these to go to zero
            goal_x = self.final_x-self.posx
            goal_y = self.final_y-self.posy

            # Determine angle and total distance ---> we want these to go to zero
            path_angle = atan2(goal_y,goal_x)
            distance = sqrt(goal_x**2 + goal_y**2)

            # These are meant for the Derivative control ---> Ask Austin how he came up with them
            diff_distance = distance - self.previous_distance
            diff_angle = path_angle - self.previous_angle
            
            # These are meant for the Integral control ---> currently not working
            total_angle = 0 ##rotation
            total_distance = 0 ##sqrt(pow(self.lidar_x,2)+pow(self.lidar_y,2))
            
            # Converts the turtlebot -pi to pi --> 0 to 2pi ---> NEEDS WORK 
            self.angle_conversion()
            
            #obstacle offset right
            obstacle_coeff_r=2.25 #up to 1.5
            if self.obstacle_depth_r<.7:
                self.obstacle_offset_r= obstacle_coeff_r*(.7-self.obstacle_depth_r)
            else:
                self.obstacle_offset_r=0   

            #obstacle offset left
            obstacle_coeff_l=2.25 #up to 1.5
            if self.obstacle_depth_l<.7:
                self.obstacle_offset_l= obstacle_coeff_l*(.7-self.obstacle_depth_l)
            else:
                self.obstacle_offset_l=0 
            

            print('Distance', distance)
            print('Path Angle', path_angle)
            # Run the controller for the bot
            self.controller(distance, total_distance, diff_distance, path_angle, total_angle, diff_angle)

            # Set previous rotation, distance and angle
            self.previous_rotation= self.rotation
            self.previous_distance = distance
            self.previous_angle = path_angle

            print("goal x,y", goal_x,goal_y)
            
            # Add to the count
            self.count+=1



def signal_handler(signal, frame):

    print('finish')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


rospy.init_node('lidar')
follower_node = turtlebot()
cv2.waitKey(1)
rospy.spin()

