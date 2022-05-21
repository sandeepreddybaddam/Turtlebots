#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import time
import message_filters #Share this info with team

"""
Object avoidance algorithm -- basic version
Input: Showing cardboard in camera view
Output: Make necessary steers
slow down as the obstacle approaches

and rotate back to find the april tag

worked on reducing latency
"""

class follower_obj_avoid:
    def __init__(self):

        self.Kp_turn = 0.4;
        self.Kp_linear = 0.4;
        self.count = 0

        self.move_cmd = Twist()
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #queue_size is a prime factor

        laser_sub = message_filters.Subscriber('/scan', LaserScan) # subscriber for laser scan
        april_sub = message_filters.Subscriber('april1', Float32MultiArray) # subscribing for camera apriltag topic

        ts = message_filters.ApproximateTimeSynchronizer([laser_sub, april_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, laser_sub, april_sub):
        left_scan = laser_sub.ranges[0:44] # scanning for obstacles in left zone from 0 to 45 degrees
        right_scan = laser_sub.ranges[315:359] # scanning for obstacles in right zone from -45 to 0 degrees
        flag = 0
        for i in left_scan:
            if i<0.5 and abs(i)!=0:
                change_rot = -0.3 #subject to change
                flag = 1 # raise flag if there is obstacle
                speed_factor = (0.5/0.2)*i
                self.count = 0
                self.count+=1
                print("Left_obs")
        for j in right_scan:
            if j<0.5 and abs(j)!=0:
                change_rot = 0.3
                flag = 1 # raise flag if there is obstacle
                speed_factor = (0.5/0.2)*j
                self.count = 0
                self.count-=1
                print("Right_obs")
        if flag==0:
            change_rot = 0
            speed_factor = 1
            print("No_obs")

        x = april_sub.data[9]; # lateral distance of apriltag wrt camera
        #y = msg.data[10];
        z = april_sub.data[11]; # longitudinal distance of apriltag wrt camera

        if z>0.20:
            speed = self.Kp_linear*(z-0.2)
            speed = min(0.18, speed)
            x_cam = -(self.Kp_turn*x)    
            self.move_cmd.linear.x = speed*speed_factor
            x_cam = min(1.4, x_cam)
            self.move_cmd.angular.z = x_cam + change_rot
            self.cmd_vel.publish(self.move_cmd)

        elif self.count>0 and x==0 and z==0: #if no april tag and did some obtacle avoidance just before
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0.2 #detected left obstacle before. So, rotate opposite to find april tag
            self.cmd_vel.publish(self.move_cmd)

        elif self.count<0 and x==0 and z==0: #if no april tag and did some obtacle avoidance just before
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = -0.2 #detected left obstacle before. So, rotate opposite to find april tag
            self.cmd_vel.publish(self.move_cmd)
            
        else:
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)

if __name__ == '__main__':
    rospy.init_node('follower')
    follower_obj_avoid()
    rospy.spin()

