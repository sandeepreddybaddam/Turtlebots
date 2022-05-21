#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time
import math
from math import sin, cos, acos, pi
from tf.transformations import euler_from_quaternion
import numpy as np

"""
Caution: Before running this script, do BRINGUP
check feedforward component

changes: 
1. Kp_y
2. speed loop(to avoid 'dt' trap initially)
implemented low pass filter
"""

class Follower:
    def __init__(self):
        self.tp = time.time()
        self.initial = time.time()
        self.exp = 0
        self.eyp = 0
        self.timelist =[]
        self.phi = []
        self.error_x = []
        self.error_y = []
        self.a = 0 # initial position and later becomes previous 'x' position
        self.count = 0 # this is to make initial phi as zero
        
        self.move_cmd = Twist()
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #queue_size is a prime factor
        rospy.Subscriber("/odom", Odometry, self.callback_odom)
    
    def lowPass(self,y,u): # low pass filter to eliminate noise for errors in x and y states
        return 0.99*y + 0.01*u # how to choose the cutoff frequency
    
    def callback_odom(self, msg):

        x1 = msg.pose.pose.position.x
        y1 = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(q)
        
        #calibration for z orientation
        if self.count=0: #Initial rotation is not always zero. So, this is kind of calibration step
            val = yaw
        phi = yaw - val # substract the initial orientation value
        self.count = 1
        
        # Initializing gain values # PD controller 
        Kp_x = 1 
        Kd_x = 200
        Kp_y = 1
        Kd_y = 200
        
        #Time interval calculation
        t1 = time.time()
        dt = t1-self.tp # time interval
        self.tp = t1

        if dt<0.01: # only for the initial pass through the script. if dt=0, then division by zero occurs
            speed = 0.05
        else:
            speed = max(0.05, round((x1-self.a)/dt, 5))
        
        self.a = x1 # self.a is the previous 'x' state
        
        # Trajectory definition
        freq = 5*pi/180 #one degree in 1 sec
        ang = freq*(t1-self.initial) #parameter in rcos() and rsin()
    
        R = 2 #radius in meters
        # y_ref = R*cos(ang)-R
        # x_ref = R*sin(ang)
        x_ref = 1
        y_ref = 0

        x_ref_dot_dot = -1*R*pow(freq, 2)*sin(ang)
        y_ref_dot_dot = -1*R*pow(freq, 2)*cos(ang)

        e_x = x_ref-x1 #x1 is the current position of bot--reading from /odom #x_ref is the trajectory
        e_y = y_ref-y1
        
        # values in list to plot error
        self.error_x.append(e_x)
        print("Error_x unfiltered: ", self.error_x)
        self.error_y.append(e_y)
        print("Error_y unfiltered: ", self.error_y)
        
        # Try filter out the noise of ex and ey
        e_x = self.lowPass(self.exp, e_x)
        e_y = self.lowPass(self.eyp, e_y)
        
        # values in list to plot error
        self.error_x.append(e_x)
        print("Error_x filtered: ", self.error_x)
        self.error_y.append(e_y)
        print("Error_y filtered: ", self.error_y)

        F_inv = [[cos(phi), sin(phi)],[-sin(phi)/speed, cos(phi)/speed]] #checked

        #x_dot_dot = (Kp_x* e_x)+ Kd_x *(e_x - self.exp)+x_ref_dot_dot
        #y_dot_dot = (Kp_y* e_y)+ Kd_y *(e_y - self.eyp)+y_ref_dot_dot

        ##feedforward??
        x_dot_dot = (Kp_x* e_x) + Kd_x *(e_x - self.exp)
        y_dot_dot = (Kp_y* e_y) + Kd_y *(e_y - self.eyp)

        self.exp = e_x
        self.eyp = e_y
        vv = np.dot(F_inv,[[x_dot_dot],[y_dot_dot]])
        vv = np.array(vv)
        
        speed = speed + vv[0,0] *dt
        ang_vel = vv[1,0]

        if speed > 0.2:
            speed = min(0.2, speed)
        if speed<-0.2:
            speed = max(-0.2, speed)
        if ang_vel > 1.4:
            ang_vel = min(1.4, ang_vel)
        if ang_vel < -1.4:
            ang_vel = max(-1.4, ang_vel)

        self.move_cmd.linear.x = speed
        self.move_cmd.angular.z = ang_vel
        #self.move_cmd.linear.x = 0
        #self.move_cmd.angular.z = 0

        self.cmd_vel.publish(self.move_cmd)
        
        # values in list to plot time
        #actual running clock   (t1-self.initial)
        self.timelist.append(t1-self.initial)
        print("Time List: ", self.timelist)
        # self.phi.append(phi)
        # print(self.phi)
        
        time.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('follower')
    Follower()
    rospy.spin()
