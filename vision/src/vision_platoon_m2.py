#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time
import math
from math import sin, cos, acos
from tf.transformations import euler_from_quaternion
import numpy as np
global tp
global V
tp = 0
V = 0.001

class Follower:
    def __init__(self):
        
        # self.xp = 0;
        # self.t0 = time.time();
        # self.tp = self.t0;
        # self.zp = 0;
        self.move_cmd = Twist()
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #queue_size is a prime factor
        rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.sub = rospy.Subscriber("april", Float32MultiArray, self.callback)
        print("init")
        
    def angle(self, x1, y1, x2, y2):
        xd = x1 - x2
        yd = y1 - y2
        f = (y2-y1)/math.sqrt(xd*xd +yd*yd)
        return np.arcsin(f) #radians
    
    def callback_odom(self, msg):
        self.x2 = msg.pose.pose.position.x
        self.y2 = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(q)
        self.z_orien = euler[2]

    def callback(self, msg):
        global tp
        global V
        x = msg.data[9];
        #y = msg.data[10];
        z = msg.data[11];
        t = time.time()
        alpha =  acos(msg.data[0])#orientation of lead bot??/
        x1 = z-0.2*sin(alpha);
        y1 = x-0.2*cos(alpha);
        #x2 defined in callback_odom
        #y2 defined in calback_odom
        
        # dT = t-tp
        
        x2=self.x2
        y2=self.y2
        S = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
        dT = abs(S/V)
        #print(x1,y1,x2,y2)     
        #z_orien = callback_odom()
        theta_f = self.z_orien #check the sign
        theta_r = self.angle(x1, y1, x2,  y2) #angle of the line joining both in global system
        P1 = dT * (sin(theta_f)+cos(theta_f));
        Kp_linear = 1.2/P1; #6
        Kp_turn = 1.2/dT; #6
        print("before_if")
        if z>0.20:
            speed = Kp_linear*((x1-x2)+(y1-y2));
            if speed>=0:
                speed = min(0.2, speed)
            elif speed<0:
                speed = max(-0.2, speed)
            V = speed
            x_cam = -(Kp_turn*(theta_f-theta_r)) # check the sign and radians    
            self.move_cmd.linear.x = speed
            #self.move_cmd.angular.z = x_cam
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            tp = t
            # zp = z
            print("in_if")
            
        else:
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            
if __name__ == '__main__':
    rospy.init_node('follower')
    Follower()
    rospy.spin()
    print("main")