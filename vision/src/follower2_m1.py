#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time
import math
from math import sin, cos, atan
from tf.transformations import euler_from_quaternion
import numpy as np
global tp
global V
tp = 0

class Follower:
    def __init__(self):
        
        # self.xp = 0;
        # self.t0 = time.time();
        # self.tp = self.t0;
        # self.zp = 0;
        self.Kp_turn = 1.4;
        self.Kd_turn = 0;
        self.Kp_linear = 0.5;
        self.Kd_linear = 0;
        self.move_cmd = Twist()
        self.cmd_vel = rospy.Publisher("tb3_3/cmd_vel", Twist, queue_size=10)
        #queue_size is a prime factor
        self.sub = rospy.Subscriber("april1", Float32MultiArray, self.callback)
        print("init")
    

    def callback(self, msg):
        #global tp
        #global V
        x = msg.data[9];
        #y = msg.data[10];
        z = msg.data[11];
        t = time.time()
        if z != 0:
            theta_f = atan(x/z);
        else:
            theta_f = 0
        print(theta_f)
        # dT = t-tp
        print("before_if")
        if z>0.20:
            speed = self.Kp_linear*z;
            if speed>=0:
                speed = min(0.2, speed)
            elif speed<0:
                speed = max(-0.2, speed)
            x_cam = self.Kp_turn*theta_f # check the sign and radians    
            self.move_cmd.linear.x = speed
            self.move_cmd.angular.z = -x_cam
            self.cmd_vel.publish(self.move_cmd)
            #tp = t
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