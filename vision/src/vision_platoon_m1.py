#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time
import math
from math import sin, cos
from tf.transformations import euler_from_quaternion
import numpy as np

xp = 0;
t0 = time.time();
tp = t0;
zp = 0;

def angle(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    f = (y2-y1)/math.sqrt(xd*xd +yd*yd)
    return np.arcsin(f) #radians

def callback_odom(msg):
    global x2
    global y2
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y
    q = (msg.pose.pose.orientation.x, 
         msg.pose.pose.orientation.y, 
         msg.pose.pose.orientation.z, 
         msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(q)
    global z_orien
    z_orien = euler[2]

def callback(msg):
    
    x = msg.data[9];
    #y = msg.data[10];
    z = msg.data[11];
    t = time.time()
    alpha = ?? #orientation of lead bot
    x1 = z-0.2*sin(alpha);
    y1 = x-0.2*cos(alpha);
    #x2 defined in callback_odom
    #y2 defined in calback_odom

    global xp #added global so that we can modify it globally
    global tp
    global zp
    dT = t-tp;
    theta_f = z_orien;
    theta_r = angle(x1, y1, x2, y2)
    P1 = dT * (sin(theta_f)+cos(theta_f))
    Kp_linear = 1.2/P1;
    Kp_turn = 1.2/dT;
    if z>0.20:
        speed = Kp_linear*((x1-x2)+(y1-y2));
        speed = min(0.2, speed)
        x_cam = -(Kp_turn*(theta_f-theta_r))......??    
        move_cmd.linear.x = speed
        move_cmd.angular.z = x_cam
        cmd_vel.publish(move_cmd)
        tp = t
        zp = z
        
    else:
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        cmd_vel.publish(move_cmd)
        
if __name__ == '__main__':
    try:
        rospy.init_node('MoveForward_Vision')
        move_cmd = Twist()
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #queue_size is a prime factor
        rospy.Subscriber("/odom", Odometry, callback_odom)
        rospy.Subscriber('april', Float32MultiArray, callback)
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")
