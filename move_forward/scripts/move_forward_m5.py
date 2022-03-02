#!/usr/bin/env python

import numpy as np
import math
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

x1=0;
y1=0;

def angle(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    f = (y2-y1)/math.sqrt(xd*xd +yd*yd)
    return np.arcsin(f) #radians

def callback(msg):
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y
    q = (msg.pose.pose.orientation.x, 
         msg.pose.pose.orientation.y, 
         msg.pose.pose.orientation.z, 
         msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(q)
    z_orien = euler[2]
    
    if x2<4:
        move_cmd.linear.x = 0.05
        if y2<=0.005 and y2>=-0.005:
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            
        elif y2>0.005:
            move_cmd.linear.x = 0.05
            t = (angle(x1, y1, x2, y2)+z_orien*22/7)/0.08        
            if t>=0 and abs(z_orien) < 22/(7*2) :#refer book
                move_cmd.angular.z = -0.08 #point of interest
                cmd_vel.publish(move_cmd)
#                time.sleep(t)
#                print t, '\n'
                print(z_orien)
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            
        elif y2<-0.005:
            move_cmd.linear.x = 0.05
            t = (angle(x1, y1, x2, y2)+z_orien*22/7)/(-0.08)
            if t>=0 and abs(z_orien) < 22/(7*2):
                move_cmd.angular.z = 0.08
                cmd_vel.publish(move_cmd)
#                time.sleep(t)
#                print t, '\n'
                print(z_orien)
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            
    else:
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        cmd_vel.publish(move_cmd)
        
    rospy.loginfo('x_position: {}'.format(x2))
    rospy.loginfo('y_position: {}'.format(y2))
    
if __name__ == '__main__':
    try:
        rospy.init_node('MoveForward')
        move_cmd = Twist()
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #queue_size is a prime factor
        rospy.Subscriber("/odom", Odometry, callback)
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")


