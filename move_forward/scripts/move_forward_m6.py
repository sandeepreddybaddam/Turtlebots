#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import time


x1=0;
y1=0;
t = 0.1

def angle(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    f = (y2-y1)/math.sqrt(xd*xd +yd*yd)
    return np.arcsin(f) #radians

def callback(msg):
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y
#    z_orien = msg.pose.pose.orientation.z
    
    if x2<3:
        move_cmd.linear.x = 0.05
        if y2<=0.005 and y2>=-0.005:
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            
        elif y2>0.005:
            move_cmd.angular.z = -0.01 #point of interest
#            t = (angle(x1, y1, x2, y2)+z_orien*22/7)/1       
            cmd_vel.publish(move_cmd)
            time.sleep(t)
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            
        elif y2<-0.005:
            move_cmd.angular.z = 0.01
#            t = (angle(x1, y1, x2, y2)+z_orien*22/7)/(-1)
            cmd_vel.publish(move_cmd)
            time.sleep(t)
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
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, callback)
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")



