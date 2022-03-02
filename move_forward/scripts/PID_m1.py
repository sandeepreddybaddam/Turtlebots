#!/usr/bin/env python

#import numpy as np
#import math
#import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion
y_p=0;

def callback(msg, y_p):
#    y_s=0;
#    y_p=0;
    Kp=2;
    Kd=2;
#    Ki=2.5;
    xm = msg.pose.pose.position.x
    ym = msg.pose.pose.position.y
    if xm<3:  
        move_cmd.linear.x = 0.1
        cmd_vel.publish(move_cmd)
        if ym !=0:
#            z_a = -[(Kp*ym) + (Kd*(ym-y_p)) + (Ki*y_s)]
            z_a = -Kp*ym-Kd*(ym-y_p)
            move_cmd.linear.x = 0.1
            move_cmd.angular.z=z_a
            cmd_vel.publish(move_cmd)
            
            y_p=ym
#            y_s=y_s+ym
            rospy.loginfo('x_position: {}'.format(xm))
            rospy.loginfo('y_position: {}'.format(ym))
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        
    else:
        move_cmd.linear.x=0
        move_cmd.angular.z=0
        cmd_vel.publish(move_cmd)
    
if __name__ == '__main__':
    try:
        rospy.init_node('MoveForward_PID')
        move_cmd = Twist()
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #queue_size is a prime factor
        rospy.Subscriber("/odom", Odometry, callback, y_p)
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")


