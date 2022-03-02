#!/usr/bin/env python

#import numpy as np
#import math
#import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion




def callback(msg):
#    yt=0;
    y_s=0;
    y_p=0;
    Kp=0.1;
    Kd=0.05;
    Ki=0.025;
    xm = msg.pose.pose.position.x
    ym = msg.pose.pose.position.y
    #q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    #euler = euler_from_quaternion(q)
    #z_orien = euler[2]
    if xm<2:
        move_cmd.linear.x = 0.05
        cmd_vel.publish(move_cmd)
        while ym !=0:
            z_a = (Kp*ym) + (Kd*abs(y_p-ym)) + (Ki*y_s)
            move_cmd.angular.z=z_a
            cmd_vel.publish(move_cmd)
            y_p=ym
            y_s+=ym
            rospy.loginfo('x_position: {}'.format(xm))
            rospy.loginfo('y_position: {}'.format(ym))
    else:
        move_cmd.linear.x=0
        cmd_vel.publish(move_cmd)
    
if __name__ == '__main__':
    try:
        rospy.init_node('MoveForward_PID')
        move_cmd = Twist()
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #queue_size is a prime factor
        rospy.Subscriber("/odom", Odometry, callback)
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")


