#!/usr/bin/env python

#import numpy as np
#import math
#import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
#from tf.transformations import euler_from_quaternion
y_p = 0;
y_s = 0;
t0 = time.time();
tp = t0;

def callback(msg, arg):
    y_p = arg[0]
    y_s = arg[1]
    t0 = arg[2]
    tp = arg[3]
    
#    #m1
#    Kp=1;
#    Kd=5;
#    Ki=1;
#    #m2
#    Kp=0.5;
#    Kd=5;
#    Ki=1;

#m3
    Kp=0.5;
    Kd=5;
    Ki=0.1;
    xm = msg.pose.pose.position.x
    ym = msg.pose.pose.position.y
    tm = time.time()
    if xm<4:  
        move_cmd.linear.x = 0.1
        cmd_vel.publish(move_cmd)
        if ym !=0:
            y_s = y_s + ym*(tm-tp)
            z_a = -(Kp*ym)-(Kd*(ym-y_p)/(tm-tp))-(Ki*y_s)
            #z_a = -(Kp*ym)-(Kd*(ym-y_p)/(tm-tp))
            move_cmd.linear.x = 0.1
            move_cmd.angular.z=z_a
            cmd_vel.publish(move_cmd)        
            y_p = ym
            tp = tm
            
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
        rospy.Subscriber("/odom", Odometry, callback, (y_p, y_s, t0, tp))
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")


