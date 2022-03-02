#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time

xp = 0;
t0 = time.time();
tp = t0;
zp = 0;

Kp_turn = 0.2; #This cannot be zero. If =0, then constant TF
#Kd_turn = 1;
#Ki not needed because the order is sufficient to make the SSE zero

Kp_linear = 0.4;
Kd_linear = 0.01;# If Kd=0 besides Ki=0, then it becomes marginally stable

def callback(msg):
    
    x = msg.data[9];
    #y = msg.data[10];
    z = msg.data[11];
    t = time.time()
    global xp #added global so that we can modify it globally
    global tp
    global zp

    if z>0.20:
        speed = -1*(-Kp_linear*z-Kd_linear*(z-zp)/(t-tp))
        speed = min(0.2, speed)
        x_cam = -(Kp_turn*x)    
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
        rospy.Subscriber('april', Float32MultiArray, callback)
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")
