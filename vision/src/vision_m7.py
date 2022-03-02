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

Kp = 0.5;
Kd = 1;

Kp1 = 0.05;
Kd1 = 1;


def callback(msg, arg):

    x = msg.data[9];
    ym = msg.data[10];
    zm = msg.data[11];
    tm = time.time()
    xp = arg[0]
    t0 = arg[1]
    tp = arg[2]
    zp = arg[3]

    #print(z)
    if zm>0.20:
        if x!=0:
            x_a = -(Kp*x)-(Kd*(x-xp)/(tm-tp))
            speed = -(Kp1*zm)-(Kd1*(zm-zp)/(tm-tp))
            move_cmd.linear.x = -speed
            move_cmd.angular.z = x_a
            cmd_vel.publish(move_cmd)        
            xp = x
            zp = zm
            tp = tm
            print("zp: "+ str(zp))
            print("speed: "+ str(speed))

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
        rospy.Subscriber('april', Float32MultiArray, callback, (xp, t0, tp, zp))
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")
