#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time

x_p = 0;
t0 = time.time();
tp = t0;
Kd = 1;
X = [];

def callback(msg, arg):

    x = 100*msg.data[9];
    ym = msg.data[10];
    zm = msg.data[11];
    print("zm: "+str(zm))
    tm = time.time()
    x_p = arg[0]
    t0 = arg[1]
    tp = arg[2]

    #print(z)
    if zm>0.20:
        if x!=0:
            x_a = -(Kd*(x-x_p)/(tm-tp))
            move_cmd.linear.x = 0.05
            move_cmd.angular.z = x_a
            cmd_vel.publish(move_cmd)        
            x_p = x
            tp = tm
        else:
            move_cmd.linear.x = 0.05
            cmd_vel.publish(move_cmd)

        x = 100*msg.data[9];
    else:
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        cmd_vel.publish(move_cmd)
    X.append(x)
    print(X)

if __name__ == '__main__':
    try:
        rospy.init_node('MoveForward_Vision')
        move_cmd = Twist()
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #queue_size is a prime factor
        rospy.Subscriber('april', Float32MultiArray, callback, (x_p, t0, tp))
        
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")
