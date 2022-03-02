#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time

class Follower:
    def __init__(self):
        self.xp = 0;
        self.t0 = time.time();
        self.tp = self.t0;
        self.zp = 0;
        self.Kp_turn = 2;
        self.Kd_linear = 2;
        self.move_cmd = Twist()
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #queue_size is a prime factor
        self.sub = rospy.Subscriber("april", Float32MultiArray, self.callback)

    def callback(self, msg):
        x = msg.data[9];
        #y = msg.data[10];
        z = msg.data[11];
        t = time.time()

        if z>0.20:
            speed = -(self.Kd_linear*(z-self.zp)/(t-self.tp))
            x_cam = -(self.Kp_turn*x)    
            self.move_cmd.linear.x = -speed
            self.move_cmd.angular.z = x_cam
            self.cmd_vel.publish(self.move_cmd)
            self.tp = t
            self.zp = z
            
        else:
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            
if __name__ == '__main__':
    rospy.init_node('follower')
    Follower()
    rospy.spin()