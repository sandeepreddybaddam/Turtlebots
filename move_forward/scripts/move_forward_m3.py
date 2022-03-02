#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.init_node('Subscriber_odometry')
c = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_cmd = Twist()
r = rospy.Rate(2)###??

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    move_cmd.linear.x = 0
    while not rospy.is_shutdown():
        rospy.loginfo('x_position: {}'.format(x))
        rospy.loginfo('y_posi tion: {}'.format(y))
        move_cmd.linear.x = 0.01
        c.publish(move_cmd)
        r.sleep()
move_cmd.linear.x = 0
c.publish(move_cmd)
rospy.Subscriber("/odom", Odometry, callback)
rospy.spin()
