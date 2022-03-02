#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.init_node('Subscriber_odometry')
c = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_cmd = Twist()
r = rospy.Rate(2)###??


def shutdown():
    move_cmd.linear.x = 0
    rospy.loginfo("Stop TurtleBot")
    
def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    while not rospy.is_shutdown():
        rospy.loginfo('x_position: {}'.format(x))
        rospy.loginfo('y_position: {}'.format(y))
        move_cmd.linear.x = 0.01
        r.sleep()

rospy.Subscriber("/odom", Odometry, callback)
c.publish(move_cmd)
rospy.on_shutdown(shutdown)
rospy.spin()