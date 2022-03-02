#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



def shutdown():
    move_cmd =Twist()
    move_cmd.linear.x = 0
    c = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    c.publish(move_cmd)
    rospy.loginfo("Stop TurtleBot")
    rospy.sleep(1)
    
def callback(msg):
    #rospy.loginfo('hello')
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    r = rospy.Rate(5)
    move_cmd = Twist()
    rospy.loginfo('x_position: {}'.format(x))
    rospy.loginfo('y_position: {}'.format(y))
    #c = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd.linear.x = 0.1
    c.publish(move_cmd)
    r.sleep()
    

if __name__ == '__main__':
    try:
        rospy.init_node('Subscriber_odometry')
        c = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, callback)
        #rospy.publish(move_cmd)
        rospy.on_shutdown(shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")
    