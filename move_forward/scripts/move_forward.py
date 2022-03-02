#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

X=0;
def callback(msg):
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y
    rospy.loginfo('x_position: {}'.format(X))
    rospy.loginfo('y_position: {}'.format(Y))
    r = rospy.Rate(10);
    move_cmd = Twist()
    
    while X<0.5:
	
	# cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	# Twist is a datatype for velocity

	# let's go forward at 0.2 m/s
	move_cmd.linear.x = 0.01
	# let's turn at 0 radians/s
	if Y<=0.01 and Y>=-0.01:
	    move_cmd.angular.z = 0
	elif Y>0.01:
	    move_cmd.angular.z = 0.01
	elif Y<-0.01:
	    move_cmd.angular.z = -0.01
	# as long as you haven't ctrl + c keeping doing...

	    # publish the velocity
	cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
	r.sleep()
	X = msg.pose.pose.position.x
        Y = msg.pose.pose.position.y
	rospy.loginfo('x_position: {}'.format(X))
    	rospy.loginfo('y_position: {}'.format(Y))
    move_cmd.linear.x = 0
    #rospy.on_shutdown(shutdown)

def shutdown():
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    cmd_vel.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep(1)

if __name__ == '__main__':
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('MoveForward', anonymous=False)

    # tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")
    
    rospy.on_shutdown(shutdown)
	# What function to call when you ctrl + c    
    #rospy.on_shutdown(shutdown)
	# Create a publisher which can "talk" to TurtleBot and tell it to move
    
    rospy.Subscriber("/odom", Odometry, callback) ##??
    rospy.spin()
    #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ

