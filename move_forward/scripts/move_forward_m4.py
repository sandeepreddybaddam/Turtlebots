#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

X=0;
Y=0;

def callback(msg):
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y      
    if X<1:
        move_cmd.linear.x = 0.02
        if Y<=0.003 and Y>=-0.003:
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            
        elif Y>0.003:
            move_cmd.angular.z = -0.03
            cmd_vel.publish(move_cmd)
            
        elif Y<-0.003:
            move_cmd.angular.z = 0.03
            cmd_vel.publish(move_cmd)
    else:
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        cmd_vel.publish(move_cmd)
        
    rospy.loginfo('x_position: {}'.format(X))
    rospy.loginfo('y_position: {}'.format(Y))

#def shutdown():
#    rospy.loginfo("Stop TurtleBot")
#    cmd_vel.publish(Twist())
#    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
#    rospy.sleep(1)
    
if __name__ == '__main__':
    try:
        rospy.init_node('MoveForward')
        move_cmd = Twist()
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, callback)
#        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")


