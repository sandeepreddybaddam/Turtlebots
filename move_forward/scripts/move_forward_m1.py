#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

X=0;
Y=0;
def callback(msg):
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y
        
    while not rospy.is_shutdown():
        if X<5:
            move_cmd.linear.x = 0.01
            if Y<=0.05 and Y>=-0.05:
                move_cmd.angular.z = 0
                cmd_vel.publish(move_cmd)
            elif Y>0.05:
                move_cmd.angular.z = 0.01
                cmd_vel.publish(move_cmd)
                r.sleep()
            elif Y<-0.05:
                move_cmd.angular.z = -0.01
                cmd_vel.publish(move_cmd)
                r.sleep()
        else:
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
        r.sleep()
        #X = msg.pose.pose.position.x  ??
        #Y = msg.pose.pose.position.y ??
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
        rospy.Subscriber("/odom", Odometry, callback)
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(5);
        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 2 HZ
        move_cmd = Twist()
        
        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        #rospy.on_shutdown(shutdown)
        rospy.spin()###??
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terimated")


