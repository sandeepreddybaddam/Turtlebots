#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def callback(msg):
    
    print("i")
    while not rospy.is_shutdown():
        print("Hi")

if __name__ == '__main__':
    rospy.init_node('get_image')
    rospy.Subscriber("/camera/color/image_raw", Image , callback)
    rospy.spin()