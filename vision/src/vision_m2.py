#!/usr/bin/env python
import message_filters
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np


def callback(ros_image, ros):
    print("K")
#    rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
#    camera_info_K = np.array(camera_info.K).reshape([3, 3])
#    camera_info_D = np.array(camera_info.D)
#    rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)
    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
    ts.registerCallback(callback)
    rospy.spin()
   
    