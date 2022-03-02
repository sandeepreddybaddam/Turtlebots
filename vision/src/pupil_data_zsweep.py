#!/usr/bin/env python

import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import apriltag #apriltag
import argparse #apriltag
from pupil_apriltags import Detector
import math #need sqrt for tag width
import struct #bytes -> numbers
import rospy #python library for ROS
from geometry_msgs.msg import Twist  #moving bot
import time #fps calculation

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

#function to make program stop correctly
import sys, signal
def signal_handler(signal, frame):
    fps=count/(time.time()-start)
    print("FPS: ",fps)
    print("pupilz",pupilz_list)
    print("depthz",depthz_list)

    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

#fps counter
count=0
start = time.time()

# r matrix
mat = Float32MultiArray()
mat.layout.dim.append(MultiArrayDimension())
mat.layout.dim.append(MultiArrayDimension())
mat.layout.dim[0].label = "height"
mat.layout.dim[1].label = "width"
mat.layout.dim[0].size = 5
mat.layout.dim[1].size = 3
mat.layout.dim[0].stride = 5*3
mat.layout.dim[1].stride = 3
mat.layout.data_offset = 0

mat_z_sweep = Float32MultiArray()
mat_z_sweep.layout.dim.append(MultiArrayDimension())
mat_z_sweep.layout.dim.append(MultiArrayDimension())
mat_z_sweep.layout.dim[0].label = "height"
mat_z_sweep.layout.dim[1].label = "width"
mat_z_sweep.layout.dim[0].size = 1
mat_z_sweep.layout.dim[1].size = 640
mat_z_sweep.layout.dim[0].stride = 1*640
mat_z_sweep.layout.dim[1].stride =1
mat_z_sweep.layout.data_offset = 0


#data collection
pupilz_list=[]
depthz_list=[]

#allign
align_to=rs.stream.color
align=rs.align(align_to)

#main code
try:
        while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                alligned_frames=align.process(frames)
                depth_frame = alligned_frames.get_depth_frame()
                color_frame = alligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                        continue
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                        images = np.hstack((resized_color_image, depth_colormap))
                else:
                        images = np.hstack((color_image, depth_colormap))
                frame = color_image
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                ### ------ april tags ------- ###

                # define the AprilTags detector options and then detect the AprilTags in the inp
                camera_matrix = np.array([[382.15597, 0, 321.80981], [0, 382.15597, 242.08337], [0, 0, 1]])
                fx = camera_matrix[0][0]
                fy = camera_matrix[1][1]
                cx = camera_matrix[0][2]
                cy = camera_matrix[1][2]
                camera_intrinsics_vector = [fx, fy, cx, cy]

                #bot 2,3
                detector = Detector(families='tag36h11')
                #detector = Detector(families='tag25h9')

                results = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_intrinsics_vector, tag_size=0.123) #.076
                print("[INFO] {} total AprilTags detected".format(len(results)))
                global amount_tags
                amount_tags=(len(results))

                for r in results:
                        mat.data = [0]*15
                        r.pose_R_round=np.matrix.round(r.pose_R,4) #round works, but then publishes the full version
                        pupilz_list.append(r.pose_t[2][0])

                        (X, Y) = (int(r.center[0]), int(r.center[1]))
                        Z=depth_frame.get_distance(X,Y)
                        depthz_list.append(Z)
                        mat.data[12]=Z

                        print("rotation matrix",r.pose_R)
                        print("translation matrix", r.pose_t)
                        print("x,y,z",X,Y,Z)

                        r_count=0
                        for i in range(3):
                                for j in range (3):
                                        mat.data[r_count] = (r.pose_R_round[i][j])
                                        r_count+=1
                        mat.data[9]=r.pose_t[0][0]
                        mat.data[10]=r.pose_t[1][0]
                        mat.data[11]=r.pose_t[2][0]

                if amount_tags==0:
                        mat.data = [0]*15

                mat_z_sweep.data = [0]*640
                for x in range (640):
                        mat_z_sweep.data[x]=depth_frame.get_distance(x,100)

                #publish april location
                pub = rospy.Publisher('april', Float32MultiArray, queue_size=10)
                pub_depth = rospy.Publisher('depth', Float32MultiArray, queue_size=10)

                rospy.init_node('pupil_april')

                pub.publish(mat)
                pub_depth.publish(mat_z_sweep)
                count+=1
                cv2.waitKey(1)

finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()
