# The main perception code, the pose of the robot should be involved to help to find out the regions of interest.

import sys, copy, time, cv2, rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs

class Camera():
    def __init__(self):
        cam_info = rospy.wait_for_message("/front_cam/color/camera_info", CameraInfo, timeout=3.0)
        self.k = cam_info.K
        self.distort = [0.0]*5 ## I don't think we need realsense distortion model here
        self.height = cam_info.width
        self.width = cam_info.height
        ## camera extrinsic: homogeneous transformation matrix from the camera (obj) to the world (ref)
        self.ht = np.zeros([4,4])
        self.bridge = CvBridge()

        self.intr = rs.pyrealsense2.intrinsics() ## construct a intrinsics for 3D <-> 2D projection
        self.intr.fx = self.k[0]
        self.intr.ppx = self.k[2]
        self.intr.fy = self.k[4]
        self.intr.ppy = self.k[5]
        self.intr.coeffs = [0, 0, 0, 0, 0]
        self.intr.width = self.width
        self.intr.height = self.height
        self.intr.model = rs.distortion.brown_conrady

        self.color_raw = np.zeros((self.height, self.width, 3))
        self.img_pub = rospy.Publisher("/py_ros_realsense_test/debug", Image, queue_size = 10)
 
    def get_rgbd(self):
        try:
            msg = rospy.wait_for_message("/front_cam/color/raw", Image, timeout=1)
            self.color_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except rospy.ROSException:
            print("Timeout! No image received.")
            exit()
    
    def proj_to_2d(self, pt_3d):
        ## An image is distroed, and has been calibrated according to a variation of the Brown-Conrady Distortion model.
        ## This model provides a closed-form formula to map from undistored points to distorted points, 
        ## while mapping in the other direction requires iteration or lookup tables. 
        ## Therefore, images with Modified Brown-Conrady Distortion are being undistorted when calling rs2_project_point_to_pixel(...).
        ## This model is used by the Intel RealSense D415's color image stream.
        ## Quote from [Projection in Intel RealSense SDK 2.0]()
        pt_3d_cam = np.dot(np.linalg.inv(self.ht), np.append(pt_3d,1))[:3]
        px = rs.rs2_project_point_to_pixel(self.intr, pt_3d_cam)

        # ## project 3d to 2d using the camera's intrisic matrix
        # ## [u,v,1] = (1/z)*(k*[x,y,z]^T)
        # k = np.array(self.k).reshape((3,3))
        # px = np.dot(k, pt_3d.reshape((3,1)))
        # # px = (int(px[0,0]), int(px[1,0]))
        # px = (int(px[0,0]/pt_3d[2]), int(px[1,0]/pt_3d[2]))

        return (int(px[0]), int(px[1])) ## RealSense gives out two floats

    def pub_img(self):
        if self.color_raw is not None:
            img_msg = self.bridge.cv2_to_imgmsg(self.color_raw)
            self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rospy.init_node("py_ros_realsense_test", anonymous=True)
    rospy.sleep(1)
    cam = Camera()
    print(cam.intr)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        cam.get_rgbd()
        cam.pub_img()
        rate.sleep()
