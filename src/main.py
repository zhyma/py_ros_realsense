#!/usr/bin/env python3

# Covert raw RealSense Depth data to RViz PointCloud2 data
# Use Pyrealsense2 to obtain RS data, no launch file is needed

import os, sys, copy, time, cv2, threading, pickle
import numpy as np
import rospy
import workspace_tf as ws_tf

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from py_ros_realsense.srv import VideoRecording, VideoRecordingResponse

import pyrealsense2 as rs

class rs_get():
    def __init__(self, serial, alias="", ht = np.eye(4), width = 1280, height = 720):
        '''
        serial: The serial NO. of the RealSense
        alias: name your camera topic
        The default width and height are set to  1280x720
        '''
        if alias=="":
            self.alias = "/rs_" + str(serial)
        else:
            self.alias = alias

        self.height = height
        self.width = width
        self.ht = ht

        self.image_pub = rospy.Publisher(self.alias+"/color/raw", Image, queue_size = 10)
        self.k_pub = rospy.Publisher(self.alias+"/color/camera_info", CameraInfo, queue_size = 10)
        self.depth_pub = rospy.Publisher(self.alias+"/depth/raw", Image, queue_size = 10)
        self.pc_pub = rospy.Publisher(self.alias+"/depth/color/points", PointCloud2, queue_size=1)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(str(serial))
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        self.k = [0.0]*9 # camera's intrinsic parameters
        self.distort = [0.0]*5
        self.get_cam_param()
        print("{} cam param is : {}".format(alias, self.k))
        self.cam_info = CameraInfo()
        self.cam_info.width = self.width
        self.cam_info.height = self.height
        if not np.allclose(self.ht, np.eye(4)):
            self.cam_info.header.frame_id = "world"
        else:
            ## remove "/" in alias to make it a valid frame_id
            self.cam_info.header.frame_id = self.alias.replace("/", "")+"_link"
        self.cam_info.K = self.k
        self.is_data_updated = False

        ## prepare for converting depth information to point cloud
        self.x_scale = (np.arange(self.width, dtype=np.float32)-self.cam_info.K[2])/self.cam_info.K[0] ## (ix-cx)/fx
        self.y_scale = (np.arange(self.height, dtype=np.float32)-self.cam_info.K[5])/self.cam_info.K[4] ## (iy-cy)/fy

        self.recording = False
        self.out = None ## videoWriter handle
        self.lock = threading.Lock()
        # self.start_srv = rospy.Service(self.alias+"/start_recording", Trigger, self.start_recording)
        self.start_srv = rospy.Service(self.alias+"/start_recording", VideoRecording, self.start_recording)
        self.stop_srv  = rospy.Service(self.alias+"/stop_recording",  Trigger, self.stop_recording)
        self.ht_reload_srv = rospy.Service(self.alias+"/reload_ht", Trigger, self.reload_ht)

        ## wait for 1s to maker sure color images arrive
        rospy.sleep(1)
        self.color_img = np.zeros((height, width, 3))
        self.depth_1d = np.zeros((height, width, 3))
        self.pc_msg = None

        data_retrieved = self.get_rgbd()
        while data_retrieved == False:
            print("waiting for data...")
            rospy.sleep(0.03)
            data_retrieved = self.get_rgbd()

    def set_config(self, config):
        ## config = "Default", "High Accuracy", "High Density"
        if config not in ["Default", "High Accuracy", "High Density"]:
            config = "Default"
        ## use preset configuration
        preset_range = self.depth_sensor.get_option_range(rs.option.visual_preset)
        for i in range(int(preset_range.max)):
            visual_preset = self.depth_sensor.get_option_value_description(rs.option.visual_preset, i)
            if visual_preset == config:
                self.depth_sensor.set_option(rs.option.visual_preset, i)

        data_retrieved = False
        while data_retrieved == False:
            print("waiting for data...")
            data_retrieved = self.get_rgbd()

    def get_cam_param(self):
        # st_profile = self.profile.get_stream(rs.stream.depth)
        st_profile = self.profile.get_stream(rs.stream.color)
        self.intr = st_profile.as_video_stream_profile().get_intrinsics()
        self.k[0] = self.intr.fx
        self.k[2] = self.intr.ppx
        self.k[4] = self.intr.fy
        self.k[5] = self.intr.ppy
        self.k[8] = 1.0

        for i in range(5):
            self.distort[i] = self.intr.coeffs[i]

    def get_rgbd(self):
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        try:
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                return -1

            self.color_img = np.asanyarray(color_frame.get_data())
            self.depth_1d = np.asanyarray(aligned_depth_frame.get_data()) ## This is z

            ## pack data into as pointcloud2
            ## need scale? z = self.depth1d 
            z = self.depth_1d * self.depth_scale
            x = self.x_scale[None, :] * z
            y = self.y_scale[:, None] * z
            valid = (z>0.1) & (z<5.0) ## let's say the objects are from 0.1m to 5.0m 
            x[~valid] = np.nan
            y[~valid] = np.nan
            z[~valid] = np.nan
            ## convert to world frame by given self.ht
            self.xyz_cam = np.stack([x, y, z], axis=-1) ## (h,w,3)
            self.xyz_world = self.xyz_cam @ self.ht[:3, :3].T + self.ht[:3, 3]
            x = self.xyz_world[:, :, 0]
            y = self.xyz_world[:, :, 1]
            z = self.xyz_world[:, :, 2]

            ## pack rgb
            r = self.color_img[:,:,2].astype(np.uint32)
            g = self.color_img[:,:,1].astype(np.uint32)
            b = self.color_img[:,:,0].astype(np.uint32)
            rgb = (r<<16)|(g<<8)|b

            ## pack cloud data
            cloud = np.zeros((self.height, self.width), dtype=[('x', np.float32), ('y', np.float32),\
                                                               ('z', np.float32), ('rgb', np.uint32)])
            cloud['x'], cloud['y'], cloud['z'], cloud['rgb'] = x, y, z, rgb
            self.pc_msg = PointCloud2()
            self.pc_msg.header = Header(stamp=rospy.Time.now(), frame_id=self.cam_info.header.frame_id)
            self.pc_msg.height = self.height
            self.pc_msg.width = self.width
            self.pc_msg.fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),\
                          PointField('z', 8, PointField.FLOAT32, 1), PointField('rgb', 12, PointField.UINT32, 1),]
            self.pc_msg.is_bigendian = False
            self.pc_msg.point_step = 16
            self.pc_msg.row_step = self.pc_msg.point_step * self.width
            self.pc_msg.is_dense = False
            self.pc_msg.data = cloud.tobytes()
            
            with self.lock: ## write to video stream
                if self.out is not None:
                    # frame = self.bridge.imgmsg_to_cv2(color_image, "bgr8")
                    # self.out.write(frame)
                    self.out.write(self.color_img)

            return True

        except Exception as e:
            print(e)
            return False

    def pub_data(self):
        img_msg = self.bridge.cv2_to_imgmsg(self.color_img, encoding="bgr8")
        self.image_pub.publish(img_msg)
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_1d)
        self.depth_pub.publish(depth_msg)
        self.k_pub.publish(self.cam_info)
        if self.pc_msg is not None:
            self.pc_pub.publish(self.pc_msg)

    def start_recording(self, req):
        with self.lock:
            if self.recording:
                # return TriggerResponse(success=False, message="Already recording")
                return VideoRecordingResponse(success=False, message="Already recording")

            # fourcc = cv2.VideoWriter_fourcc(*'XVID')
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            
            filename = time.strftime("%m-%d_%H-%M-%S", time.localtime(time.time()))
            if req.path in ["", None]:
                cwd = os.path.dirname(os.path.abspath(__file__))+"/debug/"
            else:
                cwd = req.path + '/'
            rospy.loginfo("Video saves to: "+cwd+filename+".avi")
            self.out = cv2.VideoWriter(cwd+filename+".avi", fourcc, 30.0, (self.width,self.height))
            self.recording = True

        rospy.loginfo("Recording started.")
        # return TriggerResponse(success=True, message="Recording started.")
        return VideoRecordingResponse(success=True, message="Recording started.")

    def stop_recording(self, req):
        with self.lock:
            if not self.recording:
                # return TriggerResponse(success=False, message="Not recording.")
                return VideoRecordingResponse(success=False, message="Not recording.")
            self.recording = False
            self.out.release()
            self.out = None

        rospy.loginfo("Recording stopped.")
        return TriggerResponse(success=True, message="Recording stopped.")
        # return VideoRecordingResponse(success=True, message="Recording stopped.")

    def reload_ht(self, req):
        with self.lock:
            ht_filename = rospy.get_param("~ht_file", None)
            if ht_filename not in ["", None] and os.path.isfile(ht_filename):
                with open(ht_filename, "rb") as handle:
                    data = pickle.load(handle)
                if isinstance(data, list) and len(data) > 0:
                    self.ht = data[0]
                    rospy.loginfo("new Camera transformation to the world reloaded: \n"+str(self.ht))
                    return TriggerResponse(success=True, message="new HT reloaded.")
            rospy.logwarn("Failed to reload HT! Check the file path and content.")
            return TriggerResponse(success=False, message="Failed to reload HT! Check the file path and content.")


if __name__ == '__main__':
    print(cv2.__version__)
    rospy.init_node("realsense", anonymous = True)
    np.set_printoptions(suppress=True)

    serial_no = rospy.get_param("~rs_serial_no", None)
    ht_filename = rospy.get_param("~ht_file", None)
    cam_ht = None

    if ht_filename not in ["", None] and os.path.isfile(ht_filename):
        with open(ht_filename, "rb") as handle:
            data = pickle.load(handle)
        if isinstance(data, list) and len(data) > 0:
            cam_ht = data[0]
            print("Get Camera transformation to the world: \n", cam_ht)

    if cam_ht is None:
        cam_ht = np.eye(4)
        print("Get Camera transformation to the world: \n", cam_ht)
    
    if serial_no is None:
        rospy.logwarn("NO serial number IS PROVIDED!")
        serial_no = "851112063978"
    else:
        print("Get serial number from parameters: "+serial_no)

    alias = rospy.get_name()
    tf = ws_tf.workspace_tf()
    tf.add_static_tf("world", alias.replace("/", "")+"_link", cam_ht)
    front_cam = rs_get(serial_no, alias=alias, ht=cam_ht)

    rospy.sleep(1)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        data_retrieved = front_cam.get_rgbd()
        front_cam.pub_data()
        rate.sleep()
