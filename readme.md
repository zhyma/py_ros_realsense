- Need to setup:
	- RealSense (pyrealsense2)
	- OpenCV
	- ROS

- To run the node, use `roslaunch py_ros_realsense realsense.launch`
	- The serial number of the RealSense is in the launch file.
	- The color images are published to `/front_cam/color/raw` (change the camera name in the launch file if needed).
	- The camera intrinsic is published to `/front_cam/color/camera_info`. For a RealSense camera, the intrinsic stays the same all the time.
	- The depth information are published to `/front_cam/depth/raw`.

- For connect to RealSense, publish image to YOUR_CAMERA_NAME/color/raw and depth to YOUR_CAMERA_NAME/depth/raw.
	- To start recording, use `rosservice call /front_cam/start_recording "PATH_TO_SAVE"`
	- To stop recording, use `rosservice call /front_cam/stop_recording`

- To test, run `test.py`. The image is published to `/py_ros_realsense_test/debug`.