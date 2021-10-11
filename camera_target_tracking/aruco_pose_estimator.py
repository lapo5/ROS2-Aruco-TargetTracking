#!/usr/bin/env python3

# Libraries
import sys
import json
import numpy as np
import cv2
from cv2 import aruco
from scipy.spatial.transform import Rotation as R

import threading
from functools import partial

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

import tf2_ros
import geometry_msgs
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from allied_vision_camera_interfaces.srv import CameraState

from ament_index_python.packages import get_package_share_directory


# Class definition fo the estimator
class ArucoPoseNode(Node):
	def __init__(self):
		super().__init__("aruco_pose_estimator")
		self.get_logger().info("Marker estimator node is awake...")

		self.declare_parameter("camera_module", "hal_allied_vision_camera")
		self.camera_module = self.get_parameter("camera_module").value

		self.declare_parameter("publishers.marker_image", "/target_tracking/marker_image")
		self.marker_image_topic = self.get_parameter("publishers.marker_image").value

		self.declare_parameter("publishers.marker_pose", "/target_tracking/camera_to_marker_pose")
		self.marker_pose_topic = self.get_parameter("publishers.marker_pose").value

		self.declare_parameter("pose_topic_hz", "30")
		self.pose_topic_hz = float(self.get_parameter("pose_topic_hz").value)

		self.declare_parameter("marker_side", "0.1")
		self.marker_side = float(self.get_parameter("marker_side").value)

		self.declare_parameter("subscribers.raw_frame", "/parking_camera/raw_frame")
		self.raw_frame_topic = self.get_parameter("subscribers.raw_frame").value

		self.declare_parameter("client_services.stop_camera", "/parking_camera/get_cam_state")
		self.service_stop_camera = self.get_parameter("client_services.stop_camera").value

		self.declare_parameter("frames.camera_link", "parking_camera_link")
		self.camera_link_frame = self.get_parameter("frames.camera_link").value

		self.declare_parameter("frames.marker_link", "marker_link")
		self.marker_link_frame = self.get_parameter("frames.marker_link").value

		package_share_directory = get_package_share_directory(self.camera_module)
		# Path to store the calibration file
		self.calibration_camera_path = package_share_directory + "/calibration/calib_params.json"

		# Class attributes
		self.cam_params = dict()
		self.get_logger().info("Uploading intrinsic parameters from " + self.calibration_camera_path)
		self.get_cam_parameters()
		self.get_logger().info("Parameters successfully uploaded.")
		self.frame = []
		self.marker_pose = []
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
		self.aruco_params = aruco.DetectorParameters_create()
		self.bridge = CvBridge()

		self.frame_pub = self.create_publisher(Image, self.marker_image_topic, 1)

		# Subscription
		self.frame_sub = self.create_subscription(Image, self.raw_frame_topic, self.callback_frame, 1)

		# Publishers
		self.pose_pub = self.create_publisher(PoseStamped, self.marker_pose_topic, 10)

		self.br = tf2_ros.TransformBroadcaster(self)


		self.get_logger().info("Marker estimator node ready")


	# Destructor function: call the stop service and disarm the camera regularly
	def clean_exit(self):
		self.callback_stop_service(False)


	# This function is a client which asks the camera to shutdown when the node is killed
	def  callback_stop_service(self, stop_flag):
		client = self.create_client(CameraState, self.service_stop_camera)
		while not client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for server response...")

		request = CameraState.Request()
		request.command_state = stop_flag

		future = client.call_async(request)
		future.add_done_callback(partial(self.callback_call_stop_service, stop_flag=stop_flag))

	# This function is a callback to the client future
	def callback_call_stop_service(self, future, stop_flag):
		try:
			response = future.result()
			self.get_logger().info("Camera state has been set: " + str(response.cam_state))
		except Exception as e:
			self.get_logger().info("Service call failed %r" %(e,))

	# This function publish the pose information from each frame
	def publish_pose(self):
		msg = PoseStamped()

		msg.header = Header()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = self.camera_link_frame

		# Translation
		msg.pose.position.x = self.marker_pose[0][0][0][0]
		msg.pose.position.y = self.marker_pose[0][0][0][1]
		msg.pose.position.z = self.marker_pose[0][0][0][2]

		rot = R.from_rotvec([self.marker_pose[1][0][0][0], self.marker_pose[1][0][0][1], self.marker_pose[1][0][0][2]])
		quat = rot.as_quat()

		# short-Rodrigues (angle-axis)
		msg.pose.orientation.x = quat[0]
		msg.pose.orientation.y = quat[1]
		msg.pose.orientation.z = quat[2]
		msg.pose.orientation.w = quat[3]

		# Publish the message
		self.pose_pub.publish(msg)

		self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
		self.image_message.header = Header()
		self.image_message.header.stamp = self.get_clock().now().to_msg()
		self.image_message.header.frame_id = self.camera_link_frame
		self.frame_pub.publish(self.image_message)

		# Publish Frame
		t = geometry_msgs.msg.TransformStamped()

		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = "wrist_camera_link" # To
		t.child_frame_id = "marker_link" # From
		t.transform.translation.x = msg.pose.position.x
		t.transform.translation.y = msg.pose.position.y
		t.transform.translation.z = msg.pose.position.z
		t.transform.rotation.x = msg.pose.orientation.x
		t.transform.rotation.y = msg.pose.orientation.y
		t.transform.rotation.z = msg.pose.orientation.z
		t.transform.rotation.w = msg.pose.orientation.w

		self.br.sendTransform(t)
		

	# This function upload from JSON the intrinsic camera parameters k_mtx and dist_coeff
	def get_cam_parameters(self):
		with open(self.calibration_camera_path, "r") as readfile:
			self.cam_params = json.load(readfile)

		self.cam_params["mtx"] = np.array(self.cam_params["mtx"], dtype=float).reshape(3, 3)
		self.cam_params["dist"] = np.array(self.cam_params["dist"], dtype=float)


	# This function store the received frame in a class attribute
	def callback_frame(self, msg):
		frame = self.bridge.imgmsg_to_cv2(msg)
		if len(frame.shape) == 3:
			self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		else:
			self.frame = frame

		self.estimate_pose()


	# This function detect and estimate the marker pose wrt the camera frame
	def estimate_pose(self):

		# If the 1st frame has not been received yet, return
		if len(self.frame) != 0:

			corners, ids, _ = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)

			# If there are no ids, jump in order not to make the code crashes
			if np.all(ids != None):

				# Pose estimation for each marker
				rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_side, 
					self.cam_params["mtx"], self.cam_params["dist"])

				# (!!!) This line makes sense only if we use a single marker detection
				self.marker_pose = [tvec, rvec, True]
				self.publish_pose()

				# Draw the axis on the aruco markers
				for i in range(0, ids.size):
					aruco.drawAxis(self.frame, self.cam_params["mtx"], self.cam_params["dist"], rvec[i], tvec[i], 0.1)

				# Draw a square on the markers perimeter
				aruco.drawDetectedMarkers(self.frame, corners)


			else:
				self.marker_pose = [0, 0, False]


# Main loop function
def main(args=None):
	rclpy.init(args=args)
	node = ArucoPoseNode()
	
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('ARUCO Detector Node stopped cleanly')
		
	except BaseException:
		print('Exception in ARUCO Detector:', file=sys.stderr)
		raise
	finally:
		# Destroy the node explicitly
		# (optional - Done automatically when node is garbage collected)
		#node.clean_exit()
		rclpy.shutdown() 


# Main
if __name__ == '__main__':
	main()