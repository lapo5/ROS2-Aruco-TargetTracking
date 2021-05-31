#!/usr/bin/env python3

# Libraries
import cv2
from cv2 import aruco
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import threading
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from allied_vision_camera_interfaces.msg import Pose
from allied_vision_camera_interfaces.srv import CameraState
from flir_ptu_d46_interfaces.msg import PTU
from functools import partial
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import math



# Class definition fo the estimator
class PTU_to_Rover_Transoformer(Node):
	def __init__(self):
		super().__init__("camera_to_ptu_base")

		# Subscription
		self.frame_sub = self.create_subscription(PoseStamped, "/target_tracking/camera_to_marker_pose", self.callback_frame, 10)

		self.ptu_sub = self.create_subscription(PTU, "/PTU/Flir_D46/state", self.callback_ptu, 1)

		# Publishers
		self.pose_pub = self.create_publisher(PoseStamped, "/target_tracking/ptu_to_marker_pose", 10)

		self.pan_ptu = 0.0
		self.tilt_ptu = 0.0

		self.tilt_T_baseline_cam = np.eye(4, dtype=np.float32)
		self.tilt_T_baseline_cam[0, 3] = 0.0
		self.tilt_T_baseline_cam[1, 3] = 0.0
		self.tilt_T_baseline_cam[2, 3] = 0.0



	# This function store the received frame in a class attribute
	def callback_frame(self, msg):
		
		baseline_cam_T_marker = np.eye(4, dtype=np.float32)

		rot = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		baseline_cam_T_marker[0:3, 0:3] = rot.as_matrix()
		baseline_cam_T_marker[0, 3] = msg.pose.position.x
		baseline_cam_T_marker[1, 3] = msg.pose.position.y
		baseline_cam_T_marker[2, 3] = msg.pose.position.z

		base_ptu_T_pan = np.eye(4, dtype=np.float32)
		base_ptu_T_pan = [[math.cos(self.pan_ptu), -math.sin(self.pan_ptu), 0, 0], [math.sin(self.pan_ptu), math.cos(self.pan_ptu), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

		pan_T_tilt = np.eye(4, dtype=np.float32)
		pan_T_tilt = [[math.cos(- self.tilt_ptu), 0, math.sin(- self.tilt_ptu), 0], [0, 1, 0, 0], [- math.sin( - self.tilt_ptu), 0, math.cos(- self.tilt_ptu), 0], [0, 0, 0, 1]]

		pan_T_baseline_cam = np.matmul(pan_T_tilt, self.tilt_T_baseline_cam)

		base_ptu_T_baseline_cam = np.matmul(base_ptu_T_pan, pan_T_baseline_cam)

		base_ptu_T_marker = np.matmul(base_ptu_T_baseline_cam, baseline_cam_T_marker)

		new_msg = PoseStamped()
			
		new_msg.header = Header()
		new_msg.header.stamp = self.get_clock().now().to_msg()
		new_msg.header.frame_id = "PTU_Base"
		
		# Translation
		new_msg.pose.position.x = float(base_ptu_T_marker[0, 3])
		new_msg.pose.position.y = float(base_ptu_T_marker[1, 3])
		new_msg.pose.position.z = float(base_ptu_T_marker[2, 3])

		rot = R.from_matrix(base_ptu_T_marker[0:3, 0:3])
		quat = rot.as_quat()

		new_msg.pose.orientation.x = quat[0]
		new_msg.pose.orientation.y = quat[1]
		new_msg.pose.orientation.z = quat[2]
		new_msg.pose.orientation.w = quat[3]

		# Publish the message
		self.pose_pub.publish(new_msg)

	# This function store the received frame in a class attribute


	def callback_ptu(self, msg):
		
		self.pan_ptu = msg.pan

		self.tilt_ptu = msg.tilt




# Main loop function
def main(args=None):
	rclpy.init(args=args)
	node = PTU_to_Rover_Transoformer()
	rclpy.spin(node)
	rclpy.shutdown()


# Main
if __name__ == '__main__':
	main()