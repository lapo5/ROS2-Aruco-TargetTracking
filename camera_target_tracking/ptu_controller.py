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
from flir_ptu_d46_interfaces.srv import SetPanTiltSpeed, SetPanTilt
from functools import partial
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


# Class definition fo the estimator
class PTU_to_Rover_Transoformer(Node):
	def __init__(self):
		super().__init__("ptu_controller")

		# Subscription
		self.frame_sub = self.create_subscription(PoseStamped, "/target_tracking/ptu_to_marker_pose", self.callback_frame, 10)

		# Clients
		self.client_ptu_speed = self.create_client(SetPanTiltSpeed, '/PTU/Flir_D46/set_pan_tilt_speed')
		while not self.client_ptu_speed.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service SetPanTiltSpeed not available, waiting again...')

		self.client_ptu = self.create_client(SetPanTilt, '/PTU/Flir_D46/set_pan_tilt')
		while not self.client_ptu.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service SetPanTilt not available, waiting again...')

		self.req_ptu_speed = SetPanTiltSpeed.Request()
		self.req_ptu_speed.pan_speed = 0.5
		self.req_ptu_speed.tilt_speed = 0.2

		self.send_request_ptu_speed()

		self.req_ptu_pos = SetPanTilt.Request()
		self.req_ptu_pos.pan = 0.0
		self.req_ptu_pos.tilt = 0.0

		self.discretization = 10

		self.get_logger().info('PTU Controller Ready...')


	def send_request_ptu_speed(self):
		self.future = self.client_ptu_speed.call_async(self.req_ptu_speed)


	def send_request_ptu_pos(self):
		self.future = self.client_ptu.call_async(self.req_ptu_pos)


	# This function store the received frame in a class attribute
	def callback_frame(self, msg):
		
		base_ptu_T_marker = np.eye(4, dtype=np.float32)

		rot = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		base_ptu_T_marker[0:3, 0:3] = rot.as_matrix()
		base_ptu_T_marker[0, 3] = msg.pose.position.x
		base_ptu_T_marker[1, 3] = msg.pose.position.y
		base_ptu_T_marker[2, 3] = msg.pose.position.z


		pan = math.atan2(-(msg.pose.position.x + 0.260), msg.pose.position.z)
		tilt = math.atan2(-(msg.pose.position.y + 0.160), msg.pose.position.z)

		pan_discret = int(pan / self.discretization)
		if (pan % self.discretization) > self.discretization / 2.0:
			pan_discret = pan_discret + 1

		self.req_ptu_pos.pan = pan_discret * self.discretization


		tilt_discret = int(tilt / self.discretization)
		if (tilt % self.discretization) > self.discretization / 2.0:
			tilt_discret = tilt_discret + 1
		self.req_ptu_pos.tilt = tilt_discret * self.discretization

		self.send_request_ptu_pos()
		self.get_logger().info('PTU Controller Sending PTU Pose...')



# Main loop function
def main(args=None):
	rclpy.init(args=args)
	node = PTU_to_Rover_Transoformer()
	rclpy.spin(node)
	rclpy.shutdown()


# Main
if __name__ == '__main__':
	main()