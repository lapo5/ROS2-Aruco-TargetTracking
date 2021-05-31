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
from functools import partial
import json
import numpy as np
from scipy.spatial.transform import Rotation as R



# Class definition fo the estimator
class PTU_to_Rover_Transoformer(Node):
	def __init__(self):
		super().__init__("ptu_to_robot_transform")

		# Subscription
		self.frame_sub = self.create_subscription(PoseStamped, "/target_tracking/ptu_to_marker_pose", self.callback_frame, 10)

		# Publishers
		self.pose_pub = self.create_publisher(PoseStamped, "/target_tracking/rover_to_marker", 10)

		self.robot_T_base_PTU = np.eye(4, dtype=np.float32)
		self.robot_T_base_PTU[0, 3] = 0.0
		self.robot_T_base_PTU[1, 3] = 0.0
		self.robot_T_base_PTU[2, 3] = 0.0


	# This function store the received frame in a class attribute
	def callback_frame(self, msg):
		
		base_ptu_T_marker = np.eye(4, dtype=np.float32)

		rot = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		base_ptu_T_marker[0:3, 0:3] = rot.as_matrix()
		base_ptu_T_marker[0, 3] = msg.pose.position.x
		base_ptu_T_marker[1, 3] = msg.pose.position.y
		base_ptu_T_marker[2, 3] = msg.pose.position.z

		robot_T_marker = np.matmul(self.robot_T_base_PTU, base_ptu_T_marker)

		new_msg = PoseStamped()
			
		new_msg.header = Header()
		new_msg.header.stamp = self.get_clock().now().to_msg()
		new_msg.header.frame_id = "Rover_CoM"
		
		# Translation
		new_msg.pose.position.x = float(robot_T_marker[0, 3]) 
		new_msg.pose.position.y = float(robot_T_marker[1, 3])
		new_msg.pose.position.z = float(robot_T_marker[2, 3])

		rot = R.from_matrix(robot_T_marker[0:3, 0:3])
		quat = rot.as_quat()

		# short-Rodrigues (angle-axis)
		new_msg.pose.orientation.x = quat[0]
		new_msg.pose.orientation.y = quat[1]
		new_msg.pose.orientation.z = quat[2]
		new_msg.pose.orientation.w = quat[3]

		# Publish the message
		self.pose_pub.publish(new_msg)




# Main loop function
def main(args=None):
	rclpy.init(args=args)
	node = PTU_to_Rover_Transoformer()
	rclpy.spin(node)
	rclpy.shutdown()


# Main
if __name__ == '__main__':
	main()