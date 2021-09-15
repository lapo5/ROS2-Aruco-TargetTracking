#!usr/bin/venv python3

# Libraries
import rclpy
from rclpy.node import Node
from interfaces.msg import Pose
from scipy.spatial.transform import Rotation as R
import numpy as np
import json

# File address
FILE_PATH = "./src/camera/camera/cam_tcp_h.json"



# Pose filter class definition
class PoseFilterNode(Node):
	def __init__(self):
		super().__init__("pose_filter_node")
		self.get_logger().info("Pose filter node is awake...")

		self.cam_goal = []
		self.h_tc = self.get_h_mtx()

		# Subsciption of the marker pose as rvec (rodrigues) and tvec (x, y, z)
		self.pose_sub = self.create_subscription(Pose, "raw_pose", self.callback_pose, 10)

		# Publish the TCP-based pose
		self.tcp_goal_pub = self.create_publisher(Pose, "filt_pose", 10)
		self.tcp_goal_timer = self.create_timer(0.3, self.publish_tcp_goal)



	# This function uplaod the Camera-TCP Homography from a JSON file
	def get_h_mtx(self):
		with open(FILE_PATH, "r") as readfile:
			return np.array(json.load(readfile)["cam_tcp"], dtype=float)
		self.get_logger().info("Camera-TCP matrix has been uploaded correctly.")


	# Callback function from subscription
	def callback_pose(self, msg):
		
		in_view = msg.in_view;
		translation = [msg.x * 1000, msg.y * 1000, msg.z * 1000] # Translation in [mm]
		rvec = [msg.r1, msg.r2, msg.r3]
		r = R.from_rotvec(np.array(rvec, dtype=float))
		attitude = r.as_euler('xyz', degrees=True)				 # Attitude in [deg]


		self.cam_goal = [translation, attitude, in_view]

	# Publish the transformed set of coordinates
	def publish_tcp_goal(self):

		# If the first frame has not been processed do not publish
		if len(self.cam_goal) == 0:
			return

		# If the first goal has not been detected do not publish
		if not self.cam_goal[2]:
			return

		# Transform the target frame wrt cam in a rotation matrix
		R_cm = R.from_euler("xyz", np.array(self.cam_goal[1], dtype=float), degrees=True).as_matrix()

		# Build H_cm
		H_cm_star = np.hstack((R_cm, np.array(self.cam_goal[0], dtype=float).reshape(3, 1)))
		H_cm = np.vstack((H_cm_star, np.array([0, 0, 0, 1], dtype=float)))
		
		# Pre-multiply for the TCP-Cam rotation
		H_tm = np.matmul(self.h_tc, H_cm)
		
		# Get the goal in the TCP frame
		translation = H_tm[:3, 3]
		euler = R.from_matrix(H_tm[:3, :3]).as_euler("xyz", degrees=True)

		# Publish the message
		msg = Pose()

		msg.x = translation[0]
		msg.y = translation[1]
		msg.z = translation[2]

		msg.r1 = euler[0]
		msg.r2 = euler[1]
		msg.r3 = euler[2]

		msg.in_view = True

		self.tcp_goal_pub.publish(msg)



# Main loop function
def main(args=None):
	rclpy.init(args=args)
	node = PoseFilterNode()
	rclpy.spin(node)
	rclpy.shutdown()


# Main loop
if __name__ == '__main__':
	main()
