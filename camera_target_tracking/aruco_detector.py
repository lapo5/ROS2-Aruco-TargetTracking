#!/usr/bin/env python3

# Libraries
import sys
import json
import numpy as np
import cv2
from cv2 import aruco
from scipy.spatial.transform import Rotation as R
from functools import partial

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

import tf2_ros
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory

# Class definition fo the estimator
class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("camera_module", "hal_camera")
        self.camera_module = self.get_parameter("camera_module").value

        self.declare_parameter("publish_image_feedback", "True")
        self.publish_image_feedback = self.get_parameter("publish_image_feedback").value

        self.declare_parameter("publishers.marker_image", "/target_tracking/marker_image")
        self.marker_image_topic = self.get_parameter("publishers.marker_image").value

        self.declare_parameter("publishers.marker_transform_prefix", "/target_tracking/camera_to_marker_transform/marker_")
        self.marker_pose_topic = self.get_parameter("publishers.marker_transform_prefix").value

        self.declare_parameter("publishers.marker_presence_prefix", "/target_tracking/camera_to_marker_presence/marker_")
        self.marker_presence_topic = self.get_parameter("publishers.marker_presence_prefix").value

        self.declare_parameter("pose_topic_hz", "30")
        self.pose_topic_hz = float(self.get_parameter("pose_topic_hz").value)

        self.declare_parameter("marker_side", "0.1")
        self.marker_side = float(self.get_parameter("marker_side").value)

        self.use_custom_marker_side_dict = False
        self.declare_parameter("custom_marker_side_dict.enable", "True")
        self.use_custom_marker_side_dict = self.get_parameter("custom_marker_side_dict.enable").value

        self.custom_marker_sides = dict()

        if self.use_custom_marker_side_dict:
            self.declare_parameter("custom_marker_side_dict.number_of_entries", "0")
            self.custom_dict_n_ = int(self.get_parameter("custom_marker_side_dict.number_of_entries").value)

            for i in range (0, self.custom_dict_n_):
                entry_name = "custom_marker_side_dict.entry_" + str(i)
                self.declare_parameter(entry_name + ".id", "0")
                m_id = int(self.get_parameter(entry_name + ".id").value)
                
                self.declare_parameter(entry_name + ".marker_side", "0.0")
                m_side = float(self.get_parameter(entry_name + ".marker_side").value)
                self.custom_marker_sides[m_id] = m_side

        self.declare_parameter("subscribers.raw_frame", "/camera/raw_frame")
        self.raw_frame_topic = self.get_parameter("subscribers.raw_frame").value

        self.declare_parameter("client_services.stop_camera", "/camera/get_cam_state")
        self.service_stop_camera = self.get_parameter("client_services.stop_camera").value

        self.declare_parameter("frames.camera_link", "camera_link")
        self.camera_link_frame = self.get_parameter("frames.camera_link").value
        
        self.declare_parameter("frames.marker_link_prefix", "marker_link_")
        self.marker_link_frame_prefix_ = self.get_parameter("frames.marker_link_prefix").value
        
        self.declare_parameter("aruco.dict", "5X5_100")
        self.aruco_dict_name_ = self.get_parameter("aruco.dict").value

        self.declare_parameter("camera_optic_length", "auto")
        self.camera_optic_length = self.get_parameter("camera_optic_length").value

        package_share_directory = get_package_share_directory(self.camera_module)
        if self.camera_optic_length == "auto":
            self.calibration_camera_path = package_share_directory + "/calibration/calib_params.json"
        else:
            self.calibration_camera_path = package_share_directory + "/calibration/calib_params_" + self.camera_optic_length + ".json"


        # Class attributes
        self.cam_params = dict()
        self.get_logger().info("[Aruco Pose Estimator] Uploading intrinsic parameters from " + self.calibration_camera_path)
        self.get_cam_parameters()
        self.frame = []
        self.marker_pose = []

        if self.aruco_dict_name_ == "5X5_100":
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        elif self.aruco_dict_name_ == "5X5_250":
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        elif self.aruco_dict_name_ == "5X5_1000":
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        elif self.aruco_dict_name_ == "7X7_250":
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)
        elif self.aruco_dict_name_ == "7X7_1000":
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
        else:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)

        self.search_for_grid = False
        self.declare_parameter("grid.detect_grid", "False")
        self.search_for_grid = self.get_parameter("grid.detect_grid").value

        self.declare_parameter("grid.number_of_grids", "0")
        self.grid_number = int(self.get_parameter("grid.number_of_grids").value)

        self.grid_output_ids = []
        self.grid_marker_lengths = []
        self.grid_marker_separations = []
        self.grid_rows = []
        self.grid_cols = []
        self.grid_ids = []
        self.grid_overall_ids = set()

        self.boards = []

        if self.search_for_grid:

            for i in range (0, self.grid_number):

                entry_name = "grid.grid_" + str(i) + "."
                self.declare_parameter(entry_name + "output_id", "10")
                grid_output_id = int(self.get_parameter(entry_name + "output_id").value)
                self.grid_output_ids.append(grid_output_id)
                
                self.declare_parameter(entry_name + "marker_length", "0.07")
                grid_marker_length = float(self.get_parameter(entry_name + "marker_length").value)
                self.grid_marker_lengths.append(grid_marker_length)
                
                self.declare_parameter(entry_name + "marker_separation", "0.04")
                grid_marker_separation = float(self.get_parameter(entry_name + "marker_separation").value)
                self.grid_marker_separations.append(grid_marker_separation)
                
                self.declare_parameter(entry_name + "rows", "3")
                grid_rows = int(self.get_parameter(entry_name + "rows").value)
                self.grid_rows.append(grid_rows)
                
                self.declare_parameter(entry_name + "cols", "3")
                grid_cols = int(self.get_parameter(entry_name + "cols").value)
                self.grid_cols.append(grid_cols)
                
                self.declare_parameter(entry_name + "grid_ids", [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
                grid_ids = self.get_parameter(entry_name + "grid_ids").value
                
                self.grid_ids.append(grid_ids)
                self.grid_overall_ids.update(grid_ids)

                board = aruco.GridBoard_create(
                    markersX=self.grid_rows[i], 
                    markersY=self.grid_cols[i], 
                    markerLength=self.grid_marker_lengths[i], 
                    markerSeparation=self.grid_marker_separations[i], 
                    dictionary=self.aruco_dict,
                    firstMarker=grid_ids[0]
                )
                self.boards.append(board)

        self.aruco_params = aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.frame_pub = self.create_publisher(Image, self.marker_image_topic, 1)

        self.frame_sub = self.create_subscription(Image, self.raw_frame_topic, self.callback_frame, 1)

        self.transforms_pub = dict()
        self.presence_pub = dict()

        self.marker_ids_seen = set()
        
        self.br = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("[Aruco Pose Estimator] Node Ready")


    def callback_call_stop_service(self, future, stop_flag):
        try:
            response = future.result()
            self.get_logger().info("[Aruco Pose Estimator] Camera state has been set: " + str(response.cam_state))
        except Exception as e:
            self.get_logger().info("Service call failed %r" %(e,))


    def get_cam_parameters(self):
        with open(self.calibration_camera_path, "r") as readfile:
            self.cam_params = json.load(readfile)

        self.cam_params["mtx"] = np.array(self.cam_params["mtx"], dtype=float).reshape(3, 3)
        self.cam_params["dist"] = np.array(self.cam_params["dist"], dtype=float)


    def callback_frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        if len(frame.shape) == 3:
            self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            self.frame = frame

        self.estimate_pose()


    def estimate_pose(self):
        corners, ids, rejected = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)

        # self.get_logger().warn("ids: {0}".format(ids))
        self.currently_seen_ids = set()
        if ids is not None and len(ids) > 0: 

            if self.publish_image_feedback:
                self.aruco_display(corners, ids)
            
            for (marker_corner, marker_id) in zip(corners, ids):
                
                self.currently_seen_ids.add(int(marker_id[0]))

                marker_side = self.marker_side

                if self.use_custom_marker_side_dict and marker_id[0] in self.custom_marker_sides:
                    marker_side = self.custom_marker_sides[marker_id[0]]

                # Pose estimation for each marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(marker_corner, marker_side, 
                    self.cam_params["mtx"], self.cam_params["dist"])

                if not self.search_for_grid or marker_id[0] not in self.grid_overall_ids:
                    self.publish_pose(marker_id[0], tvec[0][0], rvec[0][0])
                    
            
            if self.search_for_grid:

                for i in range (0, self.grid_number):
                    retval, rvec2, tvec2 = aruco.estimatePoseBoard(corners, ids, self.boards[i], self.cam_params["mtx"], self.cam_params["dist"], rvec, tvec)

                    self.get_logger().info("[Aruco Pose Estimator] retval: {0}".format(retval))
                    if retval > 5:
                        self.currently_seen_ids.add(self.grid_output_ids[i])
                        
                        if tvec2.shape[0] == 3:
                            tvec2_ = [tvec2[0][0], tvec2[1][0], tvec2[2][0]]
                            rvec2_ = [rvec2[0][0], rvec2[1][0], rvec2[2][0]]
                            self.publish_pose(self.grid_output_ids[i], tvec2_, rvec2_)
                        else:
                            self.publish_pose(self.grid_output_ids[i], tvec2[0][0], rvec2[0][0])
        

        for marker_not_seen in self.marker_ids_seen.difference(self.currently_seen_ids):
            presence_msg = Bool()
            presence_msg.data = False
            self.presence_pub[marker_not_seen].publish(presence_msg)


    def publish_pose(self, marker_id, tvec, rvec):

        if not marker_id in self.marker_ids_seen:
            self.marker_ids_seen.add(marker_id)
            marker_pose_topic = self.marker_pose_topic + str(marker_id)
            self.transforms_pub[marker_id] = self.create_publisher(TransformStamped, marker_pose_topic, 1)

            marker_presence_topic = self.marker_presence_topic + str(marker_id)
            self.presence_pub[marker_id] = self.create_publisher(Bool, marker_presence_topic, 1)
        
        msg = TransformStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_link_frame

        marker_link_frame = self.marker_link_frame_prefix_ + str(marker_id)
        msg.child_frame_id = marker_link_frame

        # Translation
        msg.transform.translation.x = tvec[0]
        msg.transform.translation.y = tvec[1]
        msg.transform.translation.z = tvec[2]

        rot = R.from_rotvec([rvec[0], rvec[1], rvec[2]])
        quat = rot.as_quat()

        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]

        # Publish the message
        self.transforms_pub[marker_id].publish(msg)

        presence_msg = Bool()
        presence_msg.data = True
        self.presence_pub[marker_id].publish(presence_msg)
        
        self.br.sendTransform(msg)
        

    def aruco_display(self, corners, ids):
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            self.frame_color = cv2.cvtColor(self.frame, cv2.COLOR_GRAY2BGR)
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(markerCorner, self.marker_side, 
                    self.cam_params["mtx"], self.cam_params["dist"])
                # Draw the axis on the aruco markers
                aruco.drawAxis(self.frame_color, self.cam_params["mtx"], self.cam_params["dist"], rvec, tvec, 0.1)

                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(self.frame_color, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(self.frame_color, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(self.frame_color, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(self.frame_color, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(self.frame_color, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(self.frame_color, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

            self.image_message = self.bridge.cv2_to_imgmsg(self.frame_color, encoding="bgr8")
            self.image_message.header = Header()
            self.image_message.header.stamp = self.get_clock().now().to_msg()
            self.image_message.header.frame_id = self.camera_link_frame
            self.frame_pub.publish(self.image_message)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[Aruco Pose Estimator] stopped cleanly')
    except BaseException:
        node.get_logger().info('[Aruco Pose Estimator] Exception:', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown() 


if __name__ == "__main__":
    main()