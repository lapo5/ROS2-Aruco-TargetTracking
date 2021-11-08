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
from tf2_ros import TransformException
from rclpy.duration import Duration
import geometry_msgs
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
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

        self.declare_parameter("publishers.marker_transform_prefix", "/target_tracking/camera_to_marker_transform")
        self.marker_pose_topic = self.get_parameter("publishers.marker_transform_prefix").value

        self.declare_parameter("publishers.marker_presence_prefix", "/target_tracking/camera_to_marker_presence")
        self.marker_presence_topic = self.get_parameter("publishers.marker_presence_prefix").value

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
        
        self.declare_parameter("aruco.dict", "5X5_250")
        self.aruco_dict_name_ = self.get_parameter("aruco.dict").value

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

        if self.aruco_dict_name_ == "5X5_250":
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

        self.declare_parameter("grid.output_id", "10")
        self.grid_output_id = int(self.get_parameter("grid.output_id").value)

        self.declare_parameter("grid.grid_ids", "[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]")
        self.grid_ids = self.get_parameter("grid.grid_ids").value


        self.aruco_params = aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.frame_pub = self.create_publisher(Image, self.marker_image_topic, 1)

        # Subscription
        self.frame_sub = self.create_subscription(Image, self.raw_frame_topic, self.callback_frame, 1)

        # Publishers
        self.transforms_pub = dict()
        self.presence_pub = dict()

        self.marker_ids_seen = set()

        self.board = aruco.GridBoard_create(
            markersX=3, 
            markersY=3, 
            markerLength=0.08, 
            markerSeparation=0.04, 
            dictionary=self.aruco_dict)

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

        corners, ids, rejected = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)
        
        self.currently_seen_ids = set()
        if ids is not None and len(ids) > 0: 

            self.aruco_display(corners, ids)
            
            for (marker_corner, marker_id) in zip(corners, ids):
                
                self.currently_seen_ids.add(marker_id[0])

                # Pose estimation for each marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(marker_corner, self.marker_side, 
                    self.cam_params["mtx"], self.cam_params["dist"])

                if not self.search_for_grid or marker_id[0] not in self.grid_ids:
                    self.publish_pose(marker_id[0], tvec[0][0], rvec[0][0])
                else:
                    #self.get_logger().warn("Skipping id: {0}".format(marker_id[0]))
                    pass
            
            if self.search_for_grid:
                retval, rvec2, tvec2 = aruco.estimatePoseBoard(corners, ids, self.board, self.cam_params["mtx"], self.cam_params["dist"], rvec, tvec)

                if retval > 0:
                    self.currently_seen_ids.add(self.grid_output_id)
                    
                    if tvec2.shape[0] == 3:
                        tvec2_ = [tvec2[0][0], tvec2[1][0], tvec2[2][0]]
                        rvec2_ = [rvec2[0][0], rvec2[1][0], rvec2[2][0]]
                        self.publish_pose(self.grid_output_id, tvec2_, rvec2_)
                    else:
                        self.publish_pose(self.grid_output_id, tvec2[0][0], rvec2[0][0])
        

        for marker_not_seen in self.marker_ids_seen.difference(self.currently_seen_ids):
            presence_msg = Bool()
            presence_msg.data = False
            self.presence_pub[marker_not_seen].publish(presence_msg)


    # This function publish the pose information from each frame
    def publish_pose(self, marker_id, tvec, rvec):

        if not marker_id in self.marker_ids_seen:
            self.marker_ids_seen.add(marker_id)
            marker_pose_topic = self.marker_pose_topic + "/marker_" + str(marker_id)
            self.transforms_pub[marker_id] = self.create_publisher(TransformStamped, marker_pose_topic, 1)

            marker_presence_topic = self.marker_presence_topic + "/marker_" + str(marker_id)
            self.presence_pub[marker_id] = self.create_publisher(Bool, marker_presence_topic, 1)
        
        msg = TransformStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_link_frame

        marker_link_frame = self.marker_link_frame + "_" + str(marker_id)
        msg.child_frame_id = marker_link_frame

        # Translation
        msg.transform.translation.x = tvec[0]
        msg.transform.translation.y = tvec[1]
        msg.transform.translation.z = tvec[2]

        rot = R.from_rotvec([rvec[0], rvec[1], rvec[2]])
        quat = rot.as_quat()

        # short-Rodrigues (angle-axis)
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