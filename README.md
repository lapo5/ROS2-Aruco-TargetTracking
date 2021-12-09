# Aruco Pose Filter

Aruco Detection and Pose Estimation ROS2 Package.

## Description

ROS2 Package to detect the pose of multiple aruco from an input camera stream.
The calibration of the camera must be specified via the params file.
The package contains also a node to simulate a marker, for debugging and unit tests.

## Aruco Detection Params

aruco.dict: which dictionary used for the aruco markers.
marker side: default marker side to consider.
custom_marker_side_dict: to define different sides of aruco markers and detect all in the correct way.
grid: to detect some marker as part of a grid for better performances (multiple grids supported).

## Camera Calibration Params

Must be a json file (standard output oif OpenCv Calibration process), located in <camera_module> install folder (specified by param).
- Case <camera_optic_length> param is 'auto': camera_module/calibration/calib_params.json
- Case <camera_optic_length> param is 'Xmm': camera_module/calibration/calib_params_Xmm.json

## Input/Output

Input: 

- raw_frame: 	camera feed (sensor msgs/Image)

Output: 

For each marker detected:
- Pose: 				marker_transform_prefix + marker_id 			(geometry msgs/Transform Stamped)
- Presence: 				marker_presence_prefix + marker_id				(std msgs/Bool)

## TF2

The node also publish a transform from camera link to (marker_link_prefix + marker_id)

## Depend

ROS2
opencv-python
opencv-contrib-python (aruco)
scipy
