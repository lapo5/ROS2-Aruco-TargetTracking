#!/usr/bin/env python3

# Libraries
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
from functools import partial

import rclpy
from rclpy.node import Node

import tf2_ros
import geometry_msgs
from std_msgs.msg import Header, Bool
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped


# Class definition fo the estimator
class FakeMarkerPose(Node):
    def __init__(self):
        super().__init__("publish_fake_marker")

        self.declare_parameter("pose_topic_hz", "30")
        self.pose_topic_hz = float(self.get_parameter("pose_topic_hz").value)
        
        self.declare_parameter("clients.trigger_pub", "/sim_marker/trigger_pub")
        self.trigger_pub_serv_name = self.get_parameter("clients.trigger_pub").value
        
        self.declare_parameter("publishers.marker_transform", "/target_tracking/camera_to_marker_transform/marker_69")
        self.marker_pose_topic = self.get_parameter("publishers.marker_transform").value

        self.declare_parameter("publishers.marker_presence", "/target_tracking/camera_to_marker_presence/marker_69")
        self.marker_presence_topic = self.get_parameter("publishers.marker_presence").value

        self.declare_parameter("frames.world_frame", "world")
        self.world_frame = self.get_parameter("frames.world_frame").value

        self.declare_parameter("frames.link_base", "link_base")
        self.link_base = self.get_parameter("frames.link_base").value
        
        self.declare_parameter("frames.camera_link", "wrist_camera_link")
        self.camera_link = self.get_parameter("frames.camera_link").value

        self.declare_parameter("frames.marker_link", "marker_link_69")
        self.marker_link = self.get_parameter("frames.marker_link").value

        self.pose_pub = self.create_publisher(TransformStamped, self.marker_pose_topic, 1)
        self.presence_pub = self.create_publisher(Bool, self.marker_presence_topic, 1)

        self.publishing = True
        
        self.marker_pose_msg = TransformStamped()
        
        self.marker_pose_msg.transform.translation.x = 1.0
        self.marker_pose_msg.transform.translation.y = 0.8
        self.marker_pose_msg.transform.translation.z = 0.0

        rot = R.from_euler('ZYX', [-90.0, 0.0, 90.0], degrees=True)
        self.static_R = rot.as_matrix()

        self.link_base_pose_msg = TransformStamped()
        
        self.link_base_pose_msg.transform.translation.x = 0.0
        self.link_base_pose_msg.transform.translation.y = 0.0
        self.link_base_pose_msg.transform.translation.z = 0.0

        rot = R.from_euler('ZYX', [0.0, 0.0, 0.0], degrees=True)
        quat = rot.as_quat()
        self.link_base_pose_msg.transform.rotation.x = quat[0]
        self.link_base_pose_msg.transform.rotation.y = quat[1]
        self.link_base_pose_msg.transform.rotation.z = quat[2]
        self.link_base_pose_msg.transform.rotation.w = quat[3]
        
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

         # Static Transform
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.t1 = geometry_msgs.msg.TransformStamped()
        self.t1.header.stamp = self.get_clock().now().to_msg()
        self.t1.header.frame_id = self.world_frame
        self.t1.child_frame_id = self.link_base
        self.t1.transform.translation.x =  0.0
        self.t1.transform.translation.y = 0.0
        self.t1.transform.translation.z = 0.0
        rot = R.from_euler('zyx', [0.0, 0.0, 0.0], degrees=True)
        quat = rot.as_quat()
        self.t1.transform.rotation.x = quat[0]
        self.t1.transform.rotation.y = quat[1]
        self.t1.transform.rotation.z = quat[2]
        self.t1.transform.rotation.w = quat[3]
        self.static_broadcaster.sendTransform(self.t1)


        self.current_ypr = [0.0, 0.0, 0.0]

        ###########################     PARAMS for SIM          ###########################
        # mm/sec
        self.velocity = [0.01, 0.0, 0.0]
        # Degrees/sec
        self.angular_velocity_ypr = [0.0, 0.0, 0.0]
        self.std_dev_ang_noise_ypr = [0.0, 0.0, 0.0]

        # mm/sec
        self.velocity_link_base = [0.0, 0.0, 0.0]
        ###########################     END PARAMS for SIM      ###########################

        self.pose_timer = self.create_timer(1.0/self.pose_topic_hz, self.publish_pose)

        # Service: start Sim
        self.start_service = self.create_service(Empty, self.trigger_pub_serv_name, self.trigger_pub)

    
    
    # This function stops/enable the acquisition stream
    def trigger_pub(self, request, response):
        self.get_logger().info('Start Publishing Marker Pose')

        self.publishing = not self.publishing

        return response


    # This function publish the pose information from each frame
    def publish_pose(self):
        if self.publishing:
        
            self.marker_pose_msg.header = Header()
            self.marker_pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.marker_pose_msg.header.frame_id = self.camera_link
            self.marker_pose_msg.child_frame_id = self.marker_link

            self.marker_pose_msg.transform.translation.x = self.marker_pose_msg.transform.translation.x + self.velocity[0] / self.pose_topic_hz
            self.marker_pose_msg.transform.translation.y = self.marker_pose_msg.transform.translation.y + self.velocity[1] / self.pose_topic_hz
            self.marker_pose_msg.transform.translation.z = self.marker_pose_msg.transform.translation.z + self.velocity[2] / self.pose_topic_hz

            self.current_ypr[0] = self.current_ypr[0] + + self.angular_velocity_ypr[0] / self.pose_topic_hz
            self.current_ypr[1] = self.current_ypr[1] + + self.angular_velocity_ypr[1] / self.pose_topic_hz
            self.current_ypr[2] = self.current_ypr[2] + + self.angular_velocity_ypr[2] / self.pose_topic_hz
            
            self.noise_on_yaw = np.random.normal(0, self.std_dev_ang_noise_ypr[0] / 3.0, 1)[0]
            self.noise_on_pitch = np.random.normal(0, self.std_dev_ang_noise_ypr[1] / 3.0, 1)[0]
            self.noise_on_roll = np.random.normal(0, self.std_dev_ang_noise_ypr[2] / 3.0, 1)[0]

            self.current_ypr[0] = self.current_ypr[0] + self.noise_on_yaw
            self.current_ypr[1] = self.current_ypr[1] + self.noise_on_pitch
            self.current_ypr[2] = self.current_ypr[2] + self.noise_on_roll


            rot = R.from_euler('zyx', [self.current_ypr[0], self.current_ypr[1], self.current_ypr[2]], degrees=True)
            instant_R = rot.as_matrix()

            final_R = np.matmul(self.static_R, instant_R)
            final_rot = R.from_matrix(final_R)
            quat = final_rot.as_quat()
            self.marker_pose_msg.transform.rotation.x = quat[0]
            self.marker_pose_msg.transform.rotation.y = quat[1]
            self.marker_pose_msg.transform.rotation.z = quat[2]
            self.marker_pose_msg.transform.rotation.w = quat[3]
            
            # Publish the message
            self.pose_pub.publish(self.marker_pose_msg)

            presence_msg = Bool()
            presence_msg.data = True
            self.presence_pub.publish(presence_msg)

            self.br.sendTransform(self.marker_pose_msg)
        else:
            presence_msg = Bool()
            presence_msg.data = False
            self.presence_pub.publish(presence_msg)

        self.link_base_pose_msg.header = Header()
        self.link_base_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.link_base_pose_msg.header.frame_id = self.link_base
        self.link_base_pose_msg.child_frame_id = self.camera_link

        self.link_base_pose_msg.transform.translation.x = self.link_base_pose_msg.transform.translation.x + self.velocity_link_base[0] / self.pose_topic_hz
        self.link_base_pose_msg.transform.translation.y = self.link_base_pose_msg.transform.translation.y + self.velocity_link_base[1] / self.pose_topic_hz
        self.link_base_pose_msg.transform.translation.z = self.link_base_pose_msg.transform.translation.z + self.velocity_link_base[2] / self.pose_topic_hz
        # Publish the message
        self.br.sendTransform(self.link_base_pose_msg)

        self.static_broadcaster.sendTransform(self.t1)

        


# Main loop function
def main(args=None):
    rclpy.init(args=args)
    node = FakeMarkerPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in publish_fake_marker:', file=sys.stderr)
        raise
    finally:
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()
