#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped

class TfToPoseNode(Node):
    def __init__(self):
        super().__init__('tf_to_pose_converter')
        
        self.declare_parameter('target_frame', 'odom_gicp')
        self.declare_parameter('source_frame', 'map_gicp')
        self.declare_parameter('pose_topic', '/gicp_pose')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('output_frame_id', 'map')
        
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        pose_topic = self.get_parameter('pose_topic').value
        rate = self.get_parameter('rate').value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(f"Started TF to Pose converter: {self.source_frame}->{self.target_frame} * {self.odom_frame}->{self.base_frame}")

    def timer_callback(self):
        try:
            # Get T_{map_gicp -> odom_gicp}
            trans1 = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time())
            # Get T_{odom -> base_link}
            trans2 = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())

            # pure math for pose composition
            # q = [x, y, z, w]
            q1 = [trans1.transform.rotation.x, trans1.transform.rotation.y, trans1.transform.rotation.z, trans1.transform.rotation.w]
            p1 = [trans1.transform.translation.x, trans1.transform.translation.y, trans1.transform.translation.z]
            
            q2 = [trans2.transform.rotation.x, trans2.transform.rotation.y, trans2.transform.rotation.z, trans2.transform.rotation.w]
            p2 = [trans2.transform.translation.x, trans2.transform.translation.y, trans2.transform.translation.z]
            
            # Quaternion multiplication q_res = q1 * q2
            q_res = [
                q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1],
                q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0],
                q1[3]*q2[2] + q1[0]*q2[1] - q1[1]*q2[0] + q1[2]*q2[3],
                q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
            ]
            
            # Rotate p2 by q1
            # p_rotated = q1 * p2 * q1_inv
            # simple vector rotation formula:
            import numpy as np
            # Convert quaternion to rotation matrix
            R = np.array([
                [1 - 2*q1[1]**2 - 2*q1[2]**2, 2*q1[0]*q1[1] - 2*q1[3]*q1[2], 2*q1[0]*q1[2] + 2*q1[3]*q1[1]],
                [2*q1[0]*q1[1] + 2*q1[3]*q1[2], 1 - 2*q1[0]**2 - 2*q1[2]**2, 2*q1[1]*q1[2] - 2*q1[3]*q1[0]],
                [2*q1[0]*q1[2] - 2*q1[3]*q1[1], 2*q1[1]*q1[2] + 2*q1[3]*q1[0], 1 - 2*q1[0]**2 - 2*q1[1]**2]
            ])
            p2_rotated = R.dot(np.array(p2))
            
            p_res = np.array(p1) + p2_rotated

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.output_frame_id

            msg.pose.pose.position.x = float(p_res[0])
            msg.pose.pose.position.y = float(p_res[1])
            msg.pose.pose.position.z = float(p_res[2])
            
            msg.pose.pose.orientation.x = float(q_res[0])
            msg.pose.pose.orientation.y = float(q_res[1])
            msg.pose.pose.orientation.z = float(q_res[2])
            msg.pose.pose.orientation.w = float(q_res[3])
            
            # Covariance: small for xyz, yaw, large for roll/pitch since GICP is mostly planar reliable
            msg.pose.covariance = [
                0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 100.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.5
            ]
            
            self.pose_pub.publish(msg)
        except tf2_ros.TransformException as e:
            # TF might not be available yet, ignore
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TfToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
