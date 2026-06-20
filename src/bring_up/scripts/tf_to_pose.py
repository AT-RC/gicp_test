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
        
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        pose_topic = self.get_parameter('pose_topic').value
        rate = self.get_parameter('rate').value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(f"Started TF to Pose converter: {self.source_frame} -> {self.target_frame} at {rate} Hz")

    def timer_callback(self):
        try:
            # We want the pose of odom_gicp in map_gicp
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time())
            
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.output_frame_id
            
            msg.pose.pose.position.x = trans.transform.translation.x
            msg.pose.pose.position.y = trans.transform.translation.y
            msg.pose.pose.position.z = trans.transform.translation.z
            msg.pose.pose.orientation = trans.transform.rotation
            
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
