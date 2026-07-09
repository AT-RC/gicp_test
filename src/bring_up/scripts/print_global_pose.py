#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class GlobalPosePrinter(Node):
    def __init__(self):
        super().__init__('global_pose_printer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 每 1 秒打印一次
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info("Global Pose Printer Started. Waiting for TF (map -> odin1_base_link)...")

    def on_timer(self):
        try:
            # 获取最新的 TF
            t = self.tf_buffer.lookup_transform(
                'map',
                'odin1_base_link',
                rclpy.time.Time())
            
            # 提取平移
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            
            # 提取旋转并转换为欧拉角
            q = t.transform.rotation
            sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            
            sinp = 2 * (q.w * q.y - q.z * q.x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)
                
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            yaw_deg = math.degrees(yaw)
            
            self.get_logger().info(
                f"\n=== 全局重定位位姿 (Global Pose) ===\n"
                f"  平移 (X, Y, Z): [{x:.3f}, {y:.3f}, {z:.3f}] 米\n"
                f"  朝向 (Yaw/偏航): {yaw_deg:.2f} 度"
            )
            
        except TransformException as ex:
            self.get_logger().debug(f"Could not transform map to odin1_base_link: {ex}")

def main():
    rclpy.init()
    node = GlobalPosePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
