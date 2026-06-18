import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap

def generate_launch_description():
    """
    启动 Odin1 驱动 + small_gicp 重定位的联合 Launch 文件
    直接将 Odin1 的实时点云喂给 small_gicp，并在全局 3D 地图中定位。
    """
    
    # 1. 引入 Odin1 驱动的 Launch 文件
    # 它会自动启动硬件、发布 /odin1/cloud_slam 和 odom -> odin1_base_link 的 TF
    odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('odin_ros_driver'), '/launch/odin1_ros2.launch.py'
        ])
    )

    # 2. 引入 small_gicp_relocalization，并使用 GroupAction 来限制重映射的作用域
    small_gicp_launch = GroupAction(
        actions=[
            # 将 small_gicp 默认订阅的 registered_scan 重映射为 Odin 的输出
            # 注意：如果您的 Odin 配置中 custom_map_mode=0，请将这里改为 /odin1/cloud_raw
            SetRemap(src='registered_scan', dst='/odin1/cloud_slam'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('small_gicp_relocalization'), '/launch/small_gicp_relocalization_launch.py'
                ]),
                launch_arguments={
                    # 指定你建好的 3D 地图
                    'prior_pcd_file': '/home/xjh/Desktop/map/new_map4.pcd',
                    
                    # 匹配 Odin 的坐标系名称
                    'lidar_frame': 'odin1_base_link',
                    'base_frame': 'odin1_base_link',
                    'robot_base_frame': 'odin1_base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        odin_launch,
        small_gicp_launch
    ])
