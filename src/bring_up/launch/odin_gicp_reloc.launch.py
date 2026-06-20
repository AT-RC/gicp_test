import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap, Node

def generate_launch_description():
    """
    启动 Odin1 驱动 + small_gicp 重定位的联合 Launch 文件
    直接将 Odin1 的实时点云喂给 small_gicp，并在全局 3D 地图中定位。
    建图指令：echo "set save_map 1" > /tmp/odin_command.txt（新终端中）
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
                    # 使用 Launch 参数的魔法，把全局搜索网格强制压缩成一个点（原点 [0,0,0, yaw=0]）
                    # 这样它就只会从原点启动，不会去大范围乱搜，也就彻底避免了对称性跑飞！
                    'enable_global_search': 'true',
                    'map_filter_x_min': '0.0',
                    'map_filter_x_max': '0.0',
                    'map_filter_y_min': '0.0',
                    'map_filter_y_max': '0.0',
                    'map_filter_z_min': '-1.0',
                    'map_filter_z_max': '1.0',
                    'global_search_coarse_yaw_samples': '1',
                }.items()
            )
        ]
    )

    # 3. 启动 pcl_ros 发布 3D 全局地图供 RViz 显示
    map_pcd_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_to_pointcloud',
        parameters=[
            {'file_name': '/home/xjh/Desktop/map/new_map4.pcd'},
            {'tf_frame': 'map'},
            {'frame_id': 'map'},
            {'interval': 0.5}  # 低频发布即可，地图不怎么动
        ],
        remappings=[('cloud_pcd', '/global_map')]
    )

    # 4. 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join('/home/xjh/Desktop/at_rc/src/bring_up/rviz', 'reloc.rviz')],
        output='screen'
    )

    # 5. 启动打印全局位姿的脚本
    print_pose_node = Node(
        package='bring_up',
        executable='print_global_pose.py',
        name='global_pose_printer',
        output='screen'
    )

    return LaunchDescription([
        odin_launch,
        small_gicp_launch,
        map_pcd_node,
        rviz_node,
        print_pose_node
    ])
