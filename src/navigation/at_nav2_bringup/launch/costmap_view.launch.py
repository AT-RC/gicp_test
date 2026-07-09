import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('at_nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'costmap_params.yaml'),
        description='Full path to the ROS2 parameters file to use for costmap')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'costmap.rviz'),
        description='Full path to the RVIZ config file to use')

    # 发布 PCD 为 PointCloud2 话题
    pcd_to_pointcloud_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_to_pointcloud',
        output='screen',
        parameters=[{
            'file_name': '/home/xjh/Desktop/at_rc/src/slam_and_odom/point_lio/PCD/new_map4.pcd',
            'tf_frame': 'map',
            'publishing_period_ms': 100
        }],
        remappings=[('cloud_pcd', '/terrain_map')]
    )

    # 发布伪造的 TF 树 (map -> odom -> base_link) 以便能够进行坐标转换
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    start_local_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    activate_costmap_cmd = ExecuteProcess(
        cmd=[
            'sh', '-c',
            'sleep 4 && ros2 lifecycle set /costmap/costmap configure && sleep 2 && ros2 lifecycle set /costmap/costmap activate'
        ],
        output='screen'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    from launch.actions import TimerAction
    delayed_pcd_to_pointcloud = TimerAction(
        period=10.0,
        actions=[pcd_to_pointcloud_node]
    )
    ld.add_action(static_tf_map_odom)
    ld.add_action(static_tf_odom_base)
    ld.add_action(start_local_costmap_cmd)
    ld.add_action(delayed_pcd_to_pointcloud)
    ld.add_action(activate_costmap_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
