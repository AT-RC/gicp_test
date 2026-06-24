import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pcd_file = '/home/xjh/Desktop/at_rc/src/slam_and_odom/point_lio/PCD/new_map4.pcd'
    yaml_file = '/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/maps/keepout_mask.yaml'
    
    rviz_config_file = os.path.join('/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup', 'rviz', 'verify_map.rviz')
    
    return LaunchDescription([
        # Publish 3D PCD Map
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_to_pointcloud',
            arguments=[], 
            parameters=[{'frame_id': 'map', 'tf_frame': 'map', 'file_name': pcd_file, 'interval': 1.0}],
            remappings=[('cloud_pcd', 'prior_pcd')]
        ),
        
        # Publish 2D Keepout Mask Map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': yaml_file}]
        ),
        
        # Explicitly configure and activate map_server sequentially
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 1 && ros2 lifecycle set /map_server configure && sleep 1 && ros2 lifecycle set /map_server activate'],
            output='screen'
        ),
        
        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
