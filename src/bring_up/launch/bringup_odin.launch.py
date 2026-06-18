import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取各个包的路径
    bring_up_dir = get_package_share_directory('bring_up')
    odin_driver_dir = get_package_share_directory('odin_ros_driver')

    # 2. 定义参数
    save_map = LaunchConfiguration('save_map')
    rviz = LaunchConfiguration('rviz')

    # 3. 声明参数
    declare_save_map = DeclareLaunchArgument(
        'save_map',
        default_value='true',
        description='Whether to enable mapping mode and start the map save trigger node'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # 4. 包含 Odin 雷达驱动 Launch
    odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([odin_driver_dir, 'launch', 'odin1_ros2.launch.py'])
        )
    )

    # 5. 启动 C++ 坐标监控节点 (监控 odom 和 map)
    odom_monitor_node = Node(
        package='bring_up',
        executable='odom_monitor',
        name='odom_monitor',
        output='screen',
        remappings=[
            ('lidar_odometry', '/odin1/odometry')
        ]
    )

    map_monitor_node = Node(
        package='bring_up',
        executable='map_monitor',
        name='map_monitor',
        output='screen'
    )

    # 6. 静态 TF 发布 (base_link -> odin1_base_link)
    # 根据用户确认，参数为 -0.16 0 0 0 0 0
    # 注意：static_transform_publisher 参数顺序是 x y z yaw pitch roll frame_id child_frame_id
    # 在有些 ROS 2 版本中是 x y z roll pitch yaw，通常全0没区别
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.16', '0', '0', '0', '0', '0', 'odin1_base_link', 'base_link']
    )

    # 7. 启动虚拟串口发送节点 (发送位姿到单片机)
    serial_node = Node(
        package='virtual_serial_port',
        executable='virtual_serial_port_node',
        name='virtual_serial_port',
        output='screen',
        parameters=[{
            'usb_vid': 0x0483,
            'usb_pid': 0x5740,
            'send_interval_ms': 10,
            'odom_frame': 'odom',        # Odin 输出 odom
            'base_frame': 'base_link'    # 我们通过静态 TF 将 base_link 接上了 odin1_base_link
        }]
    )

    # 8. 启动地图保存触发节点
    map_save_node = Node(
        condition=IfCondition(save_map),
        package='bring_up',
        executable='odin_map_save.py',
        name='save_map_trigger_node',
        output='screen'
    )

    # 9. 启动 RViz
    rviz_config_file = PathJoinSubstitution([odin_driver_dir, 'config', 'odin.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_save_map,
        declare_rviz_arg,
        odin_launch,
        # odom_monitor_node,
        map_monitor_node,
        static_tf_node,
        # serial_node,
        map_save_node,
        rviz_node
    ])
