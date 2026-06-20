from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取各个包的路径
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')
    point_lio_dir = get_package_share_directory('point_lio')
    bring_up_dir = get_package_share_directory('bring_up')
    gicp_dir = get_package_share_directory('small_gicp_relocalization')
    loam_interface_dir = get_package_share_directory('loam_interface')

    # 定义参数
    save_map = LaunchConfiguration('save_map')
    lio_type = LaunchConfiguration('lio_type')
    localization = LaunchConfiguration('localization')
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    enable_global_search = LaunchConfiguration('enable_global_search')
    rviz = LaunchConfiguration('rviz')

    # 声明参数
    declare_lio_type = DeclareLaunchArgument('lio_type', default_value='point_lio')
    declare_save_map = DeclareLaunchArgument('save_map', default_value='false')
    declare_localization = DeclareLaunchArgument('localization', default_value='true')
    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=PathJoinSubstitution([point_lio_dir, "PCD", "scans_1.pcd"])
    )
    declare_rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    declare_enable_global_search = DeclareLaunchArgument('enable_global_search', default_value='false')

    # 2. 包含 Livox Mid360 雷达驱动 Launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([livox_driver_dir, 'launch', 'msg_MID360_launch.py']))
    )

    # 3. 启动 Point-LIO 节点 (仍然输出 /odometry)
    point_lio_cfg_dir = PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"])
    point_lio_node = Node(
        condition=IfCondition(PythonExpression(["'", lio_type, "' == 'point_lio'"])),
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio_node',
        output='screen',
        parameters=[point_lio_cfg_dir, {
            'publish.tf_send_en': False,
            'pcd_save.pcd_save_en': save_map
        }],
        remappings=[
            ('/aft_mapped_to_init', '/odometry'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # 4. 包含 GICP 重定位 Launch (架空它的 TF 输出)
    gicp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([gicp_dir, 'launch', 'small_gicp_relocalization_launch.py'])),
        condition=IfCondition(localization),
        launch_arguments={
            'prior_pcd_file': prior_pcd_file,
            'map_frame': 'map_gicp',        # 隔离的 map
            'odom_frame': 'odom_lio',       # GICP 需要追踪 LIO 的 odom
            'base_frame': 'base_link_lio',  # GICP 需要追踪 LIO 的 base_link
            'lidar_frame': 'lidar_lio',
            'robot_base_frame': 'base_link_lio',
            'enable_global_search': enable_global_search
        }.items()
    )

    # 5. 包含 loam_interface Launch (仅用于点云转换，架空它的 TF)
    loam_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([loam_interface_dir, 'launch', 'loam_interface_launch.py'])),
        launch_arguments={
            'state_estimation_topic': '/odometry',
            'registered_scan_topic': '/cloud_registered',
            'odom_frame': 'odom_lio',       # 隔离的 odom
            'base_frame': 'base_link_lio',  # 隔离的 base_link
            'lidar_frame': 'lidar_lio'      # 隔离的 lidar
        }.items()
    )

    # 6. 启动 TF 到 Pose 转换器
    tf_to_pose_node = Node(
        package='bring_up',
        executable='tf_to_pose.py',
        name='tf_to_pose_converter',
        output='screen',
        parameters=[{
            'target_frame': 'odom_lio',     # GICP 发布的子坐标系现在是 odom_lio
            'source_frame': 'map_gicp',
            'pose_topic': '/gicp_pose',
            'rate': 10.0,
            'output_frame_id': 'map'
        }]
    )

    # 7. 启动 Local EKF (输出 odom -> base_link)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[PathJoinSubstitution([bring_up_dir, 'config', 'ekf.yaml'])],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 8. 启动 Global EKF (输出 map -> odom)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[PathJoinSubstitution([bring_up_dir, 'config', 'ekf.yaml'])],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # 9. 启动 C++ 全局坐标监控节点 (包含初始化状态检测)
    map_monitor_node = Node(
        package='bring_up',
        executable='ekf_map_monitor',
        name='ekf_map_monitor',
        output='screen'
    )

    # 10. 静态 TF 发布 (正式的 base_link -> lidar，供 EKF 系统使用)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar',
        arguments=['-0.15', '0', '0.138', '0', '0', '1.0', '0', 'base_link', 'lidar']
    )

    # 11. 静态 TF 发布 (隔离的 base_link_lio -> lidar_lio，供 LIO 系统使用)
    static_tf_node_lio = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_lio_to_lidar_lio',
        arguments=['-0.15', '0', '0.138', '0', '0', '1.0', '0', 'base_link_lio', 'lidar_lio']
    )

    # 11. 启动 RViz
    rviz_config_file = PathJoinSubstitution([bring_up_dir, 'rviz', 'airy.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_lio_type, declare_save_map, declare_localization,
        declare_prior_pcd_file_cmd, declare_enable_global_search, declare_rviz_arg,
        loam_interface_launch,
        livox_launch,
        point_lio_node,
        gicp_launch,
        static_tf_node,
        tf_to_pose_node,
        ekf_local_node,
        ekf_global_node,
        map_monitor_node,
        rviz_node
    ])
