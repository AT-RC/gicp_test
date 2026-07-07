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
    enable_court_crop = LaunchConfiguration('enable_court_crop')
    rviz = LaunchConfiguration('rviz')

    # 声明参数
    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='point_lio',
        description='Choose LIO type: fast_lio or point_lio'
    )

    declare_save_map = DeclareLaunchArgument(
        'save_map',
        default_value='false',
        description='Whether to enable mapping mode and save PCD map'
    )

    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='true',
        description='Whether to enable GICP relocalization'
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=PathJoinSubstitution([point_lio_dir, "PCD", "taskzuoh.pcd"]),
        description="Full path to prior PCD file to load",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_enable_global_search = DeclareLaunchArgument(
        'enable_global_search',
        default_value='true',
        description='Whether to enable full map global search for GICP'
    )

    declare_enable_court_crop = DeclareLaunchArgument(
        'enable_court_crop',
        default_value='false',
        description='Whether to crop live scan points before continuous GICP'
    )

    # 2. 包含 Livox Mid360 雷达驱动 Launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_driver_dir, 'launch', 'msg_MID3602_launch.py'])
        )
    )

    # 3.5 启动 Point-LIO 节点
    point_lio_cfg_dir = PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"])
    point_lio_node = Node(
        condition=IfCondition(PythonExpression(["'", lio_type, "' == 'point_lio'"])),
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio_node',
        output='screen',
        parameters=[point_lio_cfg_dir, {
            'common.map_frame': 'odom',
            'common.odom_frame': 'odom',
            'common.base_frame': 'mid360_imu',
            'common.lidar_frame': 'lidar',
            'publish.tf_send_en': False,
            'pcd_save.pcd_save_en': save_map
        }],
        remappings=[
            ('/aft_mapped_to_init', '/odometry'),
            ('/point_lio/reset_state', '/point_lio/reset_state_disabled'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # 4. 包含 GICP 重定位 Launch
    gicp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gicp_dir, 'launch', 'small_gicp_relocalization_launch.py'])
        ),
        condition=IfCondition(localization),
        launch_arguments={
            'prior_pcd_file': prior_pcd_file,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'lidar_frame': 'mid360_imu',
            'odom_topic': '/lidar_odometry',
            'enable_global_search': enable_global_search,
            'map_filter_x_min': '-1.5',
            'map_filter_x_max': '7.0',
            'map_filter_y_min': '-5.0',
            'map_filter_y_max': '1.0',
            'global_search_coarse_step': '2.0',
            'global_search_coarse_yaw_samples': '6',
            'global_search_coarse_iters': '3',
            'global_search_fine_iters': '20',
            'max_dist_sq': '9.0',
            'continuous_update_rate': '0.5',
            'max_z_deviation': '1.5',
            'enable_court_crop': enable_court_crop
        }.items()
    )

    # 5. 包含 loam_interface Launch
    # 它将 Fast-LIO 的里程计从 lidar_odom 转换到 odom 系，并发布 odom -> base_link
    loam_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([loam_interface_dir, 'launch', 'loam_interface_launch.py'])
        ),
        launch_arguments={
            'state_estimation_topic': '/odometry',
            'registered_scan_topic': '/cloud_registered',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'lidar_frame': 'mid360_imu'
        }.items()
    )

    # 6. 启动 C++ 坐标监控节点
    odom_monitor_node = Node(
        package='bring_up',
        executable='odom_monitor',
        name='odom_monitor',
        output='screen'
    )

    map_monitor_node = Node(
        package='bring_up',
        executable='map_monitor',
        name='map_monitor',
        output='screen'
    )

    # 7. 静态 TF 发布 (base_link -> MID360 IMU/body)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_mid360_imu',
        arguments=['-0.22', '0', '0', '0', '0', '1.0', '0', 'base_link', 'mid360_imu']
    )

    # 8. 启动 RViz
    rviz_config_file = PathJoinSubstitution([bring_up_dir, 'rviz', 'airy.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_lio_type,
        declare_save_map,
        declare_localization,
        declare_prior_pcd_file_cmd,
        declare_enable_global_search,
        declare_enable_court_crop,
        declare_rviz_arg,
        loam_interface_launch,
        livox_launch,
        point_lio_node,
        gicp_launch,
        # odom_monitor_node,
        map_monitor_node,
        static_tf_node,
        rviz_node
    ])
