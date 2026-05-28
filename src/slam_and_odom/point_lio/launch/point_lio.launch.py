import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    save_map = LaunchConfiguration("save_map")
    point_filter_num = LaunchConfiguration("point_filter_num")
    filter_size_surf = LaunchConfiguration("filter_size_surf")

    point_lio_dir = get_package_share_directory("point_lio")
    livox_driver_dir = get_package_share_directory("livox_ros_driver2")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Flag to launch RViz."
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )

    declare_save_map = DeclareLaunchArgument(
        "save_map",
        default_value="false",
        description="Flag to enable map saving (pcd_save_en)",
    )

    declare_point_filter_num = DeclareLaunchArgument(
        "point_filter_num",
        default_value="4",
        description="Point filter number (1 for all points)",
    )

    declare_filter_size_surf = DeclareLaunchArgument(
        "filter_size_surf",
        default_value="0.5",
        description="Voxel filter size for surface points",
    )

    # 包含 Mid360 驱动
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_driver_dir, 'launch', 'msg_MID360_launch.py'])
        )
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[
            point_lio_cfg_dir,
            {
                "pcd_save.pcd_save_en": save_map,
                "point_filter_num": point_filter_num,
                "filter_size_surf": filter_size_surf,
                "publish.tf_send_en": True
            }
        ],
        remappings=remappings,
        output="screen",
    )

    start_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "lidar"],
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox.rviz"]),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(declare_save_map)
    ld.add_action(declare_point_filter_num)
    ld.add_action(declare_filter_size_surf)
    
    ld.add_action(livox_launch)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_tf_node)
    ld.add_action(start_rviz_node)

    return ld
