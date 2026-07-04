# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    point_lio_dir = get_package_share_directory('point_lio')

    num_threads = LaunchConfiguration("num_threads")
    num_neighbors = LaunchConfiguration("num_neighbors")
    global_leaf_size = LaunchConfiguration("global_leaf_size")
    registered_leaf_size = LaunchConfiguration("registered_leaf_size")
    max_dist_sq = LaunchConfiguration("max_dist_sq")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    robot_base_frame = LaunchConfiguration("robot_base_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    enable_global_search = LaunchConfiguration("enable_global_search")
    continuous_update_rate = LaunchConfiguration("continuous_update_rate")
    max_z_deviation = LaunchConfiguration("max_z_deviation")

    declare_num_threads = DeclareLaunchArgument(
        "num_threads", default_value="4", description="Number of threads"
    )
    declare_num_neighbors = DeclareLaunchArgument(
        "num_neighbors", default_value="10", description="Number of neighbors"
    )
    declare_global_leaf_size = DeclareLaunchArgument(
        "global_leaf_size", default_value="0.25", description="Global leaf size"
    )
    declare_registered_leaf_size = DeclareLaunchArgument(
        "registered_leaf_size", default_value="0.25", description="Registered leaf size"
    )
    declare_max_dist_sq = DeclareLaunchArgument(
        "max_dist_sq", default_value="16.0", description="Max distance squared"
    )
    declare_map_frame = DeclareLaunchArgument(
        "map_frame", default_value="map", description="Map frame"
    )
    declare_odom_frame = DeclareLaunchArgument(
        "odom_frame", default_value="odom", description="Odom frame"
    )
    declare_base_frame = DeclareLaunchArgument(
        "base_frame", default_value="base_link", description="Base frame"
    )
    declare_lidar_frame = DeclareLaunchArgument(
        "lidar_frame", default_value="lidar", description="Lidar frame"
    )
    declare_robot_base_frame = DeclareLaunchArgument(
        "robot_base_frame", default_value="base_link", description="Robot base frame"
    )
    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic", default_value="/aft_mapped_to_init", description="Odometry topic for divergence detection"
    )
    declare_map_filter_x_min = DeclareLaunchArgument("map_filter_x_min", default_value="-1.0")
    declare_map_filter_x_max = DeclareLaunchArgument("map_filter_x_max", default_value="7.0")
    declare_map_filter_y_min = DeclareLaunchArgument("map_filter_y_min", default_value="-5.0")
    declare_map_filter_y_max = DeclareLaunchArgument("map_filter_y_max", default_value="1.0")
    declare_prior_pcd_file = DeclareLaunchArgument(
        "prior_pcd_file", 
        default_value=PathJoinSubstitution([point_lio_dir, "PCD", "scans.pcd"]), 
        description="Prior PCD file"
    )
    declare_enable_global_search = DeclareLaunchArgument(
        "enable_global_search", default_value="true", description="Enable full map global search"
    )
    declare_continuous_update_rate = DeclareLaunchArgument(
        "continuous_update_rate", default_value="1.0", description="Continuous update rate for GICP"
    )
    declare_update_min_translation = DeclareLaunchArgument(
        "update_min_translation", default_value="0.05", description="Minimum translation to update GICP pose (meters)"
    )
    declare_update_min_rotation = DeclareLaunchArgument(
        "update_min_rotation", default_value="0.05", description="Minimum rotation to update GICP pose (radians)"
    )
    declare_max_z_deviation = DeclareLaunchArgument(
        "max_z_deviation", default_value="0.5", description="Maximum allowed odometry Z deviation before reset (meters)"
    )

    # 赛场实时裁剪参数（原始地图系，与 transform_map.py 的裁剪框保持一致）
    declare_enable_court_crop = DeclareLaunchArgument(
        "enable_court_crop", default_value="true", description="Enable runtime court cropping of live scan"
    )
    declare_court_crop_x_min = DeclareLaunchArgument("court_crop_x_min", default_value="0.0")
    declare_court_crop_x_max = DeclareLaunchArgument("court_crop_x_max", default_value="6.0")
    declare_court_crop_y_min = DeclareLaunchArgument("court_crop_y_min", default_value="-4.0")
    declare_court_crop_y_max = DeclareLaunchArgument("court_crop_y_max", default_value="0.0")
    declare_court_crop_margin = DeclareLaunchArgument("court_crop_margin", default_value="0.3")
    declare_court_crop_z_min = DeclareLaunchArgument("court_crop_z_min", default_value="2.0")

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=[
            {
                "num_threads": num_threads,
                "num_neighbors": num_neighbors,
                "global_leaf_size": global_leaf_size,
                "registered_leaf_size": registered_leaf_size,
                "max_dist_sq": max_dist_sq,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "lidar_frame": lidar_frame,
                "robot_base_frame": robot_base_frame,
                "odom_topic": odom_topic,
                "map_filter_x_min": LaunchConfiguration("map_filter_x_min"),
                "map_filter_x_max": LaunchConfiguration("map_filter_x_max"),
                "map_filter_y_min": LaunchConfiguration("map_filter_y_min"),
                "map_filter_y_max": LaunchConfiguration("map_filter_y_max"),
                "prior_pcd_file": prior_pcd_file,
                "enable_global_search": enable_global_search,
                "continuous_update_rate": continuous_update_rate,
                "update_min_translation": LaunchConfiguration("update_min_translation"),
                "update_min_rotation": LaunchConfiguration("update_min_rotation"),
                "max_z_deviation": max_z_deviation,
                "enable_court_crop": LaunchConfiguration("enable_court_crop"),
                "court_crop_x_min": LaunchConfiguration("court_crop_x_min"),
                "court_crop_x_max": LaunchConfiguration("court_crop_x_max"),
                "court_crop_y_min": LaunchConfiguration("court_crop_y_min"),
                "court_crop_y_max": LaunchConfiguration("court_crop_y_max"),
                "court_crop_margin": LaunchConfiguration("court_crop_margin"),
                "court_crop_z_min": LaunchConfiguration("court_crop_z_min"),
            }
        ],
    )

    return LaunchDescription([
        declare_num_threads,
        declare_num_neighbors,
        declare_global_leaf_size,
        declare_registered_leaf_size,
        declare_max_dist_sq,
        declare_map_frame,
        declare_odom_frame,
        declare_base_frame,
        declare_lidar_frame,
        declare_robot_base_frame,
        declare_odom_topic,
        declare_map_filter_x_min,
        declare_map_filter_x_max,
        declare_map_filter_y_min,
        declare_map_filter_y_max,
        declare_prior_pcd_file,
        declare_enable_global_search,
        declare_continuous_update_rate,
        declare_update_min_translation,
        declare_update_min_rotation,
        declare_max_z_deviation,
        declare_enable_court_crop,
        declare_court_crop_x_min,
        declare_court_crop_x_max,
        declare_court_crop_y_min,
        declare_court_crop_y_max,
        declare_court_crop_margin,
        declare_court_crop_z_min,
        node
    ])
