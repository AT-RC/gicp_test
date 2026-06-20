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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    state_estimation_topic = LaunchConfiguration("state_estimation_topic")
    registered_scan_topic = LaunchConfiguration("registered_scan_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_tf = LaunchConfiguration("publish_tf")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the node"
    )
    declare_state_estimation_topic = DeclareLaunchArgument(
        "state_estimation_topic", default_value="aft_mapped_to_init", description="State estimation topic"
    )
    declare_registered_scan_topic = DeclareLaunchArgument(
        "registered_scan_topic", default_value="cloud_registered", description="Registered scan topic"
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
    declare_publish_tf = DeclareLaunchArgument(
        "publish_tf", default_value="true", description="Whether to publish TF"
    )

    start_loam_interface = Node(
        package="loam_interface",
        executable="loam_interface_node",
        name="loam_interface",
        namespace=namespace,
        remappings=remappings,
        output="screen",
        parameters=[
            {
                "state_estimation_topic": state_estimation_topic,
                "registered_scan_topic": registered_scan_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "lidar_frame": lidar_frame,
                "publish_tf": publish_tf,
            }
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(declare_state_estimation_topic)
    ld.add_action(declare_registered_scan_topic)
    ld.add_action(declare_odom_frame)
    ld.add_action(declare_base_frame)
    ld.add_action(declare_lidar_frame)
    ld.add_action(declare_publish_tf)
    ld.add_action(start_loam_interface)

    return ld
