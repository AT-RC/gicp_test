import re

with open('/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/launch/nav2_full.launch.py', 'r') as f:
    content = f.read()

terrain_analysis_str = """    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[
            ('/registered_scan', '/cloud_registered'),
            ('/state_estimation', '/odometry')
        ]
    )"""

terrain_ext_str = """    start_terrain_analysis_ext_cmd = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[
            ('/registered_scan', '/cloud_registered'),
            ('/state_estimation', '/odometry')
        ]
    )"""

# Use regex to replace the nodes
content = re.sub(
    r'    start_terrain_analysis_cmd = Node\([\s\S]*?parameters=\[configured_params\],\n    \)',
    terrain_analysis_str,
    content
)

content = re.sub(
    r'    start_terrain_analysis_ext_cmd = Node\([\s\S]*?parameters=\[configured_params\],\n    \)',
    terrain_ext_str,
    content
)

with open('/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/launch/nav2_full.launch.py', 'w') as f:
    f.write(content)
