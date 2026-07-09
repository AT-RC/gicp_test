"""
Launch 文件修补脚本。
主要作用：使用正则替换自动修改 bringup_ekf.launch.py，为所有的 EKF 核心节点（ekf_local, ekf_global, tf_to_pose）注入 `{'use_sim_time': True}` 参数。
这是离线回放数据包调参所必须的，否则 EKF 节点会丢弃过去时间的历史数据。
"""
import re
file_path = '/home/xjh/Desktop/at_rc/src/bring_up/launch/bringup_ekf.launch.py'
with open(file_path, 'r') as f:
    content = f.read()

# For tf_to_pose_node
content = re.sub(r"parameters=\[\{", "parameters=[{'use_sim_time': True, ", content)

# For ekf nodes
content = re.sub(r"parameters=\[PathJoinSubstitution\(\[bring_up_dir, 'config', 'ekf.yaml'\]\)\]", "parameters=[PathJoinSubstitution([bring_up_dir, 'config', 'ekf.yaml']), {'use_sim_time': True}]", content)

with open(file_path, 'w') as f:
    f.write(content)
