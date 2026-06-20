"""
绘制 EKF 调参对比结果的脚本。
主要作用：读取不同协方差参数下录制的 sqlite3 ROS 2 bag 数据库文件，提取 /gicp_pose 和 /odometry/global 的 X 轴坐标，并使用 matplotlib 将它们画在同一张图表上，生成直观的调参对比图片。
"""
import sqlite3
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sys

def extract_data(db_file):
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()
    
    # Get topic IDs
    cursor.execute("SELECT id, name, type FROM topics")
    topics = cursor.fetchall()
    
    topic_id_map = {t[1]: t[0] for t in topics}
    type_map = {t[1]: t[2] for t in topics}
    
    data = {}
    
    for topic_name in ['/gicp_pose', '/odometry/global']:
        if topic_name not in topic_id_map:
            continue
            
        topic_id = topic_id_map[topic_name]
        msg_type = get_message(type_map[topic_name])
        
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,))
        rows = cursor.fetchall()
        
        times = []
        x_vals = []
        
        for ts, blob in rows:
            msg = deserialize_message(blob, msg_type)
            if topic_name == '/gicp_pose':
                x = msg.pose.pose.position.x
            else:
                x = msg.pose.pose.position.x
            
            times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            x_vals.append(x)
            
        if len(times) > 0:
            start_ts = times[0]
            times = [(t - start_ts) for t in times]
            
        data[topic_name] = (times, x_vals)
        
    return data

fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
covs = ['0.1', '0.01', '0.001']

for i, cov in enumerate(covs):
    try:
        data = extract_data(f"out_bag_cov_{cov}/out_bag_cov_{cov}_0.db3")
        
        if '/gicp_pose' in data:
            t_g, x_g = data['/gicp_pose']
            # 将蓝线变细一点，透明度调低，避免遮挡红线
            axs[i].plot(t_g, x_g, 'b-', label='GICP Raw (Blue)', alpha=0.3, linewidth=1.0)
            
        if '/odometry/global' in data:
            t_o, x_o = data['/odometry/global']
            # 将红线加粗，并设置 zorder=5 确保它在最顶层显示
            axs[i].plot(t_o, x_o, 'r-', label=f'EKF (Red) cov={cov}', linewidth=3.5, zorder=5)
            
        axs[i].set_title(f'GICP Covariance = {cov}')
        axs[i].set_ylabel('X Position (m)')
        axs[i].legend()
        axs[i].grid(True)
    except Exception as e:
        print(f"Error loading {cov}: {e}")

axs[-1].set_xlabel('Time (s)')
plt.tight_layout()
plt.savefig('/tmp/ekf_tuning_comparison.png', dpi=150)
print("Plot saved.")
