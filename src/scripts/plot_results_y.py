import sqlite3
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def get_data(db_file, topic_name):
    conn = sqlite3.connect(db_file)
    c = conn.cursor()
    c.execute('SELECT messages.timestamp, topics.type, messages.data FROM messages JOIN topics ON topics.id = messages.topic_id WHERE topics.name = ?', (topic_name,))
    times, positions = [], []
    start_time = None
    for row in c.fetchall():
        msg_type = get_message(row[1])
        msg = deserialize_message(row[2], msg_type)
        if start_time is None: start_time = row[0]
        t = (row[0] - start_time) / 1e9
        y = msg.pose.pose.position.y
        times.append(t)
        positions.append(y)
    return times, positions

fig, axs = plt.subplots(3, 1, figsize=(14, 18), sharex=True)
bags = [("out_bag_cov_0.05/out_bag_cov_0.05_0.db3", 0.05),
        ("out_bag_cov_0.50/out_bag_cov_0.50_0.db3", 0.50),
        ("out_bag_cov_5.00/out_bag_cov_5.00_0.db3", 5.00)]

for i, (bag, cov) in enumerate(bags):
    try:
        t_gicp, y_gicp = get_data(bag, '/gicp_pose')
        t_ekf, y_ekf = get_data(bag, '/odometry/global')
        t_lio, y_lio = get_data(bag, '/odometry')
        
        axs[i].plot(t_gicp, y_gicp, label='GICP Measurement (Y)', color='blue', linewidth=3, linestyle='--', alpha=0.6)
        axs[i].plot(t_lio, y_lio, label='Point-LIO Odometry (Y)', color='green', linewidth=2, linestyle='-.', alpha=0.8)
        axs[i].plot(t_ekf, y_ekf, label='EKF Filtered (Y)', color='red', linewidth=2, alpha=0.9)
        
        axs[i].set_title(f"GICP Covariance = {cov}")
        axs[i].set_ylabel("Y Position (m)")
        axs[i].legend()
        axs[i].grid(True)
    except Exception as e:
        print(f"Error processing {bag}: {e}")
axs[-1].set_xlabel("Time (s)")
plt.tight_layout()
plt.savefig('/tmp/ekf_tuning_comparison_y.png', dpi=150)
print("Plot saved.")
