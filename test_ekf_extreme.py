import subprocess
import time
import os
import shutil
import signal
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TF_PY = "src/bring_up/scripts/tf_to_pose.py"
BAG_FILE = "dog_ekf_tuning_bag1"

def modify_cov(gicp_cov):
    with open(TF_PY, "r") as f:
        lines = f.readlines()
    for i in range(len(lines)):
        if "msg.pose.covariance =" in lines[i]:
            lines[i+1] = f"                {gicp_cov}, 0.0, 0.0, 0.0, 0.0, 0.0,\n"
            lines[i+2] = f"                0.0, {gicp_cov}, 0.0, 0.0, 0.0, 0.0,\n"
            lines[i+3] = f"                0.0, 0.0, {gicp_cov}, 0.0, 0.0, 0.0,\n"
            lines[i+6] = f"                0.0, 0.0, 0.0, 0.0, 0.0, {gicp_cov}\n"
            break
    with open(TF_PY, "w") as f:
        f.writelines(lines)
    subprocess.run(["bash", "-c", "source install/setup.bash && colcon build --packages-select bring_up"], stdout=subprocess.DEVNULL)

def run_sim(cov_val):
    modify_cov(cov_val)
    bag_out = f"temp_bag_{cov_val}"
    if os.path.exists(bag_out):
        shutil.rmtree(bag_out)
        
    p_launch = subprocess.Popen(["bash", "-c", "source install/setup.bash && ros2 launch bring_up bringup_ekf.launch.py rviz:=false"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)
    p_record = subprocess.Popen(["bash", "-c", f"source install/setup.bash && ros2 bag record /odometry/global -o {bag_out}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    bag_cmd = ['bash', '-c', f"source install/setup.bash && ros2 bag play {BAG_FILE} --clock -r 1 --start-offset 150 -d 5 --topics /tf /tf_static /odometry /livox/imu"]
    subprocess.run(bag_cmd, stdout=subprocess.DEVNULL)
    time.sleep(1)
    
    p_record.send_signal(signal.SIGINT)
    p_launch.send_signal(signal.SIGINT)
    try:
        p_record.wait(timeout=5)
        p_launch.wait(timeout=5)
    except:
        p_record.kill()
        p_launch.kill()
    return bag_out

def get_mean_y(db_file):
    conn = sqlite3.connect(db_file)
    c = conn.cursor()
    c.execute("SELECT messages.timestamp, topics.type, messages.data FROM messages JOIN topics ON topics.id = messages.topic_id WHERE topics.name = '/odometry/global'")
    times, positions = [], []
    for row in c.fetchall():
        msg_type = get_message(row[1])
        msg = deserialize_message(row[2], msg_type)
        positions.append(msg.pose.pose.position.y)
    return np.mean(positions)

b1 = run_sim(0.1)
b2 = run_sim(100.0)
m1 = get_mean_y(b1 + "/" + b1 + "_0.db3")
m2 = get_mean_y(b2 + "/" + b2 + "_0.db3")
print(f"Mean Y for 0.1: {m1}")
print(f"Mean Y for 100.0: {m2}")
