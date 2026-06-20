"""
自动化 EKF 调参脚本。
主要作用：自动修改不同的 GICP 协方差参数，并以 3 倍速自动回放数据包、运行 EKF 节点。
它会遍历指定的参数集（如 0.1, 0.01, 0.001），为每个参数录制一份独立的 /odometry/global 轨迹 bag 包，用于后续对比。
"""
import subprocess
import time
import os
import sqlite3
import matplotlib.pyplot as plt
import signal
import shutil
import re
import numpy as np

BAG_FILE = "dog_ekf_tuning_bag1"
TF_PY = "src/bring_up/scripts/tf_to_pose.py"
EKF_YAML = "src/bring_up/config/ekf.yaml"

def run_test(name, gicp_cov):
    print(f"Running test: {name} with GICP cov: {gicp_cov}")
    
    # modify file
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
    
    # remove old bag
    bag_out = f"out_bag_{name}"
    if os.path.exists(bag_out):
        shutil.rmtree(bag_out)
        
    # Launch EKF (make sure to source first)
    p_launch = subprocess.Popen(["bash", "-c", "source install/setup.bash && ros2 launch bring_up bringup_ekf.launch.py rviz:=false"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)
    
    # Record
    p_record = subprocess.Popen(["bash", "-c", f"source install/setup.bash && ros2 bag record /odometry/global /gicp_pose /odometry -o {bag_out}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    
    # Start bag playback: only 150s~300s segment, at 1x real-time speed
    bag_cmd = [
        'bash', '-c', 
        f"source install/setup.bash && ros2 bag play {BAG_FILE} --clock -r 1 --start-offset 150 --topics /tf /tf_static /odometry /livox/imu"
    ]
    subprocess.run(bag_cmd, stdout=subprocess.DEVNULL)
    time.sleep(1)
    
    # Stop record and launch
    p_record.send_signal(signal.SIGINT)
    p_launch.send_signal(signal.SIGINT)
    
    try:
        p_record.wait(timeout=5)
        p_launch.wait(timeout=5)
    except:
        p_record.kill()
        p_launch.kill()

run_test("cov_0.05", 0.05)
run_test("cov_0.50", 0.50)
run_test("cov_5.00", 5.0)
