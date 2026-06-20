import subprocess
import time
import signal
import os

BAG_FILE = "dog_ekf_tuning_bag1"

# Launch EKF
p_launch = subprocess.Popen(["ros2", "launch", "bring_up", "bringup_ekf.launch.py"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(2)

# Play
p_play = subprocess.Popen(["ros2", "bag", "play", BAG_FILE, "--clock", "-r", "10"], stdout=subprocess.DEVNULL)
time.sleep(10)

p_play.terminate()
p_launch.send_signal(signal.SIGINT)
stdout, _ = p_launch.communicate()

with open("launch_log.txt", "w") as f:
    f.write(stdout)
