#!/bin/bash
source install/setup.bash
# 启动 launch 文件并把输出重定向
ros2 launch bring_up bringup_ekf.launch.py rviz:=false > monitor_output.log 2>&1 &
LAUNCH_PID=$!
sleep 5

# 回放数据包（一小段）
ros2 bag play dog_ekf_tuning_bag1 --clock -r 1 --start-offset 150 -d 10 --topics /tf /tf_static /odometry /livox/imu > /dev/null 2>&1

sleep 2
kill -SIGINT $LAUNCH_PID
wait $LAUNCH_PID
