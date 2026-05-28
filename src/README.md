# AT-RC 机器狗竞赛工作空间说明文档

本项目是一个用于机器狗竞赛的 ROS2 工作空间，集成了激光雷达驱动、高频里程计（SLAM）、环境感知和系统一键启动等功能。

## 📁 源码目录结构 (`src`)

代码库按功能划分为以下几个主要模块：

### 1. `bring_up` (启动管理)
系统集成与调度中心。
- **核心启动脚本**: `bringup.launch.py`。
- **功能**: 一键集成速腾雷达驱动、Point-LIO 里程计、GICP 重定位节点以及坐标监控工具。
- **模式支持**: 支持建图模式（保存 PCD 地图）和定位模式（加载先验 PCD 地图）。

### 2. `drivers` (传感器驱动)
包含项目使用的激光雷达硬件驱动。
- **`livox_ros_driver2`**: 大疆 (Livox) 激光雷达驱动。
- **`sdk/rslidar_sdk`**: 速腾聚创 (RoboSense) 激光雷达驱动及 SDK。

### 3. `slam_and_odom` (定位与建图)
高性能的定位方案。
- **`point_lio`**: 一种稳健、高带宽的激光惯性里程计 (LIO) 框架。
    - **特点**: 输出频率高达 4k-8kHz，对剧烈运动和 IMU 饱和具有很强的鲁棒性。
    - **定制**: 修改版支持加载先验 PCD 点云地图进行初始定位。
- **`small_gicp_relocalization`**: 基于 GICP 算法的重定位模块，用于在已知地图中修正机器狗的位姿。
+=0l
### 4. `perception` (环境感知)
点云处理与地形分析。
- **`terrain_analysis`**: 地形分析模块，用于识别障碍物和可通行区域（主要来自 CMU 自主勘测套件）。
- **`terrain_analysis_ext`**: 地形分析的扩展增强版。
- **`sensor_scan_generation`**: 将 3D 点云转换为类似 2D 扫描的数据，方便后续导航算法使用。

### 5. `loam_interface` (接口适配)
提供 LOAM 风格的数据接口，用于兼容不同的 SLAM 算法数据格式。

### 6. `virtual_serial_port` (虚拟串口)
用于创建虚拟串口通信，通常用于调试或与机器狗底层的运动控制器进行串口协议对接。

---

## 🚀 快速上手

### 环境要求
- ROS2 Humble (建议)
- PCL (Point Cloud Library)
- Eigen3

### 编译项目
在工作空间根目录下执行：
```bash
colcon build --symlink-install
```

### 运行系统
启动集成环境：
```bash
source install/setup.bash
ros2 launch bring_up bringup.launch.py
```

#### 关键启动参数
- `save_map` (默认: `true`): 是否开启建图模式并保存 PCD 地图。
- `localization` (default: `true`): 是否开启 GICP 重定位。
- `prior_pcd_file`: 定位模式下使用的先验地图路径。

---

## 🛠 开发与维护
本项目结合了多个开源框架。更多细节请参考各子包内的 `README.md`：
- `point_lio`: 参考 [HKU-Mars Point-LIO](https://github.com/hku-mars/Point-LIO)。
- `terrain_analysis`: 参考 CMU Autonomous Exploration Development Environment。
