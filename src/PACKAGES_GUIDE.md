# Robot Perception & Navigation Packages Documentation

本文档介绍了项目中 10 个核心 ROS2 软件包的功能及其关键参数。

---

## 1. bring_up
**功能描述**: 系统的总启动包，负责协调和一键启动雷达驱动、里程计、重定位及坐标监控节点。
**关键参数**:
- `save_map` (bool, default: true): 是否开启建图模式并保存 PCD 地图。
- `localization` (bool, default: true): 是否启用 GICP 重定位。
- `prior_pcd_file` (string): 初始重定位加载的先验地图路径。

## 2. livox_ros_driver2
**功能描述**: 大疆 (Livox) 激光雷达驱动程序，支持 MID360 等型号，负责发布原始点云数据。
**关键参数**:
- `xfer_format` (int, default: 4): 点云传输格式 (0: PointCloud2, 4: Livox 特有格式)。
- `publish_freq` (double, default: 10.0): 点云发布频率 (Hz)。
- `user_config_path` (string): JSON 配置文件路径，包含雷达 IP 和连接设置。
- `frame_id` (string, default: "livox_frame"): 发布点云的坐标系名称。

## 3. rslidar_sdk
**功能描述**: 速腾聚创 (Robosense) 激光雷达驱动，负责数据解析和点云发布。
**关键参数**:
- `msg_source` (int): 消息来源 (1: 在线雷达, 3: Pcap 文件)。
- `lidar_type` (string): 雷达型号 (如 "RSAIRY")。
- `min_distance` / `max_distance` (double): 点云过滤的最短和最远距离。
- `ros_send_point_cloud_topic` (string): 发布点云的话题名称 (默认: `/rslidar_points`)。

## 4. rslidar_msg
**功能描述**: 专门定义速腾聚创雷达特有的 ROS2 消息类型（如原始数据包消息），作为 `rslidar_sdk` 的底层支持。
**关键参数**: (本包主要定义数据结构，无运行参数)

## 5. point_lio
**功能描述**: 高效的激光惯性里程计 (LIO) 算法，通过融合雷达和 IMU 数据实现实时高频的位姿估计。
**关键参数**:
- `lid_topic` (string): 输入的雷达话题。
- `imu_topic` (string): 输入的 IMU 话题。
- `point_filter_num` (int): 点云降采样率 (1 表示使用全量点云)。
- `filter_size_surf` (double): 建图时的体素滤波大小。
- `pcd_save_en` (bool): 是否启用地图自动保存。

## 6. small_gicp_relocalization
**功能描述**: 基于 Small-GICP 算法的全局重定位模块，通过将实时点云与先验地图匹配来校准机器人位姿。
**关键参数**:
- `prior_pcd_file` (string): 匹配所用的先验地图路径。
- `num_threads` (int): 匹配计算使用的并行线程数。
- `max_dist_sq` (double): 匹配时的最大对应距离平方，影响收敛速度和精度。
- `global_leaf_size` (double): 先验地图的降采样大小。

## 7. loam_interface
**功能描述**: 接口转换模块，将 SLAM/LIO 输出的里程计和点云信息转换到机器人底盘坐标系下，供下游感知和导航使用。
**关键参数**:
- `state_estimation_topic` (string): 输入的位姿估计话题。
- `registered_scan_topic` (string): 输入的已配准点云话题。
- `odom_frame` / `base_frame` (string): 定义坐标系转换的目标名称。

## 8. sensor_scan_generation
**功能描述**: 坐标转换与扫描重构模块。它订阅 SLAM 发布的全局已配准点云 (`registered_scan`)，并利用里程计信息将其逆变换回机器人局部坐标系。
**作用**: 解决全局地图数据与局部感知算法之间的坐标冲突，为下游的避障和地形分析提供以机器人为中心的实时点云流 (`sensor_scan`)。
**关键参数**:
- `lidar_frame` (string): 雷达坐标系。
- `base_frame` (string): 机器人底盘坐标系。
- `robot_base_frame` (string): 辅助坐标系 (如云台)。

## 9. terrain_analysis
**功能描述**: 实时局部地形分析模块。通过对机器人周围的点云进行体素化（Voxelization）处理，分析地形的几何特征。
**核心算法**: 使用高度分位数 (`quantileZ`) 来提取地面高度，有效过滤动态障碍物（如行人和移动车辆）。
**关键参数**:
- `scanVoxelSize` (double, default: 0.05): 分析时的体素网格分辨率。
- `decayTime` (double): 历史点云的衰减/清理时间。
- `quantileZ` (double): 用于判定地面水平的高度百分位数，较小值倾向于探测地面。
- `minRelZ` / `maxRelZ` (double): 相对于机器人的分析高度限制。

## 10. terrain_analysis_ext
**功能描述**: 扩展范围的地形分析模块，在 `terrain_analysis` 的基础上增加了对更大尺度环境的建模和路径连通性检测。
**增强功能**: 具备地形连通性检查 (`checkTerrainConn`)，能够识别断崖、深坑等虽然表面平整但机器人无法跨越的地形风险。
**关键参数**:
- `localTerrainMapRadius` (double, default: 4.0): 局部地形图的分析半径。
- `checkTerrainConn` (bool): 开启后将执行地形连通性算法。
- `terrainConnThre` (double): 连通性判定的高度阈值。
- `ceilingFilteringThre` (double): 过滤天花板或上方障碍物的高度。
