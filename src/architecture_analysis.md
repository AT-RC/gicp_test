# 系统架构分析

## 整体架构

你的系统是一个双层定位架构:Point-LIO 负责高频里程计(局部、丝滑、增量),GICP 负责对先验地图的全局重定位(纠偏、消除累积漂移)。两者通过 TF 树解耦,职责清晰:

```text
map  ──(GICP: small_gicp_relocalization, 20Hz)──▶  odom
odom ──(loam_interface, 从 /odometry 转换)──────▶  base_link
base_link ──(static_transform_publisher)────────▶  lidar
```

数据流:
```text
Livox MID360 ──▶ point_lio ──/odometry──▶ loam_interface ──/registered_scan──▶ GICP
                              (odom→base)         │                             (map→odom)
                                                  └──发布 odom→base_link TF
```

关键点:在 bringup 里 Point-LIO 的 `tf_send_en=False`,且 config 里 `prior_pcd.enable=False`。也就是说 Point-LIO 从原点开始纯里程计建图,不加载**先验地图(1);**GICP 单独负责把 odom 系对齐到先验 map 系。这是一个干净的设计——LIO 不被先验地图污染,重定位失败也不影响里程计本身。

---

## 1. bring_up (启动调度)

`bringup.launch.py` 和 `bringup_2.launch.py` 几乎完全相同,集成了 6 个子系统:livox 驱动、point_lio、GICP、loam_interface、map_monitor、static_tf、rviz(`odom_monitor` 和 `serial_node` 被注释掉了)。

两个文件的差异只有两处:

| 项 | bringup | bringup_2 (未提交) |
| --- | --- | --- |
| prior_pcd_file 默认值 | PCD/scans_1.pcd | PCD/scans_2.pcd |
| base_link→lidar 的 x 偏移 | -0.1 | -0.15 |

也就是说 `bringup_2` 是针对第二张地图(`scans_2.pcd`)的副本,并把雷达外参的 x 平移从 -10cm 改到 -15cm。

值得注意的几点:
- GICP 的 `update_min_translation=0.10` / `update_min_rotation=0.05` 在这里被硬编码下发,覆盖了 GICP launch 里 0.10/0.05 的默认值。
- 静态 TF 用四元数 `[0,0,1,0]` 表示绕 Z 轴 180°(雷达朝后装)。
- 复制粘贴维护两个 launch 文件容易漂移。更稳妥的做法是单一 launch + prior_pcd_file/lidar_x 两个参数化,在命令行切换。

---

## 2. GICP 重定位 (small_gicp_relocalization)

这是相对原版改动最大的模块,核心实现在 `small_gicp_relocalization.cpp`。原版只做"已知初始位姿后的连续配准",你扩展成了一个带全局搜索 + 退化检测 + 死区的状态机。

### 线程模型 (`SmallGicpRelocalizationNode` 构造函数)
- 一个独立 worker 线程(不占 ROS 线程),按 `continuous_update_rate` 节流,根据 `global_search_done_` 标志在两个状态间切换:
  - false → `performGlobalSearch()` (找回位姿)
  - true → `performRegistration()` (连续纠偏)
- 一个 20Hz 定时器 `publishTransform()` 始终发布 map→odom。
- `registeredPcdCallback` 只负责把点云累加进 `accumulated_cloud_`(加锁),计算和接收分离。

### 全局搜索 `performGlobalSearch()` (两阶段 coarse→fine)
1. 等累计点云 ≥ 15000 点才开始。
2. 粗搜索:在搜索区域内按 coarse_step 撒 x/y 网格 × coarse_yaw_samples 个偏航角,生成候选位姿,OpenMP 并行各跑 coarse_iters 次 GICP,用 (error + 未匹配点惩罚) / 总点数 打分,取 top-5。
3. 精搜索:对 top-5 用更细的 registered_leaf_size 和 fine_iters 次迭代,选最优。
4. `enable_global_search=false`(默认)时,搜索范围缩到上次位姿 ±2m(局部初始化);为 true 时用 map_filter_* 全图范围。

### 连续配准 `performRegistration()` 
有几个你加的工程化设计:
- 可信度评分(0~100):重叠率占 70% + RMSE 占 30%。
- 退化检测:对 Hessian result.H 求最小特征值,走廊等退化场景会被惩罚——这是个很好的设计,防止单帧约束不足时乱纠偏。
- 死区拦截:只有平移 > `update_min_translation` 或旋转 > `update_min_rotation` 才更新 TF,否则保持 Point-LIO 的丝滑轨迹。
- 丢失重定位:可信度 < 40 或不收敛累计 5 次,自动把 `global_search_done_` 置回 false,重新触发全局搜索。

### 我注意到的几个问题
1. **max_dist_sq 不一致**: GICP launch 默认 16.0,但节点构造函数 declare_parameter 默认 4.0。因为 launch 传了参数,实际生效 16.0——但这种默认值打架很容易在单独跑节点时踩坑。
2. **锁竞争**: `performRegistration` 里 `previous_result_t_` 的读取没加锁(`small_gicp_relocalization.cpp:311-313`)。它在 worker 线程读,`initialPoseCallback` 在 ROS 线程写,`pose_mutex_` 只在第 318 行更新时才加锁,读 `previous_result_t_.translation()` 做 diff 时没保护。实践中影响小,但严格说是 data race。
3. **节流问题**: `continuous_update_rate` 既控制全局搜索节流又控制连续配准节流,默认 0.5→2 秒一次。全局搜索阶段如果每 2 秒才试一次、还要攒够 15000 点,初始找回可能偏慢。可以考虑两个阶段用不同节流。

---

## 3. Point-LIO

基本是 HKU-Mars 原版 + 一个先验 PCD 加载的定制。定制点集中在三处:
1. **加载函数** `loadPointcloudFromPcd()` (`laserMapping.cpp:58`): 读 PCD 返回点云指针,失败返回 nullptr。
2. **初始位姿注入** (`laserMapping.cpp:249-262`): `enable_prior_pcd` && `is_first_kf` 时,第一帧把 `kf_output.x_.pos` 直接设为 config 里的 `init_pose[0..2]`(只设了 xyz,没设姿态),之后置 `is_first_kf=false`。
3. **用先验地图初始化 ivox** (`laserMapping.cpp:522-528`): 攒够 `init_map_size` 帧后,如果 `enable_prior_pcd` 就把先验 PCD 灌进 ivox 地图,否则用自身首帧点云。

新增帧参数(`parameters.cpp`): `map_frame` / `odom_frame` / `base_frame` 三个可配 frame,以及 `prior_pcd.enable` / `prior_pcd_map_path` / `init_pose`。

**关键观察**: 在当前 bringup 流程里这套先验 PCD 机制是关闭的(`mid360.yaml` 里 `prior_pcd.enable=False`)。重定位完全交给了 GICP 模块,Point-LIO 只做纯里程计。所以这条定制代码路径目前是 dead path——你有两套重定位方案(Point-LIO 内置 vs 外部 GICP),实际只用了 GICP 那套。

### 几个配置细节
- `use_imu_as_input=False` → 用 IMU 作为观测(Point-LIO 的高带宽模式,对 IMU 饱和鲁棒),`check_satu=True` 配合。注意:先验 PCD 的初始位姿注入代码只在 `!use_imu_as_input` 分支里(`laserMapping.cpp:249`),如果切到 input 模式这段就不生效。
- `extrinsic_T`: `[-0.011, -0.02329, 0.04412]` 是 IMU-雷达外参(MID360 内置 IMU)。
- `filter_size_surf=0.5`、`filter_size_map=0.2`、`ivox_grid_resolution=2.0`。

---

## 一句话总结

架构合理:Point-LIO 出高频里程计 → loam_interface 转换并发 odom→base_link → GICP 对先验地图算 map→odom 纠偏,三层 TF 解耦。GICP 模块是你下功夫最多的地方(全局搜索 + 退化检测 + 死区),工程化考虑到位。

可以收拾的几处:GICP 里 `max_dist_sq` 默认值打架、`previous_result_t_` 读取的 data race;Point-LIO 的先验 PCD 路径目前是关闭的冗余方案;以及两个 bringup 文件靠复制维护、容易漂移。
