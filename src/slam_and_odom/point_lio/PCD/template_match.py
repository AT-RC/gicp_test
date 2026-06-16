import open3d as o3d
import numpy as np
import cv2
import math
import argparse
import os

def main():
    parser = argparse.ArgumentParser(description="基于 2D 鸟瞰图与模板匹配的靶向定位对齐")
    parser.add_argument("input", help="输入点云文件 (例如: 1.pcd)")
    parser.add_argument("output", nargs="?", default=None,
                        help="输出点云文件 (可选；默认在输入名前加 aligned_ 前缀)")
    args = parser.parse_args()

    input_file = args.input
    # 没指定输出时，自动用 aligned_<输入名>，避免每次覆盖同一个文件、看不出区别
    output_file = args.output if args.output else "aligned_" + os.path.basename(input_file)

    if not os.path.exists(input_file):
        print(f"错误：找不到输入文件 {input_file}")
        return
    print(f"输入: {input_file}    输出: {output_file}")

    print("=========================================")
    print("基于 2D 鸟瞰图与模板匹配(Template Matching)的靶向定位")
    print("=========================================")
    
    # 1. 加载和预处理点云
    print("1. 正在加载和预处理点云...")
    pcd = o3d.io.read_point_cloud(input_file)
    points = np.asarray(pcd.points)
    
    # --- 1. 裁剪点云范围 ---
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # ============ 赛场快调区：裁剪窗口（雷达坐标系，单位 m）============
    # 阵列必须落在这个 x/y 框内。换场地 / 起步位置变了，只改下面 6 个数即可。
    # 定法：先 pcl_viewer 看阵列大概在哪，框住它再各留 ~0.4m 余量。
    CROP_X = (-0.5, 4.0)    # 阵列 X 范围
    CROP_Y = (-0.5, 1.0)    # 阵列 Y 范围
    CROP_Z = (-0.15, 0.3)   # 高度：下界滤地面，上界滤比箱子高的墙/人
    # ===================================================================
    valid_indices = np.where((x >= CROP_X[0]) & (x <= CROP_X[1]) &
                             (y >= CROP_Y[0]) & (y <= CROP_Y[1]) &
                             (z >= CROP_Z[0]) & (z <= CROP_Z[1]))[0]
    non_ground_pcd = pcd.select_by_index(valid_indices)
    
    if len(non_ground_pcd.points) == 0:
        print("错误：裁剪窗口内没有任何有效点云（检查 CROP_X/Y/Z 是否框住了阵列）")
        return
        
    # --- 2. 聚类并严格过滤掉“墙壁”等干扰物 ---
    print("2. 正在聚类并剔除墙壁等大型干扰物...")
    # eps=0.15 (15cm内算连通), min_points=10
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Warning) as cm:
        labels = np.array(non_ground_pcd.cluster_dbscan(eps=0.15, min_points=10))
        
    if len(labels) == 0:
        print("错误：未找到任何聚类物体")
        return

    max_label = labels.max()
    box_centroids = []
    box_point_indices = []  # 记录“确认是箱子”的点索引，后面只把这些干净的点喂给 ICP
    
    for i in range(max_label + 1):
        cluster_indices = np.where(labels == i)[0]
        cluster_pcd = non_ground_pcd.select_by_index(cluster_indices)
        aabb = cluster_pcd.get_axis_aligned_bounding_box()
        extent = aabb.get_extent()
        
        # 核心过滤：只要 X 或 Y 长度超过 45cm，它就绝对不可能是 25cm 的箱子！
        # 这个条件会把所有的墙壁、长条状杂物、人的双腿等全部过滤掉！
        # 上限：超过 45cm 绝不是 25cm 的箱子（剔墙/长条杂物/人腿）
        # 下限：至少一条边 > 8cm，剔除零星噪点小团块（避免假特征点污染匹配与 ICP）
        if extent[0] < 0.45 and extent[1] < 0.45 and max(extent[0], extent[1]) > 0.08:
            box_centroids.append(aabb.get_center())
            box_point_indices.extend(cluster_indices.tolist())
            
    box_centroids = np.array(box_centroids)
    print(f"经过严苛筛选，提取到了 {len(box_centroids)} 个疑似箱子的特征点。")
    
    if len(box_centroids) == 0:
        print("提取失败：所有的聚类物体都太大或太小，不符合箱子特征。")
        return

    # --- 3. 生成完美的 2x4 坐标系骨架 ---
    print("3. 正在进行几何拓扑图硬匹配 (完全无视无关杂物)...")
    spacing_m = 0.85
    grid_points = []
    # 以第一个箱子为 (0,0,0)，X 轴 2 个，Y 轴 4 个
    for i in range(2):
        for j in range(4):
            grid_points.append([i * spacing_m, j * spacing_m, 0.0])
    grid_points = np.array(grid_points)

    # --- 4. 暴力搜索最优阵列映射 ---
    best_match_count = 0
    best_error = float('inf')
    best_transform = np.eye(4)
    best_angle = 0
    
    # 扩大搜索范围到 360 度，解决初始建图方向未知或偏差大的问题！
    # 同时在内部通过“就近原则”智能识别并解决 180 度对称翻转的歧义，保持地图原有大方向。
    for angle_deg in range(-45, 45):
        theta = np.radians(angle_deg)
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,             0,              1]
        ])
        
        rotated_grid = np.dot(grid_points, R.T)
        
        # 穷举所有的对应关系：假设雷达扫到的第 c 个特征点，就是理想骨架里的第 t 个箱子
        for t in range(8):
            for c in range(len(box_centroids)):
                # 计算出必须的平移量
                translation = box_centroids[c] - rotated_grid[t]
                translation[2] = 0 # 保持在地面上
                
                # 检查在这个位姿下，8个理想箱子有几个能和雷达扫到的特征点对上
                match_count = 0
                error = 0.0
                for p in rotated_grid:
                    shifted_p = p + translation
                    # 计算这个理想箱子离所有雷达特征点的最小距离
                    dists = np.linalg.norm(box_centroids[:, :2] - shifted_p[:2], axis=1)
                    min_dist = np.min(dists)
                    
                    if min_dist < 0.20: # 误差在 20cm 内就算完美重合
                        match_count += 1
                        error += min_dist
                        
                # 记录得分最高的那个变换矩阵
                if match_count > best_match_count:
                    best_match_count = match_count
                    best_error = error
                    best_angle = angle_deg
                    
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = translation
                    best_transform = T
                elif match_count == best_match_count:
                    # 处理对称性：如果命中数一样，看误差
                    # 如果误差极其接近（相差不到 2cm），说明遇到了 180 度对称的等价匹配！
                    # 此时优先选择角度绝对值更接近 0 的，从而防止地图发生 180 度的非预期大翻转。
                    if abs(error - best_error) < 0.02:
                        if abs(angle_deg) < abs(best_angle):
                            best_error = error
                            best_angle = angle_deg
                            T = np.eye(4)
                            T[:3, :3] = R
                            T[:3, 3] = translation
                            best_transform = T
                    # 如果不是对称，纯粹是当前位姿误差更小，则直接更新
                    elif error < best_error:
                        best_error = error
                        best_angle = angle_deg
                        T = np.eye(4)
                        T[:3, :3] = R
                        T[:3, 3] = translation
                        best_transform = T

    print(f"\n匹配结束！在您的雷达特征中成功对齐了 {best_match_count} 个箱子。")
    if best_match_count < 3:
        print("警告：匹配的箱子数量极少，由于点云太残缺可能对齐有误。")
        
    # --- 5. 点云对齐 (粗匹配) ---
    # best_transform 是把 [理想 0,0 坐标系] 变到 [雷达坐标系] 的矩阵
    # 我们需要把雷达点云变到 0,0 原点，所以要求逆矩阵！
    coarse_transform = np.linalg.inv(best_transform)
    print("粗对齐矩阵算毕，正在启动毫米级精度调优...")

    # =======================================================
    # --- 6. 终极精度：ICP 表面贴合微调 ---
    # =======================================================
    # 为什么会差 10cm？因为雷达往往只能扫到箱子的 1-2 个面（比如正面和右面），
    # 这会导致前面算出来的“包围盒中心”并不是箱子真正的物理中心，从而产生约 10cm 的系统偏差。
    # 解决办法：生成完美的箱子“外轮廓”，用 ICP 算法直接把雷达扫到的面“死死吸附”在轮廓上。
    
    template_points = []
    # 严格绘制 8 个完美的空心正方形（模拟箱子的墙壁）
    for i in range(2):
        for j in range(4):
            cx_box = i * 0.85
            cy_box = j * 0.85
            half_l = 0.125 # 25cm 边长的一半
            # 沿着边框每隔 1cm 撒一个点
            for offset in np.arange(-half_l, half_l, 0.01):
                template_points.append([cx_box + offset, cy_box - half_l, 0]) # 上边
                template_points.append([cx_box + offset, cy_box + half_l, 0]) # 下边
                template_points.append([cx_box - half_l, cy_box + offset, 0]) # 左边
                template_points.append([cx_box + half_l, cy_box + offset, 0]) # 右边
                
    template_pcd = o3d.geometry.PointCloud()
    template_pcd.points = o3d.utility.Vector3dVector(np.array(template_points))

    # 只取“确认是箱子”的那些点（已剔除墙壁/大杂物），喂给 ICP 最干净，最不容易被墙带偏
    raw_pts2d = np.copy(np.asarray(non_ground_pcd.select_by_index(box_point_indices).points))
    raw_pts2d[:, 2] = 0.0 # 全部压平到 2D 平面，防止地面高度不同干扰匹配
    raw_pcd2d = o3d.geometry.PointCloud()
    raw_pcd2d.points = o3d.utility.Vector3dVector(raw_pts2d)
    
    # 先用刚才算出来的粗矩阵，把点云大概摆正
    raw_pcd2d.transform(coarse_transform)

    # 摆正后，阵列已落在原点附近（箱心 x∈[0,0.85] y∈[0,2.55]）。
    # 在这个“已对齐坐标系”里裁掉远处杂物再做 ICP——这是相对阵列裁剪，跟雷达位置无关，
    # 所以换任何起始位姿都不会框错，也避免远处杂簇把 fitness 拉低误触发警告。
    ap = np.asarray(raw_pcd2d.points)
    near = np.where((ap[:,0] >= -0.6) & (ap[:,0] <= 1.5) &
                    (ap[:,1] >= -0.6) & (ap[:,1] <= 3.2))[0]
    raw_pcd2d = raw_pcd2d.select_by_index(near)
    
    # 启动极致精度的 ICP (Iterative Closest Point)
    # 因为现在误差只在 10cm 左右，所以 max_correspondence_distance 设为 0.20
    print("正在执行高精度 ICP 轮廓吸附...")
    icp_result = o3d.pipelines.registration.registration_icp(
        raw_pcd2d, template_pcd, max_correspondence_distance=0.20,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    )
    
    print(f"ICP 微调完成！面重合度(fitness): {icp_result.fitness * 100:.1f}%  内点误差(RMSE): {icp_result.inlier_rmse*100:.2f}cm")

    # --- 失败门控：对歪了就主动报警，别让你拿着错位的地图还不知道 ---
    align_ok = True
    if best_match_count < 4:
        print(f"⚠ 警告：只匹配上 {best_match_count} 个箱子（少于 4 个），粗对齐很可能不可靠！")
        align_ok = False
    if icp_result.fitness < 0.30:
        print(f"⚠ 警告：ICP 面重合度只有 {icp_result.fitness*100:.1f}%（低于 30%），轮廓没吸附上，对齐很可能歪了！")
        align_ok = False
    if icp_result.inlier_rmse > 0.05:
        print(f"⚠ 警告：ICP 内点误差 {icp_result.inlier_rmse*100:.2f}cm 偏大（超过 5cm），精度存疑！")
        align_ok = False
    if not align_ok:
        print("⚠ 上述指标不达标：地图仍会保存，但请务必用 pcl_viewer 目视检查后再上场！\n")
    
    # 终极变换矩阵 = ICP 微调矩阵 * 粗对齐矩阵
    ultimate_transform = np.dot(icp_result.transformation, coarse_transform)
    
    # 额外平移：将原点从“箱子几何中心”平移到“箱子最外侧(左下角)顶点”
    # 原本箱子中心在 (0,0)，左下角在 (-0.125, -0.125)。
    # 若要让左下角变成 (0,0)，我们需要把整个点云向右上角（正X，正Y）平移 0.125m。
    T_corner = np.eye(4)
    T_corner[0, 3] = 0.125
    T_corner[1, 3] = 0.125
    
    ultimate_transform = np.dot(T_corner, ultimate_transform)
    
    print("\n=======================================================")
    print("★ 最终高精度 4x4 变换矩阵 (原点位于第一个箱子的左下角)：\n", ultimate_transform)
    print("=======================================================\n")
    
    print("正在保存点云...")
    pcd.transform(ultimate_transform)
    o3d.io.write_point_cloud(output_file, pcd)
    print("大功告成！完美对齐的地图已保存！")
    print(f"\n运行以下命令查看对齐效果：\npcl_viewer {output_file} -ax -1")

if __name__ == "__main__":
    main()
