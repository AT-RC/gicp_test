import open3d as o3d
import numpy as np
import cv2
import math
import argparse
import os

BOX_SIZE_M = 0.25
BOX_HALF_M = BOX_SIZE_M / 2.0
BOX_CLUSTER_MAX_EXTENT_M = 0.75
BOX_CLUSTER_MIN_POINTS = 100

def build_grid_points(nx, ny, spacing_m):
    grid_points = []
    for i in range(nx):
        for j in range(ny):
            grid_points.append([i * spacing_m, j * spacing_m, 0.0])
    return np.array(grid_points)

def distance_to_box_perimeters(points_xy, layout_x, layout_y, spacing_m, half_l):
    best_dist = np.full(points_xy.shape[0], np.inf)
    for i in range(layout_x):
        for j in range(layout_y):
            center = np.array([i * spacing_m, j * spacing_m])
            d = np.abs(points_xy - center)
            outside = np.maximum(d - half_l, 0.0)
            outside_dist = np.linalg.norm(outside, axis=1)
            inside = (d[:, 0] <= half_l) & (d[:, 1] <= half_l)
            inside_dist = np.minimum(half_l - d[:, 0], half_l - d[:, 1])
            dist = np.where(inside, inside_dist, outside_dist)
            best_dist = np.minimum(best_dist, dist)
    return best_dist

def extract_box_features(pcd, points, crop_x, crop_y, crop_z, eps):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    valid_indices = np.where((x >= crop_x[0]) & (x <= crop_x[1]) &
                             (y >= crop_y[0]) & (y <= crop_y[1]) &
                             (z >= crop_z[0]) & (z <= crop_z[1]))[0]
    non_ground_pcd = pcd.select_by_index(valid_indices)

    if len(non_ground_pcd.points) == 0:
        return non_ground_pcd, np.empty((0, 3)), [], 0, 0

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Warning):
        labels = np.array(non_ground_pcd.cluster_dbscan(eps=eps, min_points=10))

    valid_labels = labels[labels >= 0]
    cluster_count = int(valid_labels.max() + 1) if len(valid_labels) else 0
    box_centroids = []
    box_point_indices = []

    for i in range(cluster_count):
        cluster_indices = np.where(labels == i)[0]
        cluster_pcd = non_ground_pcd.select_by_index(cluster_indices)
        aabb = cluster_pcd.get_axis_aligned_bounding_box()
        extent = aabb.get_extent()

        # 真实箱子边长是 25cm，但建图点云会因为边缘/连接点膨胀；
        # 这里的 75cm 只是聚类包络上限，用来保留箱子簇，不代表箱子物理尺寸。
        if (
            len(cluster_indices) >= BOX_CLUSTER_MIN_POINTS and
            extent[0] < BOX_CLUSTER_MAX_EXTENT_M and extent[1] < BOX_CLUSTER_MAX_EXTENT_M and
            max(extent[0], extent[1]) > 0.08
        ):
            box_centroids.append(aabb.get_center())
            box_point_indices.append(cluster_indices.tolist())

    return non_ground_pcd, np.array(box_centroids), box_point_indices, len(valid_indices), cluster_count

def match_grid(box_centroids, spacing_m):
    best = {
        "match_count": 0,
        "error": float('inf'),
        "transform": np.eye(4),
        "angle": 0,
        "layout": (2, 4),
        "grid_points": build_grid_points(2, 4, spacing_m),
    }

    if len(box_centroids) == 0:
        return best

    # 同时尝试 2x4 与 4x2。不同地图的阵列长边可能落在不同坐标轴上。
    for layout in [(2, 4), (4, 2)]:
        grid_points = build_grid_points(layout[0], layout[1], spacing_m)

        for angle_deg in range(-45, 45):
            theta = np.radians(angle_deg)
            R = np.array([
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta),  np.cos(theta), 0],
                [0,             0,              1]
            ])

            rotated_grid = np.dot(grid_points, R.T)

            for t in range(len(grid_points)):
                for c in range(len(box_centroids)):
                    translation = box_centroids[c] - rotated_grid[t]
                    translation[2] = 0

                    match_count = 0
                    error = 0.0
                    for p in rotated_grid:
                        shifted_p = p + translation
                        dists = np.linalg.norm(box_centroids[:, :2] - shifted_p[:2], axis=1)
                        min_dist = np.min(dists)

                        if min_dist < 0.20:
                            match_count += 1
                            error += min_dist

                    is_better = match_count > best["match_count"]
                    if match_count == best["match_count"]:
                        if abs(error - best["error"]) < 0.02:
                            is_better = abs(angle_deg) < abs(best["angle"])
                        else:
                            is_better = error < best["error"]

                    if is_better:
                        T = np.eye(4)
                        T[:3, :3] = R
                        T[:3, 3] = translation
                        best = {
                            "match_count": match_count,
                            "error": error,
                            "transform": T,
                            "angle": angle_deg,
                            "layout": layout,
                            "grid_points": grid_points,
                        }

    return best

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
    
    # ============ 赛场快调区：裁剪窗口（雷达坐标系，单位 m）============
    # 阵列必须落在这个 x/y 框内。换场地 / 起步位置变了，只改下面 6 个数即可。
    # 定法：先 pcl_viewer 看阵列大概在哪，框住它再各留 ~0.4m 余量。
    CROP_X = (0.0, 4.0)    # 阵列 X 范围
    CROP_Y = (0.5, 3.0)    # 阵列 Y 范围
    CROP_Z_CANDIDATES = [
        (-0.15, 0.30),  # 兼容旧地图：箱子点偏低
        (-0.05, 0.40),
        (0.00, 0.30),
        (0.03, 0.35),
        (0.05, 0.35),   # 过滤低处连接带
        (0.10, 0.40),   # 兼容 fin11：箱子底部/地面点会把簇粘在一起
        (0.20, 0.30),   # 高处切片：更容易把 25cm 箱子从底部连接点中分开
        (0.20, 0.35),
        (0.22, 0.35),
    ]
    DBSCAN_EPS_CANDIDATES = [0.035, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15]
    # ===================================================================

    # --- 2. 聚类并严格过滤掉“墙壁”等干扰物 ---
    print("2. 正在聚类并剔除墙壁等大型干扰物...")
    spacing_m = 0.85
    candidates = []
    for crop_z in CROP_Z_CANDIDATES:
        for eps in DBSCAN_EPS_CANDIDATES:
            non_ground, centroids, point_indices, crop_count, cluster_count = extract_box_features(
                pcd, points, CROP_X, CROP_Y, crop_z, eps
            )
            match = match_grid(centroids, spacing_m)
            candidates.append({
                "crop_z": crop_z,
                "eps": eps,
                "non_ground": non_ground,
                "centroids": centroids,
                "point_indices": point_indices,
                "crop_count": crop_count,
                "cluster_count": cluster_count,
                "match": match,
            })
            print(f"  Z={crop_z}, eps={eps:.3f}: 裁剪 {crop_count} 点, 聚类 {cluster_count} 个, "
                  f"箱子候选 {len(centroids)} 个, 可匹配 {match['match_count']} 个, "
                  f"模板 {match['layout'][0]}x{match['layout'][1]}")

    candidates.sort(key=lambda item: (
        item["match"]["match_count"],
        -item["match"]["error"],
        -abs(len(item["centroids"]) - 8)
    ), reverse=True)
    selected = candidates[0]
    non_ground_pcd = selected["non_ground"]
    box_centroids = selected["centroids"]
    best_match = selected["match"]
    best_match_count = best_match["match_count"]
    best_error = best_match["error"]
    best_transform = best_match["transform"]
    best_angle = best_match["angle"]
    grid_points = best_match["grid_points"]
    layout_x, layout_y = best_match["layout"]

    matched_grid = np.dot(grid_points, best_transform[:3, :3].T) + best_transform[:3, 3]
    matched_box_indices = []
    for i, centroid in enumerate(box_centroids):
        dists = np.linalg.norm(matched_grid[:, :2] - centroid[:2], axis=1)
        if np.min(dists) < 0.20:
            matched_box_indices.extend(selected["point_indices"][i])
    if not matched_box_indices:
        for indices in selected["point_indices"]:
            matched_box_indices.extend(indices)
    box_point_indices = matched_box_indices

    print(f"采用 Z={selected['crop_z']}，eps={selected['eps']:.2f}，提取到 {len(box_centroids)} 个疑似箱子，"
          f"使用 {layout_x}x{layout_y} 模板。")

    if len(box_centroids) == 0:
        print("提取失败：所有裁剪窗口里都没有符合箱子尺寸的聚类物体。")
        return

    # --- 3. 选择最匹配的 2x4 / 4x2 坐标系骨架 ---
    print("3. 正在进行几何拓扑图硬匹配 (完全无视无关杂物)...")

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
    for i in range(layout_x):
        for j in range(layout_y):
            cx_box = i * 0.85
            cy_box = j * 0.85
            half_l = BOX_HALF_M
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

    # 摆正后，阵列已落在原点附近。
    # 在这个“已对齐坐标系”里裁掉远处杂物再做 ICP——这是相对阵列裁剪，跟雷达位置无关，
    # 所以换任何起始位姿都不会框错，也避免远处杂簇把 fitness 拉低误触发警告。
    ap = np.asarray(raw_pcd2d.points)
    near = np.where((ap[:,0] >= -0.6) & (ap[:,0] <= (layout_x - 1) * spacing_m + 0.6) &
                    (ap[:,1] >= -0.6) & (ap[:,1] <= (layout_y - 1) * spacing_m + 0.6))[0]
    raw_pcd2d = raw_pcd2d.select_by_index(near)

    # 箱子真实边长是 25cm。粗对齐后只保留靠近理想箱子边框的点做 ICP，
    # 避免膨胀簇里的连接带/杂点把 25cm 轮廓吸偏。
    ap = np.asarray(raw_pcd2d.points)
    edge_dist = distance_to_box_perimeters(ap[:, :2], layout_x, layout_y, spacing_m, BOX_HALF_M)
    edge_indices = np.where(edge_dist <= 0.18)[0]
    if len(edge_indices) >= 50:
        raw_pcd2d = raw_pcd2d.select_by_index(edge_indices)
    
    # 启动极致精度的 ICP (Iterative Closest Point)
    # 因为现在误差只在 10cm 左右，所以 max_correspondence_distance 设为 0.20
    print("正在执行高精度 ICP 轮廓吸附...")
    icp_result = o3d.pipelines.registration.registration_icp(
        raw_pcd2d, template_pcd, max_correspondence_distance=0.15,
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
