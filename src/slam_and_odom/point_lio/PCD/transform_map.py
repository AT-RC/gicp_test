#!/usr/bin/env python3
import math
import argparse
import sys
import os
import numpy as np
import open3d as o3d


# ============ 赛场裁剪参数（转完之后的最终地图系，单位 m）============
# 赛场范围 X[0,6] Y[-4,0]，往外放 MARGIN 余量保证 0.8m 围栏点完整保留
COURT_MARGIN = 0.3
COURT_X_MIN = 0.0 - COURT_MARGIN
COURT_X_MAX = 6.0 + COURT_MARGIN
COURT_Y_MIN = -4.0 - COURT_MARGIN
COURT_Y_MAX = 0.0 + COURT_MARGIN
# 场外区：只保留高于此值的点（天花板），切掉人群
OUTDOOR_Z_MIN = 2.0
# ====================================================================


def crop_point_cloud(pcd):
    """对称裁剪：赛场区全保留，场外只留天花板"""
    points = np.asarray(pcd.points)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    in_court = (
        (x >= COURT_X_MIN) & (x <= COURT_X_MAX) &
        (y >= COURT_Y_MIN) & (y <= COURT_Y_MAX)
    )
    high_enough = z > OUTDOOR_Z_MIN

    keep = in_court | high_enough
    kept = np.count_nonzero(keep)
    removed = len(points) - kept
    print(f"裁剪结果: 保留 {kept} 点, 删除 {removed} 点 (场外低空)")
    return pcd.select_by_index(np.where(keep)[0])


def main():
    parser = argparse.ArgumentParser(
        description="变换点云地图（旋转+平移），可选对称裁剪去除场外动态干扰")
    parser.add_argument("input", help="输入点云文件 (例如: map.pcd)")
    parser.add_argument("output", help="输出点云文件 (例如: new_map.pcd)")
    parser.add_argument("angle", type=float, help="绕Z轴旋转的角度 (度数，逆时针为正)")
    parser.add_argument("x", type=float, help="基于旋转后的新X轴平移距离")
    parser.add_argument("y", type=float, help="基于旋转后的新Y轴平移距离")
    parser.add_argument("crop", choices=["t", "f"],
                        help="是否裁剪场外动态区域: t=裁剪, f=不裁剪 (必填)")

    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"找不到输入文件: {args.input}")
        sys.exit(1)

    # 读入点云
    pcd = o3d.io.read_point_cloud(args.input)
    print(f"加载点云: {len(pcd.points)} 点")

    # 构造 4x4 变换矩阵
    theta = math.radians(args.angle)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    T = np.array([
        [cos_t, -sin_t, 0, args.x],
        [sin_t,  cos_t, 0, args.y],
        [0,      0,     1, 0],
        [0,      0,     0, 1]
    ])

    print(f"旋转角度: {args.angle}")
    print(f"平移: X={args.x}, Y={args.y}")

    # 变换
    pcd.transform(T)

    # 裁剪
    if args.crop == "t":
        print(f"执行对称裁剪 (赛场区: X[{COURT_X_MIN},{COURT_X_MAX}] Y[{COURT_Y_MIN},{COURT_Y_MAX}], 场外保留 Z>{OUTDOOR_Z_MIN})")
        pcd = crop_point_cloud(pcd)

    # 保存
    o3d.io.write_point_cloud(args.output, pcd)
    print(f"保存完成: {args.output} ({len(pcd.points)} 点)")
    print(f"查看效果: pcl_viewer {args.output} -ax -1")


if __name__ == "__main__":
    main()
