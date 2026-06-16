#!/usr/bin/env python3
import math
import argparse
import subprocess
import sys
import os

def main():
    parser = argparse.ArgumentParser(description="根据旋转角度(Z轴)和新坐标系下的平移(x, y)变换点云地图")
    parser.add_argument("input", help="输入点云文件 (例如: map.pcd)")
    parser.add_argument("output", help="输出点云文件 (例如: new_map.pcd)")
    parser.add_argument("angle", type=float, help="绕Z轴旋转的角度 (度数，逆时针为正)")
    parser.add_argument("x", type=float, help="基于旋转后的新X轴平移距离")
    parser.add_argument("y", type=float, help="基于旋转后的新Y轴平移距离")

    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"❌ 找不到输入文件: {args.input}")
        sys.exit(1)

    # 角度转弧度
    theta = math.radians(args.angle)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # 1. 计算旋转矩阵 R_z
    # 2. 计算在新坐标系下平移 (x, y) 对应的全局平移
    # t_global = R_z * [x, y, 0]^T
    t_global_x = cos_t * args.x - sin_t * args.y
    t_global_y = sin_t * args.x + cos_t * args.y
    
    # 3. 拼接为 4x4 变换矩阵的字符串 (按行排列)
    matrix_str = f"{cos_t},{-sin_t},0,{t_global_x}," \
                 f"{sin_t},{cos_t},0,{t_global_y}," \
                 f"0,0,1,0," \
                 f"0,0,0,1"

    print(f"🔹 旋转角度: {args.angle}°")
    print(f"🔹 新坐标系下平移: X={args.x}, Y={args.y}")
    print(f"🔹 计算得到的全局平移: X={t_global_x:.4f}, Y={t_global_y:.4f}")
    
    # 调用 pcl_transform_point_cloud
    cmd = [
        "pcl_transform_point_cloud",
        args.input,
        args.output,
        "-matrix",
        matrix_str
    ]

    print(f"\n执行底层命令:\n{' '.join(cmd)}\n")
    try:
        subprocess.run(cmd, check=True)
        print(f"\n✅ 转换成功！新地图已保存至: {args.output}")
        print(f"运行以下命令查看效果: pcl_viewer {args.output} -ax -1")
    except subprocess.CalledProcessError:
        print(f"\n❌ 运行失败，请检查文件格式或 PCL 工具。")
        sys.exit(1)
    except FileNotFoundError:
        print(f"\n❌ 找不到命令 pcl_transform_point_cloud，请确保已安装 pcl-tools。")
        sys.exit(1)

if __name__ == "__main__":
    main()
