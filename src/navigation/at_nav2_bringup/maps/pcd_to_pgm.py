import open3d as o3d
import numpy as np
import yaml
import os

pcd_path = "/home/xjh/Desktop/at_rc/src/slam_and_odom/point_lio/PCD/new_map4.pcd"
output_pgm = "/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/maps/keepout_mask.pgm"
output_yaml = "/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/maps/keepout_mask.yaml"

resolution = 0.05
z_min = -0.1
z_max = 0.3

print(f"Loading {pcd_path}...")
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)

# Filter by Z
valid_idx = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
points = points[valid_idx]

if len(points) == 0:
    print("No points found in the specified Z range!")
    exit(1)

# Get X and Y bounds
x_min, x_max = points[:, 0].min(), points[:, 0].max()
y_min, y_max = points[:, 1].min(), points[:, 1].max()

# Compute grid size
width = int(np.ceil((x_max - x_min) / resolution)) + 1
height = int(np.ceil((y_max - y_min) / resolution)) + 1

# Create an empty grid (255 = free space/white)
grid = np.full((height, width), 255, dtype=np.uint8)

# Map points to grid cells
# Note: in PGM, (0,0) is usually top-left, but in ROS maps origin is bottom-left
col_indices = np.floor((points[:, 0] - x_min) / resolution).astype(int)
row_indices = np.floor((points[:, 1] - y_min) / resolution).astype(int)

# Invert Y to match PGM image coordinates (top-left origin for the image)
# Actually, ROS map_server reads image. The pixel at (0, height-1) is the origin.
# So row_indices from point cloud (where higher Y means "up" in world) 
# must map to PGM coordinates where Y=0 is the top row, and Y=height-1 is the bottom row.
img_row_indices = height - 1 - row_indices

# Mark obstacles as 0 (black)
grid[img_row_indices, col_indices] = 0

# Save PGM
with open(output_pgm, 'wb') as f:
    f.write(b"P5\n")
    f.write(f"{width} {height}\n".encode('ascii'))
    f.write(b"255\n")
    f.write(grid.tobytes())

print(f"Saved {output_pgm}")

# Save YAML matching ROS map_server format
origin_x = float(x_min)
origin_y = float(y_min)

yaml_data = {
    "image": "keepout_mask.pgm",
    "mode": "scale",   # Use scale mode for Keepout Filter
    "resolution": resolution,
    "origin": [origin_x, origin_y, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}

with open(output_yaml, 'w') as f:
    yaml.dump(yaml_data, f, default_flow_style=False)

print(f"Saved {output_yaml}")
