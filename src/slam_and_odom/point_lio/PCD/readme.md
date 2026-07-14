Please keep the `PCD` folder. Do NOT delete it.

This ensures that there will be no errors when saving the .pcd file.

python3 transform_map.py scans.pcd fin11.pcd 270 0 0 f
python3 template_match.py fin11.pcd fin12.pcd
python3 transform_map.py fin12.pcd fin13.pcd 270 1.325 -0.623 f