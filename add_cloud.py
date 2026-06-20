import yaml

with open('/home/xjh/Desktop/at_rc/src/bring_up/rviz/reloc.rviz', 'r') as f:
    config = yaml.safe_load(f)

# Find the PointCloud2 display for /global_map and copy it for /odin1/cloud_slam
displays = config['Visualization Manager']['Displays']
new_display = None
for d in displays:
    if d['Class'] == 'rviz_default_plugins/PointCloud2':
        new_display = d.copy()
        break

if new_display:
    new_display['Topic']['Value'] = '/odin1/cloud_slam'
    new_display['Name'] = 'Odin_Cloud'
    # Change color to red
    new_display['Color'] = '255; 0; 0'
    displays.append(new_display)

with open('/home/xjh/Desktop/at_rc/src/bring_up/rviz/reloc.rviz', 'w') as f:
    yaml.dump(config, f)
