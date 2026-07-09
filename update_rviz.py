import yaml

with open('/home/xjh/Desktop/at_rc/src/bring_up/rviz/airy.rviz', 'r') as f:
    data = yaml.safe_load(f)

# Add Local Costmap
local_costmap = {
    'Alpha': 0.7,
    'Class': 'rviz_default_plugins/Map',
    'Color Scheme': 'costmap',
    'Draw Behind': False,
    'Enabled': True,
    'Name': 'Local Costmap',
    'Topic': {'Depth': 5, 'Durability Policy': 'Volatile', 'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable', 'Value': '/local_costmap/costmap'},
    'Update Topic': {'Depth': 5, 'Durability Policy': 'Volatile', 'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable', 'Value': '/local_costmap/costmap_updates'},
    'Use Timestamp': False,
    'Value': True
}

# Add Global Costmap
global_costmap = {
    'Alpha': 0.7,
    'Class': 'rviz_default_plugins/Map',
    'Color Scheme': 'costmap',
    'Draw Behind': False,
    'Enabled': False,
    'Name': 'Global Costmap',
    'Topic': {'Depth': 5, 'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable', 'Value': '/global_costmap/costmap'},
    'Update Topic': {'Depth': 5, 'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable', 'Value': '/global_costmap/costmap_updates'},
    'Use Timestamp': False,
    'Value': False
}

# Add Path
plan = {
    'Alpha': 1.0,
    'Class': 'rviz_default_plugins/Path',
    'Color': '0, 255, 0',
    'Enabled': True,
    'Head Diameter': 0.3,
    'Head Length': 0.2,
    'Length': 0.3,
    'Line Style': 'Lines',
    'Line Width': 0.05,
    'Name': 'Nav2 Plan',
    'Radius': 0.03,
    'Shaft Diameter': 0.1,
    'Shaft Length': 0.1,
    'Topic': {'Depth': 5, 'Durability Policy': 'Volatile', 'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable', 'Value': '/plan'},
    'Value': True
}

# Check if already exists to prevent duplicate
displays = data.get('Visualization Manager', {}).get('Displays', [])
names = [d.get('Name') for d in displays]

if 'Local Costmap' not in names:
    displays.append(local_costmap)
if 'Global Costmap' not in names:
    displays.append(global_costmap)
if 'Nav2 Plan' not in names:
    displays.append(plan)

data['Visualization Manager']['Displays'] = displays

with open('/home/xjh/Desktop/at_rc/src/bring_up/rviz/airy.rviz', 'w') as f:
    yaml.dump(data, f, default_flow_style=False)

print("RViz updated successfully!")
