import yaml

with open('/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/config/nav2_full_params.yaml', 'r') as f:
    data = yaml.safe_load(f)

footprint = "[[-0.3, -0.15], [-0.3, 0.15], [0.3, 0.15], [0.3, -0.15]]"

def update_costmap(cm_dict):
    if 'ros__parameters' not in cm_dict:
        return
    params = cm_dict['ros__parameters']
    params['footprint'] = footprint
    if 'robot_radius' in params:
        del params['robot_radius']
    if 'obstacle_layer' in params:
        params['obstacle_layer']['min_obstacle_height'] = -0.1
        params['obstacle_layer']['max_obstacle_height'] = 0.2
    if 'inflation_layer' in params:
        params['inflation_layer']['inflation_radius'] = 0.2
        params['inflation_layer']['cost_scaling_factor'] = 15.0

if 'local_costmap' in data and 'local_costmap' in data['local_costmap']:
    update_costmap(data['local_costmap']['local_costmap'])

if 'global_costmap' in data and 'global_costmap' in data['global_costmap']:
    update_costmap(data['global_costmap']['global_costmap'])

with open('/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/config/nav2_full_params.yaml', 'w') as f:
    yaml.dump(data, f)
