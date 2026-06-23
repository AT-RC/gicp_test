import yaml

def update_yaml():
    with open('/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/config/nav2_full_params.yaml', 'r') as f:
        data = yaml.safe_load(f)

    # Footprint updates
    footprint = "[[-0.3, -0.15], [-0.3, 0.15], [0.3, 0.15], [0.3, -0.15]]"
    if 'local_costmap' in data and 'local_costmap' in data['local_costmap']['ros__parameters']:
        data['local_costmap']['local_costmap']['ros__parameters']['footprint'] = footprint
        # If it had robot_radius, remove it
        if 'robot_radius' in data['local_costmap']['local_costmap']['ros__parameters']:
            del data['local_costmap']['local_costmap']['ros__parameters']['robot_radius']
            
    if 'global_costmap' in data and 'global_costmap' in data['global_costmap']['ros__parameters']:
        data['global_costmap']['global_costmap']['ros__parameters']['footprint'] = footprint
        if 'robot_radius' in data['global_costmap']['global_costmap']['ros__parameters']:
            del data['global_costmap']['global_costmap']['ros__parameters']['robot_radius']

    # Update Z heights for obstacle layer (from our tuning: min -0.1, max 0.2)
    for costmap in ['local_costmap', 'global_costmap']:
        try:
            layers = data[costmap][costmap]['ros__parameters']['obstacle_layer']
            layers['min_obstacle_height'] = -0.1
            layers['max_obstacle_height'] = 0.2
        except KeyError:
            pass

    # Update inflation layer
    for costmap in ['local_costmap', 'global_costmap']:
        try:
            layers = data[costmap][costmap]['ros__parameters']['inflation_layer']
            layers['inflation_radius'] = 0.2
            layers['cost_scaling_factor'] = 15.0
        except KeyError:
            pass

    with open('/home/xjh/Desktop/at_rc/src/navigation/at_nav2_bringup/config/nav2_full_params.yaml', 'w') as f:
        yaml.dump(data, f)

update_yaml()
