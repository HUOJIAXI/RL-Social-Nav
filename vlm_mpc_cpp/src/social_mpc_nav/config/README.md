# Configuration Files Guide

This directory contains YAML configuration files for the social MPC navigation system.

## Available Config Files

### 1. `navigation_params.yaml` (Recommended)
Complete configuration for all three nodes:
- `mpc_controller_node`
- `gt_localization_node`
- `crowd_state_node`

**Use this file for most cases.**

### 2. Individual Config Files
- `mpc_controller.yaml` - MPC controller only
- `gt_localization.yaml` - Ground truth localization only
- `crowd_state.yaml` - Crowd state extraction only

## How to Use

### Option 1: Use Default Config (navigation_params.yaml)
```bash
source install/setup.bash
ros2 launch social_mpc_nav social_mpc_demo.launch.py
```

### Option 2: Override Goal Position
```bash
ros2 launch social_mpc_nav social_mpc_demo.launch.py \
  goal_x:=10.0 \
  goal_y:=5.0
```

### Option 3: Use Custom Config File
```bash
ros2 launch social_mpc_nav social_mpc_demo.launch.py \
  config_file:=/path/to/your/custom_config.yaml
```

### Option 4: Run Single Node with Config
```bash
ros2 run social_mpc_nav mpc_controller_node \
  --ros-args --params-file config/mpc_controller.yaml
```

## Key Parameters to Tune

### Goal Settings
- `goal_x`, `goal_y`: Target position (meters)
- `goal_tolerance`: Distance to consider goal reached (default: 0.15m)

### Navigation Behavior
- `default_v_max`: Maximum speed (default: 0.6 m/s)
- `omega_max`: Maximum rotation speed (default: 0.9 rad/s)

### Collision Avoidance
- `w_obstacle`: Obstacle avoidance strength (default: 3.0)
  - Increase for more cautious avoidance
  - Decrease if robot is too hesitant
- `min_obstacle_distance`: Safety margin (default: 0.3m)
- `min_valid_laser_range`: Filter self-detections (default: 0.15m)

### MPC Tuning
- `N`: Prediction horizon steps (default: 15)
  - Longer = smoother but slower computation
- `num_rollouts`: Trajectory samples (default: 60)
  - More = better paths but slower
- `dt`: Time step (default: 0.2s)

## Example Custom Config

Create `my_config.yaml`:
```yaml
mpc_controller_node:
  ros__parameters:
    goal_x: 15.0
    goal_y: 10.0
    goal_tolerance: 0.20
    default_v_max: 0.4  # Slower, more cautious
    w_obstacle: 5.0     # Stronger obstacle avoidance
    min_obstacle_distance: 0.5  # Larger safety margin
```

Then run:
```bash
ros2 launch social_mpc_nav social_mpc_demo.launch.py \
  config_file:=my_config.yaml
```

## Tips

1. **Start with defaults** - The default values work well for most scenarios
2. **Tune one parameter at a time** - Makes it easier to see effects
3. **Check logs** - The node prints parameter values on startup
4. **Save working configs** - Copy and rename configs that work well

## Parameter Reference

See `navigation_params.yaml` for complete list with descriptions.
