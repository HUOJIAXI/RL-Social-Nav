# DWA Local Planner

Pure Dynamic Window Approach (DWA) local planner for TIAGo robot navigation.

## Features

- **Pure DWA algorithm** - No VLM integration, classic DWA implementation
- **Ground truth localization** - Uses TF from `gt_localization_node` for accurate positioning
- **Global path following** - Subscribes to `/global_path` and follows waypoints
- **Multi-objective cost function**:
  - Goal reaching
  - Heading alignment
  - Obstacle avoidance
  - Social distance (pedestrian avoidance)
  - Motion smoothness
- **Real-time logging** - CSV logs for analysis

## Usage

### Launch with Global Planning

Start all necessary nodes (GT localization, global planner, and DWA):

```bash
ros2 launch dwa_local_planner dwa_with_planning.launch.py goal_x:=10.0 goal_y:=5.0
```

This launches:
- `gt_localization_node` - Ground truth localization from Gazebo
- `global_planner_node` - A* global path planner
- `dwa_controller_node` - Pure DWA local planner

### Set Goal from rviz2

The planner also accepts goals from rviz2:
1. Open rviz2
2. Use "2D Goal Pose" tool
3. The global planner will compute a path
4. DWA will follow the path

## Topics

### Subscribed Topics

- `/goal_pose` (geometry_msgs/PoseStamped) - Goal from rviz2
- `/global_path` (nav_msgs/Path) - Global path from planner
- `/lidar/scan` (sensor_msgs/LaserScan) - Laser scan for obstacles
- `/people_2d` (social_mpc_nav/People2D) - Pedestrian positions
- TF: `map` -> `base_footprint` - Ground truth robot pose

### Published Topics

- `/task_generator_node/tiago_base/cmd_vel` (geometry_msgs/Twist) - Velocity commands

## Configuration

Edit `config/dwa_controller.yaml` to adjust:

```yaml
# Velocity limits
default_v_max: 0.6  # m/s
omega_max: 0.9      # rad/s

# DWA parameters
dt: 0.2             # Time step
predict_time: 3.0   # Prediction horizon
v_resolution: 0.05  # Linear velocity sampling
w_resolution: 0.1   # Angular velocity sampling

# Cost weights
w_goal: 1.0         # Goal reaching
w_heading: 0.5      # Heading alignment
w_obstacle: 2.0     # Obstacle avoidance
w_social: 1.5       # Pedestrian avoidance
w_smooth: 0.3       # Motion smoothness
```

## Logs

CSV logs are saved to `~/ros2_logs/dwa_local_planner/`:

- `dwa_log_YYYYMMDD_HHMMSS.csv` - Main control decisions

Columns:
- `stamp_sec` - Timestamp
- `px, py, yaw` - Robot pose
- `goal_x, goal_y` - Current goal
- `v_cmd, w_cmd` - Velocity commands
- `min_obstacle_dist` - Closest obstacle
- `min_person_dist` - Closest pedestrian
- `num_people` - Number of people detected

## Comparison with VLM-MPC

For comparison experiments, the package also includes `dwa_controller_vlm_node` which has the same VLM integration as the MPC controller. However, the recommended version is the pure DWA (`dwa_controller_node`).

## Architecture

```
gt_localization_node
    ↓ (TF: map -> odom -> base_footprint)
global_planner_node
    ↓ (/global_path)
dwa_controller_node
    ↓ (/task_generator_node/tiago_base/cmd_vel)
TIAGo Robot
```

## Algorithm

The DWA algorithm:
1. Gets robot pose from TF (ground truth)
2. Updates current waypoint from global path
3. Computes dynamic window based on velocity and acceleration limits
4. Samples (v, ω) pairs within the window
5. For each sample:
   - Simulates trajectory using unicycle model
   - Evaluates multi-objective cost
6. Selects best velocity command
7. Publishes to robot

## Dependencies

- `social_mpc_nav` - For People2D messages and global planner
- ROS 2 Humble
- TF2

## Notes

- Uses ground truth localization instead of odometry for accurate positioning
- Follows global path with 1.0m lookahead distance
- Stops when within 0.3m of final goal
- Logs all decisions for post-analysis
