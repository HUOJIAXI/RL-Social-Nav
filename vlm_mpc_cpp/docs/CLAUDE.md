# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Social MPC Navigation Stack for TIAGo mobile robot in Gazebo Harmonic using ROS 2 Humble. The system implements a random-shooting Model Predictive Controller (MPC) for socially-aware navigation with ground-truth localization and rule-based social contracts. Experimental VLM integration enables vision-based scene understanding.

## Build and Run Commands

### Prerequisites
```bash
source /opt/ros/humble/setup.bash
./scripts/install_dependencies.sh  # Run once to install dependencies
```

### Build
```bash
colcon build --packages-select social_mpc_nav
source install/setup.bash
```

### Launch Options

**Basic demo (MPC with GT localization and crowd state):**
```bash
ros2 launch social_mpc_nav social_mpc_demo.launch.py \
  goal_x:=3.0 goal_y:=0.0 \
  config_file:=/path/to/navigation_params.yaml
```

**Full navigation stack with global planner (recommended):**
```bash
ros2 launch social_mpc_nav mpc_with_global_planner.launch.py \
  goal_x:=-5.0 goal_y:=-15.0 \
  enable_debug_logging:=true \
  num_rollouts:=100 \
  N:=15
```
This launches: global_planner_node, people_adapter_node, pointcloud_to_laserscan, gt_localization_node, and mpc_controller_node.

**VLM integration (experimental):**
```bash
ros2 launch social_mpc_nav vlm_integration.launch.py
```

**Run individual nodes:**
```bash
ros2 run social_mpc_nav mpc_controller_node --ros-args --params-file config/mpc_controller.yaml
ros2 run social_mpc_nav global_planner_node --ros-args --params-file config/global_planner.yaml
```

## Architecture

### Core Components

The system is organized into independent ROS 2 nodes connected via topics and TF transforms:

1. **Perception & Localization**
   - `gt_localization_node`: Subscribes to `/gazebo/model_states`, publishes `/odom` and `map -> base_link` TF for perfect localization
   - `crowd_state_node`: Extracts person models from Gazebo, estimates velocities, publishes `People2D`
   - `people_adapter_node`: Converts `people_msgs/msg/People` to `social_mpc_nav/msg/People2D` format

2. **Planning**
   - `global_planner_node`: A* path planning on occupancy grid, publishes waypoint-based `/global_path`
   - `social_contract_helper`: Rule-based heuristics library that adjusts `v_max`, `w_goal`, and `w_social` based on crowd proximity
   - `pointcloud_to_laserscan`: Converts 3D Velodyne PointCloud2 to 2D LaserScan for obstacle detection (launched in full stack)

3. **Control**
   - `mpc_controller_node`: Random-shooting MPC over unicycle model, blends goal, social, and smoothness costs, outputs `/cmd_vel`

4. **VLM Integration (Experimental)**
   - `vlm_integration_node`: Subscribes to 3D bounding boxes (`gb_visual_detection_3d_msgs`) and person tracking info, captures RGB images, assembles multimodal prompts, triggers VLM API on events (new detections, scene change, periodic)

### Message Flow

```
Gazebo /gazebo/model_states → gt_localization_node → /odom + TF(map→base_link)
                                                    ↓
Gazebo /gazebo/model_states → crowd_state_node → People2D → mpc_controller_node
                                                             ↓
                                                        /cmd_vel → Robot

/task_generator_node/people → people_adapter_node → People2D → mpc_controller_node

global_planner_node → /global_path → mpc_controller_node (waypoint tracking mode)
```

### Coordinate Frames

- **map**: Fixed world frame (Gazebo origin)
- **odom**: Odometry frame (typically `tiago_official/odom`, coincides with map for GT localization)
- **base_link** / **base_footprint**: Robot base frame (typically `tiago_official/base_footprint`)
- **laser**: Laser scanner frame
- **world**: Gazebo world frame

**Critical**: The MPC controller operates in the `map` frame. All transformations between frames are handled via TF2. The `gt_localization_node` publishes the `map → odom` transform (identity) to enable perfect localization.

**Robot naming**: The system uses `tiago_official` as the robot name prefix in topics and frames. This is hardcoded in the full launch file but can be configured via parameters in simpler launches.

### Custom Messages

Located in `social_mpc_nav/msg/`:
- `Person2D.msg`: Single person with name, position (x, y), and velocity (vx, vy) in map frame
- `People2D.msg`: Timestamped array of Person2D messages

### Social Contract System

The `SocialContractHelper` class (social_mpc_nav/include/social_mpc_nav/social_contract.hpp) implements rule-based adjustments:
- Reduces `v_max` when people are nearby
- Increases `w_social` (social avoidance weight) when people are in front
- Logs decisions to CSV for analysis
- Returns a `SocialContract` struct consumed by MPC controller

## Key Configuration Files

All YAML configs are in `social_mpc_nav/config/`:

- `navigation_params.yaml`: Complete config for all three core nodes (recommended)
- `mpc_controller.yaml`: MPC-specific parameters
- `global_planner.yaml`: Global planner parameters
- `people_adapter.yaml`: People adapter parameters
- `vlm_integration.yaml`: VLM integration parameters

**Important tunable parameters:**
- `goal_x`, `goal_y`: Navigation goal
- `N`: MPC horizon steps (default: 15)
- `num_rollouts`: Random trajectory samples (default: 60)
- `w_obstacle`: Obstacle avoidance weight (default: 3.0)
- `min_obstacle_distance`: Safety margin (default: 0.3m)
- `default_v_max`: Max linear velocity (default: 0.6 m/s)

## Code Structure Conventions

### Executable Nodes

The package builds the following executables (see `CMakeLists.txt`):
- `gt_localization_node`: Ground-truth localization from Gazebo ModelStates
- `crowd_state_node`: Crowd state estimation from Gazebo person models
- `people_adapter_node`: Converts `people_msgs/People` to `People2D` format
- `global_planner_node`: A* global path planning on occupancy grid
- `mpc_controller_node`: Main MPC controller with social contract integration
- `vlm_integration_node`: Vision-language model integration for scene understanding
- `social_contract_helper`: Shared library (not executable) for rule-based social heuristics

### Node Implementation Pattern

All nodes follow this structure:
1. Declare parameters in constructor with defaults
2. Create publishers/subscribers
3. Initialize TF2 buffer/listener if needed
4. Main computation in callback methods
5. CSV logging with mutex protection

### MPC Controller Architecture

The `mpc_controller_node.cpp` implements:
- **State propagation**: Unicycle model `(x, y, yaw)` with `(v, omega)` controls
- **Random shooting**: Sample `num_rollouts` control sequences over horizon `N`
- **Cost function**: Weighted sum of goal-reaching, social proximity, obstacle avoidance, and smoothness
- **Social contract integration**: `SocialContractHelper` computes adaptive weights based on crowd proximity
- **Safety checks**: Emergency stop if collision imminent from laser scan
- **Waypoint tracking**: Switches between waypoint-following mode (from global planner) and direct goal mode

**Key inputs:**
- `/task_generator_node/tiago_official/odom`: Robot odometry
- `/task_generator_node/people_array` or people adapter output: Crowd state
- `/scan_2d`: 2D laser scan (converted from PointCloud2 in full launch)
- `/global_path`: Waypoint path from global planner (optional)

**Output:** `/task_generator_node/tiago_official/cmd_vel`

### Logging System

All nodes support CSV logging to `log_directory` parameter:
- `crowd_state.csv`: Timestamped person positions and velocities
- `social_contract.csv`: Social contract decisions per timestep
- `mpc_log.csv`: Control inputs and robot states

## Development Guidelines

### Project Structure

```
social_mpc_nav/
├── src/                        # C++ node implementations
├── include/social_mpc_nav/     # Header files (social_contract.hpp)
├── msg/                        # Custom ROS messages
├── config/                     # YAML parameter files
├── launch/                     # Launch files (.py)
└── CMakeLists.txt

scripts/                        # Shell scripts
├── install_dependencies.sh     # One-time dependency install
├── run_vlm_integration.sh      # VLM node launcher
├── check_vlm_node.sh           # VLM debugging
└── test_vlm_topic.sh           # VLM topic testing

*.md files                      # Documentation (see VLM_INTEGRATION.md, etc.)
```

### Testing Changes

After modifying nodes:
```bash
colcon build --packages-select social_mpc_nav
source install/setup.bash
# Launch with debug logging enabled
ros2 launch social_mpc_nav mpc_with_global_planner.launch.py enable_debug_logging:=true
```

### Coordinate Frame Debugging

The repository includes helper scripts in project root:
- `test_tf_transform.py`: Validates TF transformations
- `verify_coordinates.py`: Checks coordinate consistency across frames

**Common issues:**
- Missing TF transforms: Ensure `gt_localization_node` is running
- Frame mismatches: Check `map_frame` and `base_link_frame` parameters match TF tree
- Robot name mismatches: Verify `robot_model_name` parameter matches Gazebo model name

### VLM Integration Development

The VLM system is under active development. Current state:
- Event-based triggering implemented (new detections, scene change, periodic)
- Multimodal prompt assembly with 3D bounding boxes, person tracking, and RGB images
- Image encoding (JPEG compression, base64) for VLM API
- HTTP API client with libcurl (see `VLM_INTEGRATION.md` for Chinese documentation)
- Next steps: Parse VLM responses and inject semantic constraints into MPC social contract

**Important topics:**
- Input: `/darknet_ros_3d/bounding_boxes`, `/person_tracker/person_info`, RGB camera
- Output: `/vlm/prompt` (assembled prompt), `/vlm/response` (API response when enabled)

## Dependencies

ROS 2 packages (all ros-humble-*):
- Core: `rclcpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`
- Simulation: `gazebo_msgs`, `gazebo_ros_pkgs`
- Perception: `people_msgs`, `gb_visual_detection_3d_msgs`, `person_tracker`, `pointcloud_to_laserscan`
- Visualization: `visualization_msgs`

External dependencies:
- `libcurl` (for VLM HTTP API calls)
- `OpenCV` and `cv_bridge` (for image processing in VLM integration)
- Standard build tools: `build-essential`, `cmake`, `python3-colcon-common-extensions`

All dependencies managed via `scripts/install_dependencies.sh`.

## Future Extensions

The codebase is designed with clear extension points:
- **MPC solver**: Replace random shooting with OSQP/CasADi in `mpc_controller_node.cpp`
- **Perception**: Swap GT localization with SLAM + people detection
- **Social contract**: Replace rules with learned policies or VLM reasoning
- **Global planner**: Integrate with Nav2 for production deployments
