# VLM-Enhanced Social MPC Navigation

A Vision-Language Model (VLM) enhanced Model Predictive Control (MPC) navigation system for social robot navigation in crowded environments. The system integrates semantic scene understanding from VLMs with real-time trajectory optimization to enable socially-aware navigation.

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Building the Project](#building-the-project)
- [Configuration](#configuration)
- [Running the System](#running-the-system)
- [VLM Integration](#vlm-integration)
- [Parameter Tuning](#parameter-tuning)
- [Monitoring and Debugging](#monitoring-and-debugging)
- [File Structure](#file-structure)
- [Troubleshooting](#troubleshooting)
- [Research Background](#research-background)

---

## Overview

This project implements a social navigation system for mobile robots (TIAGo) in Gazebo simulation. It combines:

- **Random-Shooting MPC**: Real-time trajectory optimization with obstacle and pedestrian avoidance
- **VLM Scene Understanding**: GPT-4V/Claude vision model for semantic scene analysis
- **Social Contracts**: Rule-based and VLM-guided social behavior adaptation
- **Global Planning**: A* path planning with waypoint tracking
- **Cluster-Aware Navigation**: Considers pedestrian group dynamics

The system operates in ROS 2 Humble and is designed for research in human-robot interaction and social navigation.

---

## Key Features

### 🤖 Navigation Capabilities
- **Random-shooting MPC** with unicycle dynamics model
- **Social contract system** that adapts speed and proximity based on crowd density
- **Global path planning** with A* algorithm and dynamic waypoint tracking
- **Multi-objective optimization**: Goal reaching, social proximity, obstacle avoidance, and smoothness

### 🧠 VLM Integration
- **Scene understanding**: Corridor, lobby, doorway, crossing, queue detection
- **Crowd density analysis**: Empty, sparse, medium, dense classification
- **Action recommendations**: Go ahead, slow down, stop and wait, yield
- **Directional preferences**: Left/right corridor side suggestions
- **Personal distance adaptation**: Dynamic safety margins based on scene context

### 📊 VLM Cost Terms (New)
The VLM outputs are integrated directly into the MPC objective function with four new cost terms:

1. **Directional Preference Cost** (`w_vlm_directional`): Guides robot to preferred corridor side
2. **Action-Based Cost** (`w_vlm_action`): Enforces VLM action recommendations (stop, yield)
3. **Scene-Specific Cost** (`w_vlm_scene`): Context-aware penalties (doorway, crossing)
4. **Personal Distance Violation Cost** (`w_vlm_personal`): Enhanced social proximity based on VLM

### 🔄 Robust Fallback System
- Cached VLM parameters with configurable expiry (30s default)
- Timeout detection (5s threshold)
- Safe default parameters when VLM unavailable
- Health monitoring and status reporting

### 📈 Monitoring & Logging
- CSV logging for all nodes (MPC decisions, social contracts, VLM translations)
- RViz visualization markers for VLM scene understanding
- Performance metrics (translation latency, parsing errors)
- Real-time status topics

---

## System Architecture

### Node Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo Simulation                        │
│  - TIAGo Base Robot          - Pedestrian Actors               │
│  - Sensor Data (LiDAR, RGB)  - Model States                    │
└────────────┬────────────────────────────┬───────────────────────┘
             │                            │
             v                            v
┌────────────────────────┐    ┌──────────────────────────┐
│  gt_localization_node  │    │   vlm_integration_node   │
│  - Publishes map->odom │    │  - RGB image capture     │
│    TF transform        │    │  - 3D bbox tracking      │
│  - Perfect localization│    │  - Cluster info          │
└────────────┬───────────┘    │  - VLM API calls         │
             │                └──────────┬───────────────┘
             │                           │
             │                           v /vlm/response
             │                ┌──────────────────────────┐
             │                │  vlm_translator_node     │
             │                │  - JSON parsing          │
             │                │  - Validation/clamping   │
             │                │  - Fallback logic        │
             │                │  - CSV logging           │
             │                └──────────┬───────────────┘
             │                           │
             │                           v /vlm/mpc_parameters
             │                ┌──────────────────────────┐
             ├───────────────>│  global_planner_node     │
             │                │  - A* path planning      │
             │                │  - Waypoint generation   │
             │                └──────────┬───────────────┘
             │                           │
             │                           v /global_path
             │                ┌──────────────────────────┐
             ├───────────────>│ mpc_controller_vlm_node  │
             │                │  - Random-shooting MPC   │
/odom,       │                │  - Social contract       │
/lidar,      │                │  - VLM modulation        │
/people ────>│                │  - 4 VLM cost terms      │
             │                └──────────┬───────────────┘
             │                           │
             │                           v /cmd_vel
             └───────────────────────────┴───────────────>
                                                   Robot Control
```

### Key Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/task_generator_node/tiago_base/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/task_generator_node/tiago_base/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/task_generator_node/tiago_base/lidar` | `sensor_msgs/LaserScan` | 2D laser scan |
| `/task_generator_node/people_array` | `people_msgs/People` | Detected people |
| `/person_tracker/human_clusters` | `person_tracker_msgs/HumanClusterArray` | Pedestrian clusters |
| `/vlm/response` | `std_msgs/String` | VLM JSON response |
| `/vlm/mpc_parameters` | `social_mpc_nav/VLMParameters` | Translated MPC parameters |
| `/vlm/translator_status` | `social_mpc_nav/TranslatorStatus` | Translator health status |
| `/global_path` | `nav_msgs/Path` | Global waypoint path |

### Custom Messages

#### `VLMParameters.msg`
```
builtin_interfaces/Time stamp
string source                    # "vlm" | "fallback_cached" | "fallback_rules"
float32 confidence              # 0.0-1.0
float32 age_sec                 # Age of parameters

# Scene understanding
string scene_type               # corridor, lobby, doorway, queue, crossing, etc.
string crowd_density            # empty, sparse, medium, dense
string recommended_action       # go_ahead, slow_down, stop_and_wait, yield, etc.

# Control parameters
float32 speed_scale             # 0.0-1.0 multiplier for v_max
float32 min_personal_distance   # 0.5-2.0 meters
string side_preference          # left, right, neutral
bool need_to_wait              # Emergency stop flag
string explanation
```

#### `TranslatorStatus.msg`
```
builtin_interfaces/Time stamp
bool vlm_active
float32 vlm_last_response_age
int32 vlm_responses_received
int32 vlm_parsing_errors
bool using_fallback
string fallback_reason
float32 translation_latency_ms
```

---

## Installation

### Prerequisites

- **OS**: Ubuntu 22.04
- **ROS**: ROS 2 Humble
- **Gazebo**: Gazebo Harmonic
- **Python**: 3.10+

### System Dependencies

```bash
# ROS 2 Humble (if not installed)
sudo apt update
sudo apt install ros-humble-desktop

# Required ROS 2 packages
sudo apt install \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf2-tools \
    ros-humble-people-msgs \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-cv-bridge

# Build tools
sudo apt install \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep

# External libraries
sudo apt install \
    libcurl4-openssl-dev \
    libopencv-dev
```

### Workspace Setup

```bash
# Create workspace
mkdir -p ~/phd_upm/vlm_mpc_cpp
cd ~/phd_upm/vlm_mpc_cpp

# Clone repository (or copy your existing code)
# git clone <repository_url> .

# Install ROS dependencies
rosdep install --from-paths . --ignore-src -r -y

# Source ROS 2
source /opt/ros/humble/setup.bash
```

---

## Building the Project

```bash
# Navigate to workspace root
cd ~/phd_upm/vlm_mpc_cpp

# Build the package
colcon build --packages-select social_mpc_nav

# Source the workspace
source install/setup.bash
```

### Verify Build

```bash
# Check if messages were generated
ros2 interface show social_mpc_nav/msg/VLMParameters
ros2 interface show social_mpc_nav/msg/TranslatorStatus

# Check if executables exist
ros2 pkg executables social_mpc_nav
```

Expected output should include:
- `gt_localization_node`
- `global_planner_node`
- `mpc_controller_node`
- `mpc_controller_vlm_node`
- `vlm_integration_node`
- `vlm_translator_node`
- `people_adapter_node`

---

## Configuration

### Configuration Files

All configuration files are located in `social_mpc_nav/config/`:

#### 1. `navigation_params.yaml` - Main configuration
```yaml
mpc_controller_node:
  ros__parameters:
    # Goal settings
    goal_x: 10.0
    goal_y: 5.0
    goal_tolerance: 0.15

    # MPC parameters
    dt: 0.2                # Time step (seconds)
    N: 15                  # Horizon steps
    num_rollouts: 60       # Random trajectories to sample

    # Velocity limits
    default_v_max: 0.6     # Max linear velocity (m/s)
    omega_max: 0.9         # Max angular velocity (rad/s)

    # Cost weights
    w_smooth: 0.1          # Smoothness
    w_obstacle: 3.0        # Obstacle avoidance

    # VLM cost weights (for mpc_controller_vlm_node)
    w_vlm_directional: 1.0    # Directional preference
    w_vlm_action: 2.0         # Action compliance
    w_vlm_scene: 1.5          # Scene-specific
    w_vlm_personal: 5.0       # Personal distance violation

    # Topics (tiago_base)
    odom_topic: "/task_generator_node/tiago_base/odom"
    cmd_vel_topic: "/task_generator_node/tiago_base/cmd_vel"
```

#### 2. `vlm_translator.yaml` - VLM translator configuration
```yaml
vlm_translator_node:
  ros__parameters:
    # Fallback parameters
    vlm_timeout_threshold_sec: 5.0
    cache_expiry_sec: 30.0
    use_cached_fallback: true

    # Validation ranges
    min_speed_scale: 0.0
    max_speed_scale: 1.0
    min_personal_distance: 0.5
    max_personal_distance: 2.0

    # Logging
    log_translation_to_csv: true
    enable_debug_logging: false
```

#### 3. `vlm_integration_tiago_base.yaml` - VLM integration configuration

Contains VLM API settings, image capture parameters, and event triggers. Edit this file to:
- Add your OpenAI/Anthropic API key
- Configure VLM model (gpt-4-vision-preview, claude-3-sonnet, etc.)
- Adjust triggering frequency and conditions

---

## Running the System

### Option 1: Complete VLM System (Recommended)

Launch all nodes with a single command:

```bash
# Basic launch
ros2 launch social_mpc_nav mpc_vlm_full.launch.py

# With custom goal
ros2 launch social_mpc_nav mpc_vlm_full.launch.py \
    goal_x:=-5.0 goal_y:=-15.0

# With debug logging
ros2 launch social_mpc_nav mpc_vlm_full.launch.py \
    goal_x:=-5.0 goal_y:=-15.0 \
    enable_debug_logging:=true

# Tune VLM weights
ros2 launch social_mpc_nav mpc_vlm_full.launch.py \
    w_vlm_directional:=1.5 \
    w_vlm_action:=3.0 \
    w_vlm_scene:=2.0 \
    w_vlm_personal:=8.0

# Full customization
ros2 launch social_mpc_nav mpc_vlm_full.launch.py \
    goal_x:=10.0 goal_y:=5.0 \
    num_rollouts:=150 \
    N:=20 \
    w_vlm_action:=3.0 \
    enable_vlm_api:=true \
    enable_debug_logging:=true
```

This launches:
- Ground-truth localization
- Global planner
- People adapter
- VLM integration node
- VLM translator node
- VLM-enhanced MPC controller

### Option 2: Standard MPC (Without VLM)

For comparison, launch the standard MPC without VLM enhancements:

```bash
ros2 launch social_mpc_nav mpc_with_global_planner.launch.py \
    goal_x:=-5.0 goal_y:=-15.0
```

### Option 3: Manual Node Launch (For Debugging)

Launch nodes individually in separate terminals:

```bash
# Terminal 1: Localization
ros2 run social_mpc_nav gt_localization_node --ros-args \
    --params-file social_mpc_nav/config/navigation_params.yaml

# Terminal 2: Global Planner
ros2 run social_mpc_nav global_planner_node --ros-args \
    --params-file social_mpc_nav/config/global_planner.yaml

# Terminal 3: People Adapter
ros2 run social_mpc_nav people_adapter_node --ros-args \
    --params-file social_mpc_nav/config/people_adapter.yaml

# Terminal 4: VLM Integration
ros2 run social_mpc_nav vlm_integration_node --ros-args \
    --params-file social_mpc_nav/config/vlm_integration_tiago_base.yaml

# Terminal 5: VLM Translator
ros2 run social_mpc_nav vlm_translator_node --ros-args \
    --params-file social_mpc_nav/config/vlm_translator.yaml

# Terminal 6: VLM-Enhanced MPC Controller
ros2 run social_mpc_nav mpc_controller_vlm_node --ros-args \
    --params-file social_mpc_nav/config/navigation_params.yaml
```

---

## VLM Integration

### VLM Scene Understanding

The VLM analyzes RGB images with 3D bounding boxes and provides semantic understanding:

**Input to VLM**:
- RGB camera image
- 3D bounding boxes of detected objects/people
- Person tracking information (cluster membership, velocities)
- Robot state (position, orientation, goal)

**VLM Output** (JSON format):
```json
{
  "scene_type": "corridor",
  "crowd_density": "medium",
  "recommended_action": "slow_down_and_go",
  "speed_scale": 0.7,
  "min_personal_distance": 1.2,
  "side_preference": "right",
  "need_to_wait": false,
  "explanation": "Medium crowd in corridor. People walking on left side. Stay right and reduce speed."
}
```

### How VLM Affects Navigation

#### 1. Social Contract Modulation
VLM parameters modulate the rule-based social contract:

```cpp
// Speed modulation
contract.v_max *= vlm->speed_scale;  // 0.7 → reduce max speed to 70%

// Social weight adjustment
contract.w_social *= (vlm->min_personal_distance / 1.0);  // 1.2m → increase social cost by 20%

// Emergency stop
if (vlm->need_to_wait) {
    contract.v_max = 0.0;
    contract.w_social = 10.0;
}
```

#### 2. Direct MPC Cost Terms

Four new cost terms are added to the MPC objective function:

**a) Directional Preference Cost** (`w_vlm_directional = 1.0`)
```cpp
// Penalizes deviation from preferred corridor side
if (side_preference == "left" || "right") {
    cost += w_vlm_directional * lateral_deviation²
}
```

**b) Action-Based Cost** (`w_vlm_action = 2.0`)
```cpp
// Enforces VLM-recommended actions
if (recommended_action == "stop_and_wait") {
    cost += w_vlm_action * v²  // Penalize forward motion
}
if (recommended_action == "yield_to_pedestrian") {
    cost += w_vlm_action * (2.0 - frontal_distance)
}
```

**c) Scene-Specific Cost** (`w_vlm_scene = 1.5`)
```cpp
// Context-aware penalties
if (scene_type == "doorway" && v < 0.1) {
    cost += w_vlm_scene * 10.0  // Don't stop in doorways
}
if (scene_type == "crossing" && in_crossing_center) {
    cost += w_vlm_scene * 5.0  // Avoid crossing centers
}
```

**d) Personal Distance Violation Cost** (`w_vlm_personal = 5.0`)
```cpp
// Enhanced social proximity
if (distance_to_person < vlm->min_personal_distance) {
    violation = min_personal_distance - distance
    cost += w_vlm_personal * violation²
}
```

### Cost Weight Hierarchy

The MPC objective balances multiple competing objectives. Recommended weight hierarchy:

1. **Collision avoidance**: Highest priority (hard constraint, cost = 1000.0)
2. **VLM personal distance**: High priority (5.0)
3. **Obstacle avoidance**: High priority (3.0)
4. **VLM action compliance**: Medium priority (2.0)
5. **Goal reaching**: Medium priority (1.0-2.0, adaptive)
6. **VLM scene-specific**: Medium-low priority (1.5)
7. **VLM directional**: Low priority (1.0)
8. **Smoothness**: Low priority (0.1)

### Fallback Behavior

When VLM is unavailable (timeout, parsing error, API failure):

1. **Cached parameters** (if age < 30s): Use last valid VLM output
2. **Default parameters**:
   - `speed_scale = 1.0` (no speed reduction)
   - `min_personal_distance = 1.0` (standard social distance)
   - `side_preference = "neutral"` (no directional bias)
   - `need_to_wait = false` (continue navigation)

---

## Parameter Tuning

### MPC Controller Parameters

| Parameter | Default | Description | Tuning Guidance |
|-----------|---------|-------------|-----------------|
| `num_rollouts` | 60-100 | Number of random trajectories | Increase for crowded scenarios (100-200) |
| `N` | 15 | Prediction horizon steps | Longer horizon = smoother (10-20) |
| `dt` | 0.2 | Time step (seconds) | Smaller = finer resolution (0.1-0.3) |
| `default_v_max` | 0.6 | Max velocity (m/s) | Adjust for robot capabilities (0.3-1.0) |
| `w_obstacle` | 3.0 | Obstacle avoidance weight | Increase if too aggressive (2.0-5.0) |

### VLM Cost Weights

| Parameter | Default | Description | Tuning Guidance |
|-----------|---------|-------------|-----------------|
| `w_vlm_directional` | 1.0 | Directional preference | Increase for stronger side preference (0.5-2.0) |
| `w_vlm_action` | 2.0 | Action compliance | Increase if ignoring VLM actions (1.0-4.0) |
| `w_vlm_scene` | 1.5 | Scene-specific behavior | Increase for stronger context adaptation (1.0-3.0) |
| `w_vlm_personal` | 5.0 | Personal distance violation | Increase if too close to people (3.0-10.0) |

---

## Monitoring and Debugging

### Real-Time Monitoring

```bash
# Monitor VLM responses
ros2 topic echo /vlm/response

# Monitor translated parameters
ros2 topic echo /vlm/mpc_parameters

# Monitor translator status
ros2 topic echo /vlm/translator_status

# Monitor robot velocity commands
ros2 topic echo /task_generator_node/tiago_base/cmd_vel
```

### CSV Logs

Logs are saved to `~/ros2_logs/social_mpc_nav/`:

1. **`vlm_translation.csv`**: VLM translation history
2. **`social_contract.csv`**: Social contract decisions
3. **`mpc_log.csv`**: MPC controller decisions (if enabled)
4. **`crowd_state.csv`**: Detected people positions and velocities

---

## File Structure

```
vlm_mpc_cpp/
├── social_mpc_nav/
│   ├── src/                                     # C++ node implementations
│   ├── include/social_mpc_nav/                  # Header files
│   ├── msg/                                     # Custom ROS messages
│   ├── config/                                  # YAML parameter files
│   ├── launch/                                  # Launch files
│   └── CMakeLists.txt
├── README.md                                    # This file
└── CLAUDE.md                                    # AI assistant project context
```

---

## Troubleshooting

### Common Issues

#### 1. "Failed to parse params file" error
**Solution**: Use correct relative path from workspace root
```bash
cd ~/phd_upm/vlm_mpc_cpp
ros2 run social_mpc_nav mpc_controller_vlm_node --ros-args \
    --params-file social_mpc_nav/config/navigation_params.yaml
```

#### 2. "odom passed to lookupTransform does not exist"
**Solution**: Start gt_localization_node or use complete launch file
```bash
ros2 launch social_mpc_nav mpc_vlm_full.launch.py
```

#### 3. VLM translator always using fallback
**Solution**: Ensure VLM integration node is running and publishing
```bash
ros2 topic echo /vlm/response
ros2 topic echo /vlm/translator_status
```

---

## Research Background

### Key Concepts

**Random-Shooting MPC**: Sampling-based MPC that generates random control sequences, evaluates their cost, and selects the best one.

**Social Contracts**: Adaptive rules that adjust robot behavior based on social context (crowd density, proximity to people).

**VLM Scene Understanding**: Leveraging vision-language models (GPT-4V, Claude) to interpret complex social scenes.

**Cost Function Integration**: Integrates semantic understanding directly into the optimization objective for seamless control.

### Comparison: Standard MPC vs VLM-Enhanced MPC

| Aspect | Standard MPC | VLM-Enhanced MPC |
|--------|-------------|------------------|
| Scene understanding | None (reactive only) | Semantic (doorway, corridor, queue) |
| Social behavior | Fixed rules | Context-adaptive |
| Crowd awareness | Treats people independently | Considers group dynamics |
| Side preference | None | Learns from context |
| Personal distance | Fixed (1.0m) | Adaptive (0.5-2.0m) |

---

## License

This project is research software developed for academic purposes.

---

## Contact

- **Project**: VLM-Enhanced Social MPC Navigation
- **Institution**: Universidad Politécnica de Madrid (UPM)
- **Environment**: Gazebo Harmonic + ROS 2 Humble

---

**Last Updated**: December 2024
