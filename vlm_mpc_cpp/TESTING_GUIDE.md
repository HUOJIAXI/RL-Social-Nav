# VLM-MPC Testing Guide

## Overview

This guide explains how to properly test and compare MPC with and without VLM integration to enhance VLM performance.

## Quick Start

### Single Command Testing

```bash
# Run complete comparison for scenarios S1, S2, S3 with 10 trials each
./scripts/run_full_comparison.sh 10 S1 S2 S3
```

This will:
1. Run 10 trials of each scenario WITHOUT VLM
2. Run 10 trials of each scenario WITH VLM
3. Extract metrics from all trials
4. Generate comparison plots
5. Create a summary report

**Output:** `~/ros2_logs/social_mpc_nav/experiments/comparison_YYYYMMDD_HHMMSS/`

---

## Understanding Your MPC Implementations

You have **3 MPC implementations** with ~70% code overlap:

| Implementation | File | Solver | VLM Support | Status |
|----------------|------|--------|-------------|--------|
| **Base MPC** | `mpc_controller_node.cpp` | Random-shooting | ❌ No | Legacy |
| **VLM MPC** | `mpc_controller_vlm_node.cpp` | Random-shooting | ✅ Yes | Active |
| **CasADi MPC** | `mpc_controller_casadi_node.cpp` | CasADi optimizer | ✅ Yes | **Recommended** |

### Recommended: Use CasADi Implementation

The CasADi node (`mpc_controller_casadi_node.cpp`) is the most advanced and supports both VLM and no-VLM modes:

```bash
# Launch with VLM
ros2 launch social_mpc_nav mpc_casadi_full.launch.py enable_vlm:=true

# Launch without VLM (pure MPC baseline)
ros2 launch social_mpc_nav mpc_casadi_full.launch.py enable_vlm:=false
```

**Advantages:**
- Same solver for both conditions (fair comparison)
- Gradient-based optimization (potentially better solutions)
- Can enable/disable VLM via launch parameter
- Already used by your `run_experiments.sh` script

---

## What Gets Tested

### Metrics Extracted

Your `extract_metrics.py` computes:

#### 1. Navigation Efficiency
- `time_to_goal_sec` - How long to reach goal
- `path_length_m` - Total distance traveled
- `path_efficiency` - Euclidean/actual ratio (closer to 1.0 is better)
- `avg_velocity_ms` - Mean velocity
- `num_stops` - Number of stops (v < 0.05 m/s)
- `goal_reached` - Success flag (within 0.5m)

#### 2. Safety
- `min_obstacle_distance_m` - Closest approach to obstacles
- `min_person_distance_m` - Closest approach to people
- `hard_violations_obstacles` - Collisions with obstacles (< 0.2m)
- `hard_violations_people` - Collisions with people (< 0.2m)
- `soft_violations_obstacles` - Proximity violations (< 0.3m)
- `danger_time_ratio` - % time in danger zone

#### 3. Social Compliance (KEY FOR VLM)
- `avg_personal_distance_m` - Mean distance to people
- `personal_space_violations` - Times closer than 0.5m
- `comfort_score` - Exponential comfort metric (higher is better)

#### 4. Motion Smoothness
- `velocity_variance` - Smoothness of linear velocity
- `angular_variance` - Smoothness of angular velocity
- `avg_jerk` - Acceleration changes (lower is smoother)

#### 5. Computational
- `avg_solve_time_ms` - MPC solve time
- `max_solve_time_ms` - Worst-case solve time

---

## VLM Integration Points

### How VLM Affects MPC

The VLM translator publishes parameters to `/vlm/mpc_parameters`:

```
VLMParameters:
  source: "vlm" | "fallback_cached" | "fallback_rules"
  scene_type: "corridor" | "lobby" | "doorway" | "queue" | "crossing"
  crowd_density: "empty" | "sparse" | "medium" | "dense"
  recommended_action: "go_ahead" | "slow_down" | "stop_and_wait" | "yield"
  speed_scale: 0.0-1.0              # Velocity multiplier
  min_personal_distance: 0.5-2.0    # Adaptive personal space
  side_preference: "left" | "right" | "neutral"
  need_to_wait: bool                # Emergency stop
  adaptive_min_obstacle_distance: 0.25-0.5  # Dynamic safety margin
```

### VLM Cost Components (in `mpc_controller_vlm_node.cpp`)

The VLM node adds these costs:

```cpp
// 1. Directional cost (corridor side preference)
cost += w_vlm_directional * computeVLMDirectionalCost(...)

// 2. Action cost (slow down, stop, yield)
cost += w_vlm_action * computeVLMActionCost(...)

// 3. Scene cost (doorway, crossing, queue)
cost += w_vlm_scene * computeVLMSceneCost(...)

// 4. Personal distance cost (adaptive comfort zone)
cost += w_vlm_personal * computeVLMPersonalDistanceCost(...)
```

**Default weights** (in `config/mpc_controller.yaml`):
- `w_vlm_directional: 1.0`
- `w_vlm_action: 2.0`
- `w_vlm_scene: 1.5`
- `w_vlm_personal: 5.0`

---

## Testing Scenarios

Define scenarios in `run_experiments.sh` (lines 62-79):

| Scenario | Description | Goal | Expected VLM Behavior |
|----------|-------------|------|----------------------|
| S1 | Sparse corridor | (10, 0) | Side preference, smooth passage |
| S2 | Dense corridor | (10, 0) | Slow down, maintain distance |
| S3 | Doorway | (15, 0) | Yield behavior, wait if needed |
| S4 | Queue/crossing | (15, 5) | Stop and wait, high social awareness |
| S5 | Crowded lobby | (8, 5) | Adaptive speed, personal space |

---

## What VLM Should Improve

Based on your metrics, VLM should show:

### ✅ Expected Improvements (p < 0.05)
- **Higher `comfort_score`** (target: > 0.7)
- **Higher `min_person_distance_m`** (target: > 1.0m)
- **Fewer `personal_space_violations`** (target: < 5 per trial)
- **Higher `avg_personal_distance_m`** (when people present)

### ⚠️ Acceptable Trade-offs
- **Slightly longer `time_to_goal_sec`** (acceptable if < 20% increase)
- **Slightly lower `path_efficiency`** (due to detours)
- **Slightly higher `num_stops`** (due to yielding)

### ❌ Must NOT Degrade
- **`min_obstacle_distance_m`** (safety critical)
- **`hard_violations_*`** (must be zero)
- **`avg_solve_time_ms`** (real-time constraint)

---

## Step-by-Step Testing Process

### 1. Prepare Environment

```bash
cd /home/huojiaxi/phd_upm/vlm_mpc_cpp
source install/setup.bash

# Clean previous logs (optional)
rm -rf ~/ros2_logs/social_mpc_nav/experiments/*
```

### 2. Run Automated Comparison

```bash
# Quick test (5 trials, 2 scenarios)
./scripts/run_full_comparison.sh 5 S1 S2

# Full test (10 trials, all scenarios)
./scripts/run_full_comparison.sh 10 S1 S2 S3 S4 S5
```

### 3. Analyze Results

After completion, check:

```bash
# View summary report
cat ~/ros2_logs/social_mpc_nav/experiments/comparison_*/report.md

# View plots
eog ~/ros2_logs/social_mpc_nav/experiments/comparison_*/plots/*.png
```

### 4. Interpret Results

Look for:
- **Statistical significance:** p-value < 0.05
- **Effect size:** Improvement > 10%
- **Consistency:** Same trend across scenarios

---

## Manual Testing (for Debugging)

### Test Single Trial

```bash
# Terminal 1: Launch system
ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
    enable_vlm:=true \
    log_directory:=/tmp/test_trial \
    log_mpc_to_csv:=true

# Terminal 2: Publish goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}}}"

# Terminal 3: Monitor VLM parameters
ros2 topic echo /vlm/mpc_parameters

# After completion, check logs
cat /tmp/test_trial/mpc_casadi_log.csv
```

---

## Tuning VLM Weights for Better Performance

If VLM is not improving social metrics:

### Increase Social Awareness

Edit `social_mpc_nav/config/mpc_controller.yaml`:

```yaml
# Make VLM more cautious around people
w_vlm_personal: 10.0  # Increase from 5.0
w_vlm_action: 3.0     # Increase from 2.0

# Increase personal distance threshold
min_personal_distance: 1.5  # Increase from 1.0
```

### Reduce Navigation Overhead

If VLM is too slow:

```yaml
# Reduce VLM influence on navigation
w_vlm_directional: 0.5  # Decrease from 1.0
w_vlm_scene: 1.0        # Decrease from 1.5

# Allow higher speeds in safe scenes
speed_scale_min: 0.7    # Increase from 0.5
```

### Test Changes

```bash
# Rebuild after config changes
colcon build --packages-select social_mpc_nav

# Re-run tests
./scripts/run_full_comparison.sh 10 S1 S2
```

---

## Understanding the Test Pipeline

```
run_experiments.sh
    ↓
[Launch Gazebo + MPC (enable_vlm=true/false)]
    ↓
[Publish goal]
    ↓
[Wait for completion (120s)]
    ↓
[Save CSV logs]
    ↓
extract_metrics.py
    ↓
[Parse CSV, compute metrics]
    ↓
[Save metrics.json]
    ↓
visualize_comparison.py
    ↓
[Create box plots, bar charts]
    ↓
generate_summary_report.py
    ↓
[Statistical tests, recommendations]
    ↓
[Final report.md]
```

---

## Redundancies to Remove

Your current setup has these redundancies:

### Duplicate Scripts (Can Delete)
- `test_vlm_topic.sh` → Use `ros2 topic echo /vlm/mpc_parameters`
- `check_vlm_node.sh` → Use `ros2 node list | grep vlm`
- `test_prompt_output.sh`, `capture_prompt.sh`, etc. → Debugging only

### Keep These
- `run_experiments.sh` - Core automation
- `extract_metrics.py` - Metric computation
- `visualize_comparison.py` - Plotting
- `run_full_comparison.sh` - **NEW** master script
- `generate_summary_report.py` - **NEW** reporting

### Code Refactoring Needed
- 3 MPC implementations have ~70% code overlap
- Consider consolidating into base class + VLM extension
- Extract common functions to utilities

---

## Troubleshooting

### No VLM Parameters Being Published

```bash
# Check VLM nodes are running
ros2 node list | grep vlm

# Check VLM integration node
ros2 topic echo /vlm/response

# Check VLM translator
ros2 topic echo /vlm/mpc_parameters
```

### Experiments Timeout

Increase timeout in `run_experiments.sh`:
```bash
TIMEOUT=180  # Increase from 120 seconds
```

### Metrics Not Extracted

Check CSV log format:
```bash
head -20 ~/ros2_logs/social_mpc_nav/experiments/*_trial0_mpc_casadi_log.csv
```

Should have columns: `timestamp,x,y,yaw,v_cmd,w_cmd,goal_dist,min_obs_dist,min_person_dist,...`

---

## Expected Output

After running `./scripts/run_full_comparison.sh 10 S1 S2`:

```
~/ros2_logs/social_mpc_nav/experiments/comparison_20251214_153000/
├── metrics.json                              # Raw metrics
├── report.md                                 # Summary report
└── plots/
    ├── time_to_goal_sec_comparison.png
    ├── comfort_score_comparison.png
    ├── min_person_distance_m_comparison.png
    ├── path_efficiency_comparison.png
    └── ...
```

**Sample Report Output:**

```markdown
# VLM-MPC vs No-VLM Comparison Report

## Executive Summary

| Metric | No-VLM Mean | VLM Mean | Improvement | Significant? |
|--------|-------------|----------|-------------|--------------|
| Min Person Distance (m) | 0.812 | 1.234 | +52.0% | ✅ Yes (p=0.003) |
| Comfort Score | 0.621 | 0.782 | +25.9% | ✅ Yes (p=0.012) |
| Time to Goal (s) | 98.3 | 112.5 | +14.5% | ✅ Yes (p=0.042) |

### Overall Verdict: 85% of key metrics improved

✅ **VLM integration shows strong positive impact**
```

---

## Next Steps

1. **Run initial comparison:** `./scripts/run_full_comparison.sh 10 S1 S2`
2. **Analyze report:** Check which metrics improved
3. **Tune weights:** Adjust VLM cost weights in config
4. **Iterate:** Re-run tests after changes
5. **Document:** Update VLM_INTEGRATION.md with findings

---

## References

- Main implementation: `social_mpc_nav/src/mpc_controller_casadi_node.cpp`
- VLM costs: `social_mpc_nav/include/social_mpc_nav/mpc_vlm_helpers.hpp`
- Configuration: `social_mpc_nav/config/mpc_controller.yaml`
- VLM integration: `social_mpc_nav/src/vlm_integration_node.cpp`
