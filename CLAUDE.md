# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Research repository for socially-aware robot navigation combining deep reinforcement learning and model predictive control. Based on the ICRA 2019 paper "Crowd-Robot Interaction: Crowd-aware Robot Navigation with Attention-based Deep Reinforcement Learning."

Two main subsystems:
- **CrowdNav** — PyTorch-based deep RL for crowd navigation (training, simulation, evaluation)
- **vlm_mpc_cpp** — ROS 2 Humble C++ MPC navigation stack with VLM + SARL integration (standard colcon workspace with packages in `vlm_mpc_cpp/src/`; see `vlm_mpc_cpp/docs/CLAUDE.md` for detailed guidance)

Supporting components: **Python-RVO2** (Cython wrapper for RVO2 collision avoidance library) and **main.py** (standalone Stable Baselines3 PPO demo on Breakout-v5).

## Commands

### Installation

```bash
# RVO2 collision avoidance library
cd Python-RVO2 && pip install -e .

# CrowdNav package
cd CrowdNav && pip install -e .

# Python dependencies
pip install gym torch numpy matplotlib
```

### CrowdNav Training & Evaluation

All commands run from `CrowdNav/crowd_nav/`:

```bash
# Train (policies: sarl, cadrl, lstm_rl, orca)
python train.py --policy sarl --gpu
python train.py --policy sarl          # CPU-only

# Evaluate
python test.py --policy sarl --model_dir data/output --phase test

# Evaluate with visualization
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 0

# Plot training curves
python utils/plot.py data/output/output.log
```

### vlm_mpc_cpp Build & Launch

```bash
source /opt/ros/humble/setup.bash
cd vlm_mpc_cpp
colcon build --packages-select social_mpc_nav
source install/setup.bash

# Full navigation stack (recommended)
ros2 launch social_mpc_nav mpc_with_global_planner.launch.py goal_x:=-5.0 goal_y:=-15.0

# VLM-enhanced navigation
ros2 launch social_mpc_nav mpc_vlm_full.launch.py goal_x:=10.0 goal_y:=5.0

# VLM + SARL integrated navigation (interpretable social navigation)
ros2 launch social_mpc_nav mpc_sarl_vlm.launch.py \
    enable_vlm:=true log_mpc_to_csv:=true
```

## Architecture

### CrowdNav

Two-phase training pipeline: imitation learning (IL) pre-training from ORCA demonstrations, then reinforcement learning (RL) fine-tuning with epsilon-greedy exploration and experience replay.

**Simulation** (`crowd_sim/envs/`): Custom Gym environment simulating humans via RVO2. Supports circle-crossing and square-crossing scenarios. Configurable human count, velocities, sensing range, and reward structure.

**Policies** (`crowd_nav/policy/`): All RL policies inherit from `multi_human_rl.py`, which handles vectorized state transformation for variable numbers of humans. Individual policies:
- `cadrl.py` — pairwise human-robot interaction modeling
- `sarl.py` — self-attention pooling over all human states (best performer)
- `lstm_rl.py` — recurrent encoding of human states
- `policy_factory.py` — factory pattern for policy instantiation

**Training loop** (`train.py`): Manages IL/RL phases, epsilon decay schedule, replay buffer, and model checkpointing. Uses `utils/explorer.py` for episode rollouts and `utils/trainer.py` for SGD optimization.

**Configuration** (`crowd_nav/configs/`):
- `env.config` — environment parameters (time limit, rewards, scenario)
- `policy.config` — network architecture (MLP dims, attention dims, RL hyperparams)
- `train.config` — training schedule (learning rates, batch size, epsilon decay, IL/RL episode counts)

### vlm_mpc_cpp

Modular ROS 2 node architecture. See `vlm_mpc_cpp/docs/CLAUDE.md` for full details including message flow, coordinate frames, node descriptions, and configuration parameters. Key highlights:
- Random-shooting MPC over unicycle model with weighted cost function (goal, social, obstacle, smoothness)
- `SocialContractHelper` adjusts MPC weights based on crowd proximity
- VLM integration adds four additional cost terms from GPT-4V/Claude scene analysis
- All nodes log to CSV for experiment analysis

### VLM + SARL Integration

The SARL (Socially Attentive RL) integration provides interpretable social navigation by combining VLM scene understanding with SARL's learned per-pedestrian attention. Three synergy mechanisms:

**Synergy 1 — SARL attention enriches VLM prompts**: `vlm_integration_node.cpp` subscribes to `/sarl/output` and injects attention weights into per-person descriptions sent to the VLM. This replaces distance-only priority labels with learned behavioral importance (e.g., "SARL_ATTENTION=0.72, RL_RISK=HIGH").

**Synergy 2 — VLM scene context conditions SARL cost weights**: `mpc_controller_sarl_node.cpp` uses VLM `scene_type` and `crowd_density` to scale SARL's MPC cost contributions via `sarl_helpers.hpp::getSceneMultiplier()`. In constrained spaces (corridors, doorways), SARL's attention-weighted cost is amplified. In open spaces, it's reduced.

**Synergy 3 — Combined interpretability**: The `/navigation/explanation` topic publishes `NavigationExplanation` messages that merge VLM scene context with SARL per-person attention, providing human-readable explanations of navigation decisions.

**Key files**:
- `src/social_mpc_nav/scripts/sarl_bridge_node.py` — Python node loading trained SARL model, publishes attention weights at 10 Hz + batch V(s) evaluation service
- `src/social_mpc_nav/src/mpc_controller_sarl_node.cpp` — MPC controller with attention-weighted social cost and SARL terminal value
- `src/social_mpc_nav/include/social_mpc_nav/sarl_helpers.hpp` — Cost functions and scene-dependent weight multipliers
- `src/social_mpc_nav/msg/SARLOutput.msg` — Per-person attention weights and V(s)
- `src/social_mpc_nav/msg/NavigationExplanation.msg` — Combined VLM+SARL interpretability output
- `src/social_mpc_nav/srv/EvaluateSARLBatch.srv` — Batch terminal state evaluation for MPC rollouts
- `src/social_mpc_nav/config/mpc_sarl.yaml` — MPC + SARL weight configuration
- `src/social_mpc_nav/config/sarl_bridge.yaml` — SARL bridge node configuration
- `src/social_mpc_nav/launch/mpc_sarl_vlm.launch.py` — Full VLM+SARL+MPC stack launch

**SARL model**: Trained weights at `CrowdNav/crowd_nav/data/output/rl_model.pth`. The bridge node replicates the `ValueNetwork` architecture and `rotate()` coordinate transform from CrowdNav. Unicycle-to-holonomic conversion: `vx = v*cos(yaw), vy = v*sin(yaw)`.

**RViz visualization**: `/sarl/attention_markers` publishes colored sphere markers (green=low, red=high attention) above each person, with attention weight labels and V(s) display above the robot.

## Key Dependencies

| Component | Stack |
|-----------|-------|
| CrowdNav | Python 3.8+, PyTorch, Gym, NumPy, Matplotlib, RVO2 |
| vlm_mpc_cpp | ROS 2 Humble, C++17, CMake, Gazebo Harmonic, OpenCV, libcurl, CasADi (optional) |
| main.py | Stable Baselines3, gymnasium, ale-py |
