# RL-Social-Nav

Reinforcement Learning for Socially-Aware Robot Navigation in Crowded Environments.

This project implements deep reinforcement learning methods for robot navigation in crowded spaces, based on the ICRA 2019 paper [Crowd-Robot Interaction: Crowd-aware Robot Navigation with Attention-based Deep Reinforcement Learning](https://arxiv.org/abs/1809.08835).

## Abstract

Mobility in an effective and socially-compliant manner is an essential yet challenging task for robots operating in crowded spaces. This project implements attention-based deep reinforcement learning techniques to learn socially cooperative navigation policies that model both Human-Robot and Human-Human interactions.

## Project Structure

```
RL/
├── CrowdNav/                 # Main navigation package
│   ├── crowd_nav/            # Training and testing code
│   │   ├── configs/          # Configuration files
│   │   ├── policy/           # RL policies (CADRL, SARL, LSTM-RL)
│   │   ├── utils/            # Training utilities
│   │   ├── train.py          # Training script
│   │   └── test.py           # Testing script
│   └── crowd_sim/            # Simulation environment
│       └── envs/             # Gym environment for crowd simulation
├── Python-RVO2/              # RVO2 library for collision avoidance
└── README.md
```

## Installation

### Prerequisites
- Python 3.8+
- PyTorch
- CUDA (optional, for GPU training)

### Setup

1. Install Python-RVO2 library:
```bash
cd Python-RVO2
pip install -e .
```

2. Install CrowdNav packages:
```bash
cd CrowdNav
pip install -e .
```

3. Install additional dependencies:
```bash
pip install gym torch numpy matplotlib
```

## Usage

All commands should be executed from the `CrowdNav/crowd_nav/` directory.

### Training

Train a policy with different algorithms:

```bash
# Train SARL policy (CPU)
python train.py --policy sarl

# Train SARL policy (GPU)
python train.py --policy sarl --gpu

# Train other policies
python train.py --policy cadrl --gpu
python train.py --policy lstm_rl --gpu
```

### Testing

Test trained policies:

```bash
# Test ORCA baseline
python test.py --policy orca --phase test

# Test trained SARL model
python test.py --policy sarl --model_dir data/output --phase test
```

### Visualization

Visualize navigation results:

```bash
# Visualize a specific test case
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 0
```

### Plot Training Curve

```bash
python utils/plot.py data/output/output.log
```

## Available Policies

| Policy | Description |
|--------|-------------|
| `orca` | Optimal Reciprocal Collision Avoidance (baseline) |
| `cadrl` | Collision Avoidance with Deep Reinforcement Learning |
| `lstm_rl` | LSTM-based RL policy |
| `sarl` | Socially Attentive RL with self-attention mechanism |

## Configuration

Configuration files are located in `CrowdNav/crowd_nav/configs/`:
- `env.config` - Environment parameters
- `policy.config` - Policy hyperparameters
- `train.config` - Training settings

## Citation

```bibtex
@inproceedings{chen2019crowd,
  title={Crowd-robot interaction: Crowd-aware robot navigation with attention-based deep reinforcement learning},
  author={Chen, Changan and Liu, Yuejiang and Kreiss, Sven and Alahi, Alexandre},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={6015--6022},
  year={2019},
  organization={IEEE}
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
