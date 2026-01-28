# VLM-MPC Comparison Experiments

Comprehensive experimental framework for comparing VLM-enhanced MPC against baseline (no-VLM) MPC.

## 📊 Evaluation Metrics

### 1. Navigation Efficiency
- **Time to Goal** - Total navigation time (lower is better)
- **Path Length** - Total distance traveled (lower is better)
- **Path Efficiency** - Ratio of optimal to actual path (higher is better)
- **Average Velocity** - Mean forward velocity (higher indicates smoother navigation)
- **Number of Stops** - Count of velocity ≈ 0 events (lower is better)
- **Task Success Rate** - Percentage of trials reaching goal

### 2. Safety
- **Min Distance to Obstacles** - Closest approach to static obstacles (higher is better)
- **Min Distance to People** - Closest approach to pedestrians (higher is better)
- **Hard Constraint Violations** - Count of d < 0.2m events (should be ZERO)
- **Soft Constraint Violations** - Count of d < adaptive threshold
- **Safety Violation Rate** - Percentage of timesteps with violations
- **Danger Time Ratio** - Fraction of time in danger zone (< 0.5m)

### 3. Social Compliance
- **Personal Space Violations** - Time in people's personal zone (< 0.5m)
- **Average Personal Distance** - Mean distance to nearest person (higher is better)
- **Comfort Score** - Weighted distance metric (1 - exp(-d/σ))
- **Yielding Behavior** - Count of stopping for people

### 4. Motion Smoothness
- **Velocity Variance** - Smoothness of linear velocity (lower is better)
- **Angular Velocity Variance** - Smoothness of rotation (lower is better)
- **Linear Jerk** - Rate of acceleration change (lower is better)
- **Angular Jerk** - Rate of angular acceleration change (lower is better)
- **Number of Direction Changes** - Count of significant heading changes

### 5. Computational Performance
- **MPC Solve Time** - Optimization computation time (lower is better)
- **Control Loop Frequency** - Actual Hz achieved
- **VLM Call Frequency** - VLM queries per second (VLM-MPC only)
- **VLM Response Latency** - Time from request to response (VLM-MPC only)

---

## 🧪 Test Scenarios

| ID | Environment | Crowd Density | Pedestrian Behavior | Difficulty |
|----|-------------|---------------|---------------------|------------|
| **S1** | Open Space | Empty | N/A | Easy |
| **S2** | Open Space | Sparse (2-3) | Static standing | Easy |
| **S3** | Corridor | Sparse (2-3) | Walking same direction | Medium |
| **S4** | Corridor | Medium (4-6) | Walking opposite direction | Medium |
| **S5** | Corridor | Dense (7-10) | Mixed directions | Hard |
| **S6** | Doorway | Sparse (2-3) | Crossing through | Hard |
| **S7** | Doorway | Medium (4-6) | Queuing/waiting | Very Hard |
| **S8** | Crossing | Medium (4-6) | Perpendicular crossing | Hard |
| **S9** | Lobby | Dense (7-10) | Random walking | Very Hard |
| **S10** | Mixed | Dynamic | Dynamic | Stress Test |

### Scenario Parameters
Each scenario defines:
- Start position (fixed)
- Goal position (≥10m away)
- Pedestrian waypoints/paths
- Obstacle layout
- Time limit (120 seconds)

---

## 🚀 Running Experiments

### Prerequisites
```bash
# Install Python dependencies
pip install pandas numpy matplotlib seaborn scipy

# Build the package
cd ~/phd_upm/vlm_mpc_cpp
colcon build --packages-select social_mpc_nav
source install/setup.bash
```

### Step 1: Run Individual Trials

**Option A: Manual launch**
```bash
# No-VLM baseline
ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
    enable_vlm:=false \
    log_mpc_to_csv:=true \
    log_directory:=~/ros2_logs/experiments/S1_no_vlm_trial0

# VLM-MPC
ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
    enable_vlm:=true \
    log_mpc_to_csv:=true \
    log_directory:=~/ros2_logs/experiments/S1_vlm_trial0
```

**Option B: Automated script**
```bash
cd ~/phd_upm/vlm_mpc_cpp

# Run 10 trials for scenario S1 with VLM
./scripts/run_experiments.sh S1 vlm 10

# Run 10 trials for scenario S1 without VLM
./scripts/run_experiments.sh S1 no_vlm 10
```

### Step 2: Complete Experimental Suite

```bash
# Run all scenarios for both conditions
for scenario in S1 S2 S3 S4 S5 S6 S7 S8 S9 S10; do
    for condition in no_vlm vlm; do
        echo "Running $scenario with $condition"
        ./scripts/run_experiments.sh $scenario $condition 10
    done
done
```

**Expected time:** ~40 hours (10 scenarios × 2 conditions × 10 trials × 120s)

**Tip:** Run in parallel on multiple machines or use shorter timeout for faster iteration.

---

## 📈 Data Analysis

### Step 3: Extract Metrics

```bash
cd ~/phd_upm/vlm_mpc_cpp

# Extract metrics from all trials
python scripts/extract_metrics.py \
    --log-dir ~/ros2_logs/social_mpc_nav/experiments \
    --output results/metrics.json \
    --scenarios S1 S2 S3 S4 S5 S6 S7 S8 S9 S10 \
    --conditions no_vlm vlm \
    --trials 10
```

**Output:** `results/metrics.json` containing:
- Individual trial metrics
- Aggregate statistics (mean, std, min, max, median)
- Organized by scenario and condition

### Step 4: Visualize Results

```bash
# Generate comparison plots
python scripts/visualize_comparison.py \
    --metrics results/metrics.json \
    --output results/plots/
```

**Output plots:**
- `time_to_goal_sec_comparison.png` - Box plots for each scenario
- `path_efficiency_comparison.png` - Path efficiency comparison
- `min_obstacle_distance_m_comparison.png` - Safety metrics
- `comfort_score_comparison.png` - Social compliance
- `aggregate_comparison.png` - Summary across all scenarios
- `summary_table.csv` - Numerical results table
- `summary_table.tex` - LaTeX table for papers

### Step 5: Statistical Analysis

```python
import json
import pandas as pd
from scipy import stats

# Load metrics
with open('results/metrics.json', 'r') as f:
    data = json.load(f)

# Compare conditions for scenario S1
s1_novlm = data['S1_no_vlm']['trials']
s1_vlm = data['S1_vlm']['trials']

# Extract time to goal
novlm_times = [t['navigation_efficiency']['time_to_goal_sec'] for t in s1_novlm]
vlm_times = [t['navigation_efficiency']['time_to_goal_sec'] for t in s1_vlm]

# T-test
t_stat, p_value = stats.ttest_ind(novlm_times, vlm_times)
print(f"Time to goal: t={t_stat:.3f}, p={p_value:.3f}")

# Effect size (Cohen's d)
pooled_std = np.sqrt((np.std(novlm_times)**2 + np.std(vlm_times)**2) / 2)
cohens_d = (np.mean(vlm_times) - np.mean(novlm_times)) / pooled_std
print(f"Cohen's d: {cohens_d:.3f}")
```

---

## 📝 Expected Results

### Hypotheses

| Hypothesis | Metric | Expected Result |
|------------|--------|-----------------|
| H1: VLM improves efficiency in dense crowds | Time to Goal (S5, S7, S9) | VLM < No-VLM (p < 0.05) |
| H2: VLM maintains safety | Hard Violations | Both = 0 |
| H3: VLM improves social compliance | Comfort Score | VLM > No-VLM (p < 0.05) |
| H4: VLM adapts to scenes | Adaptive Distance Range | VLM shows variance, No-VLM fixed |
| H5: VLM has acceptable overhead | Solve Time | VLM ≈ No-VLM (p > 0.05) |

### Sample Results Table

| Scenario | Condition | Time (s) ↓ | Path Eff ↑ | Min Obs (m) ↑ | Comfort ↑ | p-value |
|----------|-----------|------------|------------|---------------|-----------|---------|
| S1 (Easy) | No-VLM | 45.2±3.1 | 0.92±0.04 | 0.31±0.05 | 0.78±0.06 | - |
| | **VLM-MPC** | **44.8±2.9** | **0.93±0.03** | **0.32±0.04** | **0.79±0.05** | 0.421 |
| S5 (Dense) | No-VLM | 78.5±12.3 | 0.65±0.12 | 0.28±0.08 | 0.45±0.15 | - |
| | **VLM-MPC** | **62.3±8.7** | **0.78±0.08** | **0.29±0.06** | **0.68±0.09** | **0.003*** |
| S7 (Doorway) | No-VLM | 95.2±18.6 | 0.58±0.15 | 0.26±0.09 | 0.38±0.18 | - |
| | **VLM-MPC** | **71.4±11.2** | **0.72±0.10** | **0.27±0.07** | **0.61±0.12** | **0.001*** |

*↑ higher is better, ↓ lower is better, *** p < 0.001*

---

## 📁 Directory Structure

```
vlm_mpc_cpp/
├── scripts/
│   ├── run_experiments.sh          # Automated experiment runner
│   ├── extract_metrics.py          # Metrics extraction
│   └── visualize_comparison.py     # Visualization and tables
├── results/
│   ├── metrics.json                # Extracted metrics
│   ├── plots/                      # Comparison plots
│   │   ├── time_to_goal_comparison.png
│   │   ├── aggregate_comparison.png
│   │   └── ...
│   ├── summary_table.csv           # Results table (CSV)
│   └── summary_table.tex           # Results table (LaTeX)
└── EXPERIMENTS.md                  # This file
```

---

## 🔬 Advanced Analysis

### Scenario-Specific Analysis

```python
# Compare VLM performance across crowd densities
densities = {
    'sparse': ['S2', 'S3', 'S6'],
    'medium': ['S4', 'S8'],
    'dense': ['S5', 'S7', 'S9']
}

for density, scenarios in densities.items():
    vlm_times = []
    for s in scenarios:
        key = f'{s}_vlm'
        if key in data:
            times = [t['navigation_efficiency']['time_to_goal_sec']
                    for t in data[key]['trials']]
            vlm_times.extend(times)

    print(f"{density}: mean={np.mean(vlm_times):.2f}s, std={np.std(vlm_times):.2f}s")
```

### Adaptive Behavior Analysis

```python
# Analyze VLM adaptive distance across scenarios
for scenario in ['S1', 'S5', 'S7']:
    key = f'{scenario}_vlm'
    if key in data:
        # Extract adaptive distances from VLM logs
        # (requires additional VLM logging)
        pass
```

---

## 📚 Citation

If you use this experimental framework, please cite:

```bibtex
@article{your2024vlmmpc,
  title={VLM-Enhanced Model Predictive Control for Socially-Aware Robot Navigation},
  author={Your Name},
  journal={Your Conference/Journal},
  year={2024}
}
```

---

## 🐛 Troubleshooting

**Problem:** Trials timeout without reaching goal
- **Solution:** Increase timeout or adjust scenario difficulty

**Problem:** VLM API calls failing
- **Solution:** Check VLM server running, verify API endpoint

**Problem:** Metrics extraction fails
- **Solution:** Check CSV log format matches expected columns

**Problem:** Statistical tests show no significance
- **Solution:** Increase number of trials (n > 30 recommended) or adjust scenario difficulty

---

## 📞 Support

For questions or issues:
1. Check logs in `~/ros2_logs/social_mpc_nav/`
2. Verify CSV files are generated correctly
3. Review Python script output for errors

Good luck with your experiments! 🚀
