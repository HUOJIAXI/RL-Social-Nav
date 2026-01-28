# Quick Start: VLM-MPC Comparison Experiments

## 🎯 Goal
Compare VLM-enhanced MPC against baseline (no-VLM) MPC across multiple scenarios and metrics.

## ⚡ Quick Start (5 minutes)

### 1. Run a Single Trial
```bash
cd ~/phd_upm/vlm_mpc_cpp
source install/setup.bash

# Test with VLM
ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
    enable_vlm:=true \
    log_mpc_to_csv:=true
```

**In another terminal:** Publish a goal
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0, z: 0.0}}}"
```

**Wait for robot to reach goal, then check logs:**
```bash
cat ~/ros2_logs/social_mpc_nav/mpc_casadi_log.csv | tail -20
```

### 2. Run Automated Experiments
```bash
# Install dependencies
pip install pandas numpy matplotlib seaborn scipy

# Run 3 trials for scenario S1 with both conditions
./scripts/run_experiments.sh S1 vlm 3
./scripts/run_experiments.sh S1 no_vlm 3
```

### 3. Analyze Results
```bash
# Extract metrics
python scripts/extract_metrics.py \
    --log-dir ~/ros2_logs/social_mpc_nav/experiments \
    --scenarios S1 \
    --conditions no_vlm vlm \
    --trials 3 \
    --output quick_results.json

# Visualize
python scripts/visualize_comparison.py \
    --metrics quick_results.json \
    --output quick_plots/
```

**Check results:**
```bash
ls quick_plots/
# Should see: time_to_goal_comparison.png, aggregate_comparison.png, etc.

cat quick_plots/summary_table.csv
```

---

## 📊 Key Metrics to Check

### Navigation Efficiency
✅ **Time to Goal** - VLM should be faster in dense scenarios (S5, S7, S9)
✅ **Path Efficiency** - VLM should have straighter paths

### Safety
✅ **Min Obstacle Distance** - Both should be > 0.2m (hard constraint)
✅ **Hard Violations** - Both should be ZERO

### Social Compliance
✅ **Comfort Score** - VLM should be higher (more respectful distances)
✅ **Personal Space Violations** - VLM should be lower

---

## 📁 What Gets Logged

Each trial generates:
```
~/ros2_logs/social_mpc_nav/experiments/
├── S1_vlm_trial0_mpc_casadi_log.csv
├── S1_vlm_trial1_mpc_casadi_log.csv
├── S1_no_vlm_trial0_mpc_casadi_log.csv
└── ...
```

**CSV columns:**
- `timestamp, x, y, yaw` - Robot pose
- `v_cmd, w_cmd` - Commanded velocities
- `goal_dist` - Distance to goal
- `solve_time_ms` - MPC computation time
- `min_obs_dist, min_person_dist` - Safety metrics
- `total_violations, violation_rate` - Safety violations

---

## 🎨 Expected Visualizations

After running the analysis, you'll get:

### 1. Box Plots (per scenario)
![Time Comparison](docs/example_time_comparison.png)
- Shows distribution across trials
- Statistical significance (*, **, ***)

### 2. Aggregate Comparison
![Aggregate](docs/example_aggregate.png)
- Bar charts comparing all metrics
- Green border = VLM performs better

### 3. Summary Table
```
Scenario | Condition | Time (s) | Path Eff | Min Obs (m) | Comfort | p-value
---------|-----------|----------|----------|-------------|---------|--------
S1       | No-VLM    | 45.2±3.1 | 0.92±0.04| 0.31±0.05   | 0.78    | -
S1       | VLM-MPC   | 44.8±2.9 | 0.93±0.03| 0.32±0.04   | 0.79    | 0.421
```

---

## 🔬 Full Experimental Suite

For publication-quality results:

```bash
# Run all 10 scenarios, both conditions, 10 trials each
# Total: 200 trials (~40 hours)

for scenario in S1 S2 S3 S4 S5 S6 S7 S8 S9 S10; do
    for condition in no_vlm vlm; do
        ./scripts/run_experiments.sh $scenario $condition 10
    done
done

# Extract and visualize
python scripts/extract_metrics.py \
    --log-dir ~/ros2_logs/social_mpc_nav/experiments \
    --scenarios S1 S2 S3 S4 S5 S6 S7 S8 S9 S10 \
    --output final_results.json

python scripts/visualize_comparison.py \
    --metrics final_results.json \
    --output final_plots/
```

---

## 💡 Tips for Better Results

### 1. Ensure Consistent Environment
- Same Gazebo world for all trials
- Same pedestrian spawn positions (with controlled randomness)
- Same obstacle layout

### 2. Verify VLM is Working
```bash
# Check VLM is providing valid outputs (not fallback)
grep "VLM WARMED UP" ~/.ros/log/*  # Should see confirmation message
```

### 3. Monitor During Trials
```bash
# In separate terminal, monitor live metrics
watch -n 1 'tail -5 ~/ros2_logs/social_mpc_nav/mpc_casadi_log.csv'
```

### 4. Validate Data Quality
```bash
# Check for incomplete trials
wc -l ~/ros2_logs/social_mpc_nav/experiments/*.csv
# All should have similar line counts (within ±20%)
```

---

## 🐛 Common Issues

**"VLM fallback - using cached/rules"**
→ VLM server not running or API call failed
→ Check: `ros2 topic echo /vlm/mpc_parameters` - source should be "vlm"

**"MPC solve time > 100ms"**
→ Optimization struggling (too many constraints or bad initial guess)
→ Solution: Reduce horizon (N) or increase tolerance

**"Robot stops in middle of path"**
→ Likely obstacle or person too close triggering safety stop
→ Check: `min_obs_dist` and `min_person_dist` in logs

---

## 📈 Interpreting Results

### When VLM Should Win
- **Dense scenarios (S5, S7, S9):** VLM adapts distances → faster navigation
- **Social scenarios (S6, S7, S8):** VLM yields appropriately → higher comfort
- **Doorways/crossings:** VLM allows tighter navigation → more efficient

### When VLM ≈ No-VLM
- **Empty/sparse scenarios (S1, S2):** Little benefit from adaptation
- **Safety metrics:** Both respect hard constraints
- **Solve time:** VLM overhead should be minimal

### Red Flags
- ❌ **VLM has hard constraint violations** → Bug in implementation!
- ❌ **VLM slower in all scenarios** → VLM hurting performance
- ❌ **No statistical significance** → Need more trials or clearer scenarios

---

See **EXPERIMENTS.md** for comprehensive documentation.

Happy experimenting! 🚀
