# Parameter Sweep Experiment Framework

Complete framework for fair comparison of DWA, MPC, and VLM-MPC using Pareto frontier analysis.

## Overview

This experiment framework enables fair comparison across navigation methods by:
1. Using **identical pedestrian trajectories** for all methods
2. Performing **systematic parameter sweep** for DWA to explore its configuration space
3. Using **fixed optimal parameters** for MPC and VLM-MPC baselines
4. Analyzing **trade-offs** using Pareto frontier visualization

## Files

- `run_parameter_sweep.py` - Execute parameter sweep experiment
- `pareto_analysis.py` - Analyze results and generate Pareto frontier plots
- `PARAMETER_SWEEP_README.md` - This file

## Quick Start

### Step 1: Create Scenario (Optional)

Create an example scenario file with predefined pedestrian trajectories:

```bash
python run_parameter_sweep.py --create_scenario pedestrian_scenario.yaml --dry_run
```

Edit the scenario file to customize:
- Goal position
- Pedestrian waypoint trajectories
- Static obstacles

### Step 2: Run Parameter Sweep

Execute the full parameter sweep:

```bash
python run_parameter_sweep.py \
  --output_dir sweep_results/ \
  --scenario_file pedestrian_scenario.yaml
```

This will:
- Run DWA with 20 different parameter combinations (5 inflation radii × 4 max velocities)
- Run MPC baseline (no VLM)
- Run VLM-MPC baseline
- Save all results to `sweep_results/`

**Expected duration**: ~15-30 minutes depending on scenario complexity

### Step 3: Analyze Results

Generate Pareto frontier plots:

```bash
python pareto_analysis.py \
  --sweep_results sweep_results/ \
  --output pareto_plots/
```

This creates:
- `pareto_safety_efficiency.png` - Safety vs Efficiency trade-off
- `pareto_smoothness_efficiency.png` - Smoothness vs Efficiency trade-off
- `pareto_combined_comparison.png` - Both plots side-by-side
- `pareto_summary.csv` - Statistical summary table

## DWA Parameter Grid

### Default Parameters

**Inflation Radius** (obstacle/person inflation):
- Values: [0.3, 0.4, 0.5, 0.6, 0.7] meters
- Effect: Larger values increase safety margin but may reduce path efficiency

**Max Linear Velocity**:
- Values: [0.4, 0.5, 0.6, 0.7] m/s
- Effect: Higher velocities reduce time to goal but may compromise safety

**Total configurations**: 5 × 4 = 20

### Customizing Parameter Grid

Edit `run_parameter_sweep.py` lines 24-25:

```python
self.inflation_radii = [0.3, 0.4, 0.5, 0.6, 0.7]  # Your values
self.max_velocities = [0.4, 0.5, 0.6, 0.7]        # Your values
```

## MPC Baseline Parameters

**MPC (No VLM)**:
- Prediction horizon: 20 steps
- Control horizon: 10 steps
- VLM integration: Disabled

**VLM-MPC**:
- Prediction horizon: 20 steps
- Control horizon: 10 steps
- VLM integration: Enabled

## Metrics Recorded

For each run, the following metrics are computed:

### Efficiency
- **Time to Goal** (seconds)
- Path Length (meters)
- Path Efficiency (straight-line distance / path length)

### Safety
- **Minimum Person Distance** (meters) - Closest approach to any pedestrian
- Average Person Distance (meters)
- Closest Obstacle Distance (meters)

### Smoothness
- **Angular Smoothness** - Average |Δω| (angular velocity change)
- **Angular Jerk** - Rate of change of angular acceleration
- Linear Smoothness - Average |Δv| (linear velocity change)

### Social Compliance
- Average Velocity Near Pedestrians (within 3m)
- Personal Space Violation Rate (violations per meter)
- Time in Personal Zone (%)

## Pareto Frontier Analysis

### Plot 1: Safety vs Efficiency
- **X-axis**: Time to Goal (lower is better)
- **Y-axis**: Minimum Person Distance (higher is better)
- **Interpretation**: Shows trade-off between reaching goal quickly vs maintaining safe distance

### Plot 2: Smoothness vs Efficiency
- **X-axis**: Time to Goal (lower is better)
- **Y-axis**: Angular Smoothness (lower is better - smoother motion)
- **Interpretation**: Shows trade-off between speed and smooth, comfortable motion

### Reading the Plots

- **Blue dots**: DWA parameter sweep configurations
- **Red square**: MPC baseline (no VLM)
- **Blue diamond**: VLM-MPC baseline
- **Dashed blue line**: Pareto frontier for DWA (non-dominated solutions)

**Pareto optimal points**: Points on the frontier where you cannot improve one metric without worsening another.

## Advanced Usage

### Run Only DWA Sweep (Skip Baselines)

If you already have baseline data:

```bash
python run_parameter_sweep.py \
  --output_dir sweep_results/ \
  --skip_baselines
```

### Dry Run (Test Configuration)

Preview configuration without running:

```bash
python run_parameter_sweep.py \
  --output_dir sweep_results/ \
  --scenario_file my_scenario.yaml \
  --dry_run
```

### Analyze Existing Results

If you have results from a previous sweep:

```bash
python pareto_analysis.py \
  --sweep_results path/to/existing/results/ \
  --output new_plots/
```

## File Structure

After running the experiment:

```
sweep_results/
├── dwa_r0.3_v0.4.csv          # DWA run (r=0.3m, v=0.4m/s)
├── dwa_r0.3_v0.5.csv          # DWA run (r=0.3m, v=0.5m/s)
├── ...                         # 20 DWA configurations
├── mpc_baseline.csv            # MPC (no VLM) baseline
├── vlm_mpc_baseline.csv        # VLM-MPC baseline
└── dwa_sweep_results.json      # Metadata for DWA runs

pareto_plots/
├── pareto_safety_efficiency.png
├── pareto_smoothness_efficiency.png
├── pareto_combined_comparison.png
└── pareto_summary.csv
```

## Integration with Existing Tests

To use the same scenario from your existing tests:

1. **Extract scenario from test logs**: Create YAML from test1/test2/test3 pedestrian data
2. **Use in sweep**: `--scenario_file extracted_scenario.yaml`
3. **Compare**: Results will be directly comparable to your averaged test results

## Troubleshooting

### Issue: ROS2 launch files not found

**Solution**: Ensure your ROS2 workspace is sourced:
```bash
source /path/to/your/ros2_ws/install/setup.bash
```

### Issue: Timeout during runs

**Solution**: Increase timeout in `run_parameter_sweep.py` line 89:
```python
result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)  # 10 minutes
```

### Issue: No output files created

**Solution**: Check that your launch files accept the parameters:
- `inflation_radius`
- `max_velocity`
- `output_file`
- `scenario_file`

### Issue: Pareto analysis shows no data

**Solution**:
1. Check CSV files exist in sweep_results/
2. Verify CSV format matches expected columns
3. Check for infinity values in data

## Expected Results

Based on previous experiments, we expect:

1. **DWA**:
   - Wide range of configurations on Pareto frontier
   - Some very safe (large inflation) but slow
   - Some fast but less safe (small inflation)

2. **MPC (No VLM)**:
   - Fast and efficient (best time to goal)
   - Moderate safety
   - Very smooth motion

3. **VLM-MPC**:
   - Slower but more socially aware
   - Better safety near pedestrians
   - Smooth motion with social considerations

## Citation

If using this framework, please cite:
```bibtex
@misc{parameter_sweep_framework,
  title={Parameter Sweep Framework for Social Navigation Comparison},
  author={Your Name},
  year={2025}
}
```

## Questions?

For issues or questions:
1. Check CSV log files for errors
2. Verify ROS2 launch file compatibility
3. Review terminal output during sweep execution
