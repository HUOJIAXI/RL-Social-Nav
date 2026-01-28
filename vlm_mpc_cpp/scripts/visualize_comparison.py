#!/usr/bin/env python3
"""
Visualize comparison between VLM-MPC, No-VLM MPC, and DWA.

Usage:
    python visualize_comparison.py --vlm vlm.csv --no_vlm no_vlm.csv --dwa dwa.csv --output plots/
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from scipy import stats


# Set style
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (15, 10)


def load_csv_data(csv_file: str) -> pd.DataFrame:
    """Load data from CSV file."""
    # Read CSV and handle infinity values
    df = pd.read_csv(csv_file)

    # Replace extremely large numbers (DBL_MAX) with NaN
    # DBL_MAX is approximately 1.7976931348623157e+308
    for col in df.columns:
        if df[col].dtype in ['float64', 'float32', 'object']:
            # Convert to numeric if needed, coercing errors to NaN
            df[col] = pd.to_numeric(df[col], errors='coerce')
            # Replace values larger than 1e100 with NaN (these are effectively infinity)
            df.loc[df[col] > 1e100, col] = np.nan
            df.loc[df[col] < -1e100, col] = np.nan

    # Replace any remaining inf/-inf with NaN
    df = df.replace([np.inf, -np.inf], np.nan)

    return df


def calculate_metrics(df: pd.DataFrame, method_name: str) -> dict:
    """Calculate comprehensive metrics from CSV data."""
    # Remove inf values for calculations
    df_clean = df.replace([np.inf, -np.inf], np.nan)

    # Time to goal
    timestamps = df['timestamp'].values
    time_to_goal = timestamps[-1] - timestamps[0]

    # Path length (approximate from x, y coordinates)
    x = df['x'].values
    y = df['y'].values
    path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))

    # Goal distance (straight line distance from start to end)
    goal_distance = df['goal_dist'].iloc[0]
    straight_line_dist = goal_distance - df['goal_dist'].iloc[-1]
    path_efficiency = straight_line_dist / path_length if path_length > 0 else 0

    # Velocity statistics
    avg_linear_vel = df['v_cmd'].mean()
    avg_angular_vel = df['w_cmd'].abs().mean()

    # Smoothness (velocity changes)
    linear_vel_changes = np.abs(np.diff(df['v_cmd'].values))
    angular_vel_changes = np.abs(np.diff(df['w_cmd'].values))
    avg_linear_smoothness = np.mean(linear_vel_changes)
    avg_angular_smoothness = np.mean(angular_vel_changes)

    # Safety metrics
    min_obs_dist_values = df_clean['min_obs_dist'].dropna()
    avg_min_obs_dist = min_obs_dist_values.mean() if len(min_obs_dist_values) > 0 else np.nan
    closest_obs_dist = min_obs_dist_values.min() if len(min_obs_dist_values) > 0 else np.nan

    # Social distance metrics
    min_person_dist_values = df_clean['min_person_dist'].dropna()
    avg_min_person_dist = min_person_dist_values.mean() if len(min_person_dist_values) > 0 else np.nan
    closest_person_dist = min_person_dist_values.min() if len(min_person_dist_values) > 0 else np.nan
    person_encounters = len(min_person_dist_values)

    # Social avoidance maneuvers - detect turns near pedestrians
    social_avoidance_turns = 0
    if 'yaw' in df.columns and 'min_person_dist' in df.columns:
        yaw = df['yaw'].values
        person_dist = df_clean['min_person_dist'].values

        # Calculate heading changes
        yaw_diff = np.abs(np.diff(yaw))
        # Handle angle wrapping (if difference > pi, it wrapped around)
        yaw_diff = np.minimum(yaw_diff, 2*np.pi - yaw_diff)

        # Define thresholds
        significant_turn_threshold = np.deg2rad(8)  # 8 degrees - more sensitive to social avoidance
        social_distance_threshold = 3.0  # meters - consider turns within 3m of person as social avoidance

        # Detect significant turns near pedestrians
        for i in range(len(yaw_diff)):
            if yaw_diff[i] > significant_turn_threshold:
                # Check if person was nearby (look at window around the turn)
                window_start = max(0, i-2)
                window_end = min(len(person_dist), i+3)
                nearby_person_dists = person_dist[window_start:window_end]

                # Filter out NaN values
                nearby_person_dists = nearby_person_dists[~np.isnan(nearby_person_dists)]

                if len(nearby_person_dists) > 0 and np.min(nearby_person_dists) < social_distance_threshold:
                    social_avoidance_turns += 1

    # Additional social navigation metrics (using rates for comparability)
    personal_space_violation_rate = 0.0  # violations per meter
    personal_space_violation_time_pct = 0.0  # percentage of time
    intimate_zone_time_pct = 0.0
    personal_zone_time_pct = 0.0
    social_zone_time_pct = 0.0
    public_zone_time_pct = 0.0
    avg_velocity_near_pedestrians = np.nan
    proactive_avoidance_pct = 0.0  # percentage of avoidance maneuvers that are proactive
    reactive_avoidance_rate = 0.0  # per meter
    social_avoidance_turn_rate = 0.0  # per meter
    pedestrian_interaction_percentage = 0.0

    if 'min_person_dist' in df.columns and 'v_cmd' in df.columns:
        person_dist = df_clean['min_person_dist'].values
        velocity = df['v_cmd'].values

        # Calculate time deltas
        if len(df) > 1:
            time_deltas = np.diff(timestamps)
            avg_dt = np.mean(time_deltas)
        else:
            avg_dt = 0.1  # default 10Hz

        # 1. Personal Space Violations (< 1.2m) - as rate
        personal_space_mask = person_dist < 1.2
        personal_space_violations_count = np.sum(personal_space_mask)
        personal_space_violation_time = np.sum(personal_space_mask) * avg_dt

        # Rate: violations per meter traveled
        personal_space_violation_rate = (personal_space_violations_count / path_length) if path_length > 0 else 0
        # Percentage of time
        personal_space_violation_time_pct = (personal_space_violation_time / time_to_goal * 100) if time_to_goal > 0 else 0

        # 2. Social Zone Compliance - as percentages
        # Intimate: < 0.45m, Personal: 0.45-1.2m, Social: 1.2-3.6m, Public: > 3.6m
        intimate_mask = person_dist < 0.45
        personal_mask = (person_dist >= 0.45) & (person_dist < 1.2)
        social_mask = (person_dist >= 1.2) & (person_dist < 3.6)
        public_mask = person_dist >= 3.6

        time_in_intimate_zone = np.sum(intimate_mask & ~np.isnan(person_dist)) * avg_dt
        time_in_personal_zone = np.sum(personal_mask & ~np.isnan(person_dist)) * avg_dt
        time_in_social_zone = np.sum(social_mask & ~np.isnan(person_dist)) * avg_dt
        time_in_public_zone = np.sum(public_mask & ~np.isnan(person_dist)) * avg_dt

        # Convert to percentages
        intimate_zone_time_pct = (time_in_intimate_zone / time_to_goal * 100) if time_to_goal > 0 else 0
        personal_zone_time_pct = (time_in_personal_zone / time_to_goal * 100) if time_to_goal > 0 else 0
        social_zone_time_pct = (time_in_social_zone / time_to_goal * 100) if time_to_goal > 0 else 0
        public_zone_time_pct = (time_in_public_zone / time_to_goal * 100) if time_to_goal > 0 else 0

        # 3. Velocity Near Pedestrians (within 3m) - already a rate
        near_pedestrian_mask = (person_dist < 3.0) & ~np.isnan(person_dist)
        if np.sum(near_pedestrian_mask) > 0:
            avg_velocity_near_pedestrians = np.mean(velocity[near_pedestrian_mask])

        # 4. Proactive vs Reactive Avoidance - as rates per meter
        proactive_count = 0
        reactive_count = 0
        if 'yaw' in df.columns and len(df) > 1:
            yaw = df['yaw'].values
            yaw_diff = np.abs(np.diff(yaw))
            yaw_diff = np.minimum(yaw_diff, 2*np.pi - yaw_diff)

            turn_threshold = np.deg2rad(8)

            for i in range(len(yaw_diff)):
                if yaw_diff[i] > turn_threshold:
                    # Check person distance at turn point
                    if i < len(person_dist) and not np.isnan(person_dist[i]):
                        if person_dist[i] > 2.0 and person_dist[i] < 3.5:
                            # Proactive: turning when person is >2m away
                            proactive_count += 1
                        elif person_dist[i] <= 2.0:
                            # Reactive: turning when person is <2m away
                            reactive_count += 1

        # Calculate proactive avoidance as percentage of total avoidance maneuvers
        total_avoidance_maneuvers = proactive_count + reactive_count
        proactive_avoidance_pct = (proactive_count / total_avoidance_maneuvers * 100) if total_avoidance_maneuvers > 0 else 0
        reactive_avoidance_rate = (reactive_count / path_length) if path_length > 0 else 0

        # Social avoidance turn rate
        social_avoidance_turn_rate = (social_avoidance_turns / path_length) if path_length > 0 else 0

        # 5. Pedestrian Interaction Time - already as percentage
        interaction_mask = (person_dist < 3.0) & ~np.isnan(person_dist)
        pedestrian_interaction_time = np.sum(interaction_mask) * avg_dt
        pedestrian_interaction_percentage = (pedestrian_interaction_time / time_to_goal * 100) if time_to_goal > 0 else 0

    # MPC-specific metrics (if available)
    if 'solve_time_ms' in df.columns:
        avg_solve_time = df['solve_time_ms'].mean()
        max_solve_time = df['solve_time_ms'].max()
        min_solve_time = df['solve_time_ms'].min()
    else:
        avg_solve_time = np.nan
        max_solve_time = np.nan
        min_solve_time = np.nan

    if 'cost' in df.columns:
        avg_cost = df_clean['cost'].mean()
        final_cost = df_clean['cost'].iloc[-1]
    else:
        avg_cost = np.nan
        final_cost = np.nan

    if 'safety_verified' in df.columns:
        total_checks = df['total_checks'].iloc[-1] if 'total_checks' in df.columns else len(df)
        total_violations = df['total_violations'].iloc[-1] if 'total_violations' in df.columns else 0
        violation_rate = (total_violations / total_checks * 100) if total_checks > 0 else 0
    else:
        total_checks = len(df)
        total_violations = 0
        violation_rate = 0

    return {
        'method': method_name,
        'time_to_goal': time_to_goal,
        'path_length': path_length,
        'path_efficiency': path_efficiency,
        'avg_linear_vel': avg_linear_vel,
        'avg_angular_vel': avg_angular_vel,
        'avg_linear_smoothness': avg_linear_smoothness,
        'avg_angular_smoothness': avg_angular_smoothness,
        'avg_min_obs_dist': avg_min_obs_dist,
        'closest_obs_dist': closest_obs_dist,
        'avg_min_person_dist': avg_min_person_dist,
        'closest_person_dist': closest_person_dist,
        'person_encounters': person_encounters,
        'social_avoidance_turn_rate': social_avoidance_turn_rate,
        'personal_space_violation_rate': personal_space_violation_rate,
        'personal_space_violation_time_pct': personal_space_violation_time_pct,
        'intimate_zone_time_pct': intimate_zone_time_pct,
        'personal_zone_time_pct': personal_zone_time_pct,
        'social_zone_time_pct': social_zone_time_pct,
        'public_zone_time_pct': public_zone_time_pct,
        'avg_velocity_near_pedestrians': avg_velocity_near_pedestrians,
        'proactive_avoidance_pct': proactive_avoidance_pct,
        'reactive_avoidance_rate': reactive_avoidance_rate,
        'pedestrian_interaction_percentage': pedestrian_interaction_percentage,
        'avg_solve_time': avg_solve_time,
        'max_solve_time': max_solve_time,
        'min_solve_time': min_solve_time,
        'avg_cost': avg_cost,
        'final_cost': final_cost,
        'total_checks': total_checks,
        'total_violations': total_violations,
        'violation_rate': violation_rate,
        'total_samples': len(df)
    }


def plot_comparison_bars(metrics_list: list, output_dir: Path):
    """Create comprehensive bar chart comparison across all three methods."""
    methods = [m['method'] for m in metrics_list]
    colors = ['#ff9999', '#66b3ff', '#99ff99']  # No-VLM, VLM, DWA

    fig = plt.figure(figsize=(20, 12))

    # Define metrics to plot
    plot_specs = [
        ('time_to_goal', 'Time to Goal (s)', False, 'lower_better'),
        ('path_length', 'Path Length (m)', False, 'lower_better'),
        ('path_efficiency', 'Path Efficiency', False, 'higher_better'),
        ('avg_linear_vel', 'Avg Linear Velocity (m/s)', False, None),
        ('avg_angular_vel', 'Avg Angular Velocity (rad/s)', False, None),
        ('avg_linear_smoothness', 'Linear Smoothness\n(avg |Δv|)', True, 'lower_better'),
        ('avg_angular_smoothness', 'Angular Smoothness\n(avg |Δω|)', True, 'lower_better'),
        ('avg_min_obs_dist', 'Avg Min Obstacle Dist (m)', False, 'higher_better'),
        ('closest_obs_dist', 'Closest Obstacle Dist (m)', False, 'higher_better'),
        ('avg_min_person_dist', 'Avg Min Person Dist (m)', False, 'higher_better'),
        ('closest_person_dist', 'Closest Person Dist (m)', False, 'higher_better'),
        ('social_avoidance_turn_rate', 'Social Avoidance Rate\n(turns/m)', True, None),
        ('personal_space_violation_rate', 'Personal Space Violation\nRate (viols/m)', True, 'lower_better'),
        ('personal_zone_time_pct', 'Time in Personal Zone (%)', False, 'lower_better'),
        ('social_zone_time_pct', 'Time in Social Zone (%)', False, 'higher_better'),
        ('avg_velocity_near_pedestrians', 'Avg Velocity Near Peds (m/s)', False, 'lower_better'),
        ('proactive_avoidance_pct', 'Proactive Avoidance\n(%)', False, 'higher_better'),
        ('reactive_avoidance_rate', 'Reactive Avoidance\nRate (per m)', True, 'lower_better'),
        ('pedestrian_interaction_percentage', 'Pedestrian Interaction (%)', False, None),
    ]

    n_plots = len(plot_specs)
    n_cols = 4
    n_rows = (n_plots + n_cols - 1) // n_cols

    for idx, (metric_key, ylabel, is_small, better_dir) in enumerate(plot_specs):
        ax = plt.subplot(n_rows, n_cols, idx + 1)

        values = [m.get(metric_key, np.nan) for m in metrics_list]
        x_pos = np.arange(len(methods))

        bars = ax.bar(x_pos, values, color=colors[:len(methods)], alpha=0.8, edgecolor='black', linewidth=1.5)

        # Highlight best performance
        if better_dir and not all(np.isnan(values)):
            valid_values = [v for v in values if not np.isnan(v)]
            if len(valid_values) > 0:
                if better_dir == 'higher_better':
                    best_val = max(valid_values)
                else:
                    best_val = min(valid_values)

                for i, v in enumerate(values):
                    if not np.isnan(v) and v == best_val:
                        bars[i].set_edgecolor('green')
                        bars[i].set_linewidth(3)

        ax.set_ylabel(ylabel, fontweight='bold', fontsize=10)
        ax.set_xticks(x_pos)
        ax.set_xticklabels(methods, rotation=15, ha='right', fontsize=9)
        ax.grid(True, alpha=0.3, axis='y')

        # Add value labels on bars
        for i, v in enumerate(values):
            if not np.isnan(v):
                if is_small:
                    label = f'{v:.4f}'
                elif v < 1:
                    label = f'{v:.3f}'
                else:
                    label = f'{v:.1f}'
                ax.text(i, v, label, ha='center', va='bottom', fontsize=8, fontweight='bold')

    plt.tight_layout()
    output_file = output_dir / 'three_way_comparison.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_file}")


def create_summary_table(metrics_list: list, output_dir: Path):
    """Create comprehensive summary table and report."""
    # Create DataFrame from metrics
    df = pd.DataFrame(metrics_list)

    # Reorder columns for better readability
    column_order = [
        'method',
        'time_to_goal',
        'path_length',
        'path_efficiency',
        'avg_linear_vel',
        'avg_angular_vel',
        'avg_linear_smoothness',
        'avg_angular_smoothness',
        'avg_min_obs_dist',
        'closest_obs_dist',
        'avg_min_person_dist',
        'closest_person_dist',
        'person_encounters',
        'social_avoidance_turn_rate',
        'personal_space_violation_rate',
        'personal_space_violation_time_pct',
        'intimate_zone_time_pct',
        'personal_zone_time_pct',
        'social_zone_time_pct',
        'public_zone_time_pct',
        'avg_velocity_near_pedestrians',
        'proactive_avoidance_pct',
        'reactive_avoidance_rate',
        'pedestrian_interaction_percentage',
        'violation_rate',
        'total_violations',
        'avg_solve_time',
        'max_solve_time',
        'total_samples'
    ]

    df_ordered = df[[col for col in column_order if col in df.columns]]

    # Save as CSV
    csv_file = output_dir / 'three_way_summary.csv'
    df_ordered.to_csv(csv_file, index=False, float_format='%.4f')
    print(f"Saved: {csv_file}")

    # Create markdown report
    report_file = output_dir / 'THREE_WAY_COMPARISON_REPORT.md'
    with open(report_file, 'w') as f:
        f.write("# Three-Way Navigation Comparison: VLM-MPC vs MPC vs DWA\n\n")
        f.write(f"**Generated:** {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write("---\n\n")

        # Executive Summary
        f.write("## Executive Summary\n\n")
        f.write("| Metric | VLM-MPC | MPC (No VLM) | DWA | Winner |\n")
        f.write("|--------|---------|--------------|-----|--------|\n")

        metrics_to_compare = [
            ('time_to_goal', 'Time to Goal (s)', 'lower'),
            ('path_efficiency', 'Path Efficiency', 'higher'),
            ('avg_min_obs_dist', 'Avg Obstacle Dist (m)', 'higher'),
            ('avg_min_person_dist', 'Avg Person Dist (m)', 'higher'),
        ]

        for metric_key, metric_name, better in metrics_to_compare:
            values = {m['method']: m.get(metric_key, np.nan) for m in metrics_list}
            valid_values = {k: v for k, v in values.items() if not np.isnan(v)}

            if valid_values:
                if better == 'lower':
                    winner = min(valid_values, key=valid_values.get)
                else:
                    winner = max(valid_values, key=valid_values.get)

                row_values = []
                for method in ['VLM-MPC', 'MPC (No VLM)', 'DWA']:
                    val = values.get(method, np.nan)
                    if np.isnan(val):
                        row_values.append('N/A')
                    elif metric_key in ['avg_linear_smoothness', 'avg_angular_smoothness']:
                        row_values.append(f'{val:.5f}')
                    elif val < 1:
                        row_values.append(f'{val:.3f}')
                    else:
                        row_values.append(f'{val:.2f}')

                f.write(f"| {metric_name} | {row_values[0]} | {row_values[1]} | {row_values[2]} | **{winner}** |\n")

        # Detailed Metrics
        f.write("\n---\n\n")
        f.write("## Detailed Performance Metrics\n\n")

        # Navigation Performance
        f.write("### 1. Navigation Performance\n\n")
        for m in metrics_list:
            f.write(f"**{m['method']}:**\n")
            f.write(f"- Time to Goal: {m['time_to_goal']:.2f} s\n")
            f.write(f"- Path Length: {m['path_length']:.2f} m\n")
            f.write(f"- Path Efficiency: {m['path_efficiency']:.3f} ({m['path_efficiency']*100:.1f}%)\n")
            f.write(f"- Avg Linear Velocity: {m['avg_linear_vel']:.3f} m/s\n")
            f.write(f"- Avg Angular Velocity: {m['avg_angular_vel']:.3f} rad/s\n\n")

        # Safety Performance
        f.write("### 2. Safety Performance\n\n")
        for m in metrics_list:
            f.write(f"**{m['method']}:**\n")
            f.write(f"- Avg Min Obstacle Distance: {m['avg_min_obs_dist']:.3f} m\n")
            f.write(f"- Closest Obstacle Approach: {m['closest_obs_dist']:.3f} m\n")
            f.write(f"- Safety Violation Rate: {m['violation_rate']:.2f}%\n")
            f.write(f"- Total Violations: {int(m['total_violations'])}\n\n")

        # Social Compliance
        f.write("### 3. Social Compliance\n\n")
        for m in metrics_list:
            if not np.isnan(m['avg_min_person_dist']):
                f.write(f"**{m['method']}:**\n")
                f.write(f"- Avg Min Person Distance: {m['avg_min_person_dist']:.3f} m\n")
                f.write(f"- Closest Person Approach: {m['closest_person_dist']:.3f} m\n")
                f.write(f"- Person Encounters: {int(m['person_encounters'])}\n\n")

        # Computational Performance
        f.write("### 4. Computational Performance\n\n")
        for m in metrics_list:
            if not np.isnan(m['avg_solve_time']):
                f.write(f"**{m['method']}:**\n")
                f.write(f"- Avg Solve Time: {m['avg_solve_time']:.2f} ms\n")
                f.write(f"- Max Solve Time: {m['max_solve_time']:.2f} ms\n")
                f.write(f"- Min Solve Time: {m['min_solve_time']:.2f} ms\n\n")
            else:
                f.write(f"**{m['method']}:** N/A (non-optimization based)\n\n")

        # Control Smoothness
        f.write("### 5. Control Smoothness\n\n")
        for m in metrics_list:
            f.write(f"**{m['method']}:**\n")
            f.write(f"- Linear Smoothness (avg |Δv|): {m['avg_linear_smoothness']:.5f} m/s²\n")
            f.write(f"- Angular Smoothness (avg |Δω|): {m['avg_angular_smoothness']:.5f} rad/s²\n\n")

    print(f"Saved: {report_file}")


def main():
    parser = argparse.ArgumentParser(description='Three-way comparison: VLM-MPC vs MPC vs DWA')
    parser.add_argument('--vlm', type=str, required=True, help='VLM-MPC CSV file')
    parser.add_argument('--no_vlm', type=str, required=True, help='MPC without VLM CSV file')
    parser.add_argument('--dwa', type=str, required=True, help='DWA CSV file')
    parser.add_argument('--output', type=str, default='.', help='Output directory')

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Loading data files...")
    # Load CSV data
    vlm_df = load_csv_data(args.vlm)
    no_vlm_df = load_csv_data(args.no_vlm)
    dwa_df = load_csv_data(args.dwa)

    print("Calculating metrics...")
    # Calculate metrics for each method
    vlm_metrics = calculate_metrics(vlm_df, 'VLM-MPC')
    no_vlm_metrics = calculate_metrics(no_vlm_df, 'MPC (No VLM)')
    dwa_metrics = calculate_metrics(dwa_df, 'DWA')

    metrics_list = [vlm_metrics, no_vlm_metrics, dwa_metrics]

    # Print summary to console
    print("\n" + "="*80)
    print("THREE-WAY COMPARISON SUMMARY")
    print("="*80)
    for m in metrics_list:
        print(f"\n{m['method']}:")
        print(f"  Time to Goal: {m['time_to_goal']:.2f} s")
        print(f"  Path Efficiency: {m['path_efficiency']:.3f}")
        print(f"  Avg Min Obstacle Dist: {m['avg_min_obs_dist']:.3f} m")
        if not np.isnan(m['avg_min_person_dist']):
            print(f"  Avg Min Person Dist: {m['avg_min_person_dist']:.3f} m")

    print("\n" + "="*80)
    print("Generating visualizations...")

    # Generate plots
    plot_comparison_bars(metrics_list, output_dir)

    # Generate summary table and report
    create_summary_table(metrics_list, output_dir)

    print("\nVisualization complete!")
    print(f"Results saved to: {output_dir.absolute()}")


if __name__ == '__main__':
    main()
