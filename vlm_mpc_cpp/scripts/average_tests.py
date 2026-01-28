#!/usr/bin/env python3
"""
Average metrics across multiple test runs and create comparison.

Usage:
    python average_tests.py --tests test1 test2 --output averaged_results/
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import List

# Set style
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (20, 12)


def load_summary_csv(test_dir: Path) -> pd.DataFrame:
    """Load summary CSV from a test directory."""
    summary_file = test_dir / 'three_way_summary.csv'
    if not summary_file.exists():
        raise FileNotFoundError(f"Summary file not found: {summary_file}")
    return pd.read_csv(summary_file)


def average_metrics(test_dirs: List[Path]) -> pd.DataFrame:
    """Average metrics across multiple test runs."""
    all_summaries = []

    for test_dir in test_dirs:
        df = load_summary_csv(test_dir)
        all_summaries.append(df)

    # Concatenate all summaries
    combined = pd.concat(all_summaries, ignore_index=True)

    # Group by method and calculate mean and std
    grouped = combined.groupby('method')

    avg_metrics = grouped.mean(numeric_only=True)
    std_metrics = grouped.std(numeric_only=True)

    # Create result dataframe with mean ± std
    result = avg_metrics.copy()

    # Add std as additional columns
    for col in std_metrics.columns:
        result[f'{col}_std'] = std_metrics[col]

    # Reset index to make 'method' a column
    result = result.reset_index()

    return result, avg_metrics, std_metrics


def plot_averaged_comparison(avg_df: pd.DataFrame, std_df: pd.DataFrame, output_dir: Path, n_tests: int):
    """Create comprehensive bar chart comparison with error bars."""
    methods = avg_df.index.tolist()
    colors = ['#ff9999', '#66b3ff', '#99ff99']  # No-VLM, VLM, DWA

    # Map method names to colors
    method_colors = {}
    for i, method in enumerate(methods):
        if 'VLM-MPC' in method:
            method_colors[method] = '#66b3ff'
        elif 'No VLM' in method or 'MPC' in method:
            method_colors[method] = '#ff9999'
        elif 'DWA' in method:
            method_colors[method] = '#99ff99'
        else:
            method_colors[method] = colors[i % len(colors)]

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

        if metric_key not in avg_df.columns:
            continue

        values = avg_df[metric_key].values
        errors = std_df[metric_key].values if metric_key in std_df.columns else np.zeros_like(values)
        x_pos = np.arange(len(methods))

        bar_colors = [method_colors.get(m, '#cccccc') for m in methods]
        bars = ax.bar(x_pos, values, yerr=errors, color=bar_colors, alpha=0.8,
                     edgecolor='black', linewidth=1.5, capsize=5)

        # Highlight best performance
        if better_dir and not all(np.isnan(values)):
            valid_idx = ~np.isnan(values)
            if np.any(valid_idx):
                if better_dir == 'higher_better':
                    best_idx = np.nanargmax(values)
                else:
                    best_idx = np.nanargmin(values)

                bars[best_idx].set_edgecolor('green')
                bars[best_idx].set_linewidth(3)

        ax.set_ylabel(ylabel, fontweight='bold', fontsize=10)
        ax.set_xticks(x_pos)
        ax.set_xticklabels(methods, rotation=15, ha='right', fontsize=9)
        ax.grid(True, alpha=0.3, axis='y')

        # Add value labels on bars
        for i, (v, e) in enumerate(zip(values, errors)):
            if not np.isnan(v):
                if is_small:
                    label = f'{v:.4f}±{e:.4f}'
                elif v < 1:
                    label = f'{v:.3f}±{e:.3f}'
                else:
                    label = f'{v:.1f}±{e:.1f}'
                ax.text(i, v + e, label, ha='center', va='bottom', fontsize=7, fontweight='bold')

    plt.suptitle('Averaged Navigation Comparison', fontsize=16, fontweight='bold')
    plt.tight_layout()

    output_file = output_dir / 'averaged_comparison.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_file}")


def create_averaged_report(result_df: pd.DataFrame, avg_df: pd.DataFrame,
                          std_df: pd.DataFrame, output_dir: Path, n_tests: int):
    """Create comprehensive averaged report."""

    # Save CSV
    csv_file = output_dir / 'averaged_summary.csv'
    result_df.to_csv(csv_file, index=False, float_format='%.4f')
    print(f"Saved: {csv_file}")

    # Create markdown report
    report_file = output_dir / 'AVERAGED_COMPARISON_REPORT.md'
    with open(report_file, 'w') as f:
        f.write(f"# Averaged Navigation Comparison\n\n")
        f.write(f"**Generated:** {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write("---\n\n")

        # Executive Summary
        f.write("## Executive Summary\n\n")
        f.write("All values shown as: **Mean ± Std Dev**\n\n")
        f.write("| Metric | VLM-MPC | MPC (No VLM) | DWA | Winner |\n")
        f.write("|--------|---------|--------------|-----|--------|\n")

        metrics_to_compare = [
            ('time_to_goal', 'Time to Goal (s)', 'lower'),
            ('path_efficiency', 'Path Efficiency', 'higher'),
            ('avg_min_obs_dist', 'Avg Obstacle Dist (m)', 'higher'),
            ('avg_min_person_dist', 'Avg Person Dist (m)', 'higher'),
            ('social_avoidance_turn_rate', 'Social Avoidance Rate (turns/m)', None),
            ('personal_space_violation_rate', 'Personal Space Violation Rate (viols/m)', 'lower'),
            ('personal_zone_time_pct', 'Time in Personal Zone (%)', 'lower'),
            ('social_zone_time_pct', 'Time in Social Zone (%)', 'higher'),
            ('avg_velocity_near_pedestrians', 'Avg Velocity Near Peds (m/s)', 'lower'),
            ('proactive_avoidance_pct', 'Proactive Avoidance (%)', 'higher'),
        ]

        for metric_key, metric_name, better in metrics_to_compare:
            if metric_key not in avg_df.columns:
                continue

            row_values = []
            valid_methods = []

            for method in ['VLM-MPC', 'MPC (No VLM)', 'DWA']:
                if method in avg_df.index:
                    avg_val = avg_df.loc[method, metric_key]
                    std_val = std_df.loc[method, metric_key] if method in std_df.index else 0

                    if np.isnan(avg_val):
                        row_values.append('N/A')
                    else:
                        valid_methods.append((method, avg_val))
                        if metric_key in ['avg_linear_smoothness', 'avg_angular_smoothness']:
                            row_values.append(f'{avg_val:.5f}±{std_val:.5f}')
                        elif avg_val < 1:
                            row_values.append(f'{avg_val:.3f}±{std_val:.3f}')
                        else:
                            row_values.append(f'{avg_val:.2f}±{std_val:.2f}')
                else:
                    row_values.append('N/A')

            # Determine winner
            winner = 'N/A'
            if valid_methods and better is not None:
                if better == 'lower':
                    winner = min(valid_methods, key=lambda x: x[1])[0]
                else:
                    winner = max(valid_methods, key=lambda x: x[1])[0]

            f.write(f"| {metric_name} | {row_values[0]} | {row_values[1]} | {row_values[2]} | **{winner}** |\n")

        # Detailed Metrics
        f.write("\n---\n\n")
        f.write("## Detailed Performance Metrics\n\n")

        # Navigation Performance
        f.write("### 1. Navigation Performance\n\n")
        for method in avg_df.index:
            f.write(f"**{method}:**\n")
            f.write(f"- Time to Goal: {avg_df.loc[method, 'time_to_goal']:.2f} ± {std_df.loc[method, 'time_to_goal']:.2f} s\n")
            f.write(f"- Path Length: {avg_df.loc[method, 'path_length']:.2f} ± {std_df.loc[method, 'path_length']:.2f} m\n")
            f.write(f"- Path Efficiency: {avg_df.loc[method, 'path_efficiency']:.3f} ± {std_df.loc[method, 'path_efficiency']:.3f}\n")
            f.write(f"- Avg Linear Velocity: {avg_df.loc[method, 'avg_linear_vel']:.3f} ± {std_df.loc[method, 'avg_linear_vel']:.3f} m/s\n")
            f.write(f"- Avg Angular Velocity: {avg_df.loc[method, 'avg_angular_vel']:.3f} ± {std_df.loc[method, 'avg_angular_vel']:.3f} rad/s\n\n")

        # Safety Performance
        f.write("### 2. Safety Performance\n\n")
        for method in avg_df.index:
            f.write(f"**{method}:**\n")
            f.write(f"- Avg Min Obstacle Distance: {avg_df.loc[method, 'avg_min_obs_dist']:.3f} ± {std_df.loc[method, 'avg_min_obs_dist']:.3f} m\n")
            f.write(f"- Closest Obstacle Approach: {avg_df.loc[method, 'closest_obs_dist']:.3f} ± {std_df.loc[method, 'closest_obs_dist']:.3f} m\n")
            f.write(f"- Safety Violation Rate: {avg_df.loc[method, 'violation_rate']:.2f} ± {std_df.loc[method, 'violation_rate']:.2f}%\n")
            f.write(f"- Total Violations (avg): {avg_df.loc[method, 'total_violations']:.1f} ± {std_df.loc[method, 'total_violations']:.1f}\n\n")

        # Social Compliance
        f.write("### 3. Social Compliance\n\n")
        for method in avg_df.index:
            if not np.isnan(avg_df.loc[method, 'avg_min_person_dist']):
                f.write(f"**{method}:**\n")
                f.write(f"- Avg Min Person Distance: {avg_df.loc[method, 'avg_min_person_dist']:.3f} ± {std_df.loc[method, 'avg_min_person_dist']:.3f} m\n")
                f.write(f"- Closest Person Approach: {avg_df.loc[method, 'closest_person_dist']:.3f} ± {std_df.loc[method, 'closest_person_dist']:.3f} m\n")
                f.write(f"- Person Encounters (avg): {avg_df.loc[method, 'person_encounters']:.1f} ± {std_df.loc[method, 'person_encounters']:.1f}\n\n")

        # Computational Performance
        f.write("### 4. Computational Performance\n\n")
        for method in avg_df.index:
            if 'avg_solve_time' in avg_df.columns and not np.isnan(avg_df.loc[method, 'avg_solve_time']):
                f.write(f"**{method}:**\n")
                f.write(f"- Avg Solve Time: {avg_df.loc[method, 'avg_solve_time']:.2f} ± {std_df.loc[method, 'avg_solve_time']:.2f} ms\n")
                if 'max_solve_time' in avg_df.columns:
                    f.write(f"- Max Solve Time: {avg_df.loc[method, 'max_solve_time']:.2f} ± {std_df.loc[method, 'max_solve_time']:.2f} ms\n")
                if 'min_solve_time' in avg_df.columns:
                    f.write(f"- Min Solve Time: {avg_df.loc[method, 'min_solve_time']:.2f} ± {std_df.loc[method, 'min_solve_time']:.2f} ms\n")
                f.write("\n")
            else:
                f.write(f"**{method}:** N/A (non-optimization based)\n\n")

        # Control Smoothness
        f.write("### 5. Control Smoothness\n\n")
        for method in avg_df.index:
            f.write(f"**{method}:**\n")
            f.write(f"- Linear Smoothness: {avg_df.loc[method, 'avg_linear_smoothness']:.5f} ± {std_df.loc[method, 'avg_linear_smoothness']:.5f} m/s²\n")
            f.write(f"- Angular Smoothness: {avg_df.loc[method, 'avg_angular_smoothness']:.5f} ± {std_df.loc[method, 'avg_angular_smoothness']:.5f} rad/s²\n\n")

    print(f"Saved: {report_file}")


def main():
    parser = argparse.ArgumentParser(description='Average metrics across multiple test runs')
    parser.add_argument('--tests', nargs='+', required=True, help='Test directories to average')
    parser.add_argument('--output', type=str, default='averaged_results', help='Output directory')

    args = parser.parse_args()

    # Convert test names to paths
    base_dir = Path('/home/huojiaxi/ros2_logs/social_mpc_nav/experiments/res')
    test_dirs = [base_dir / test for test in args.tests]

    # Verify all test directories exist
    for test_dir in test_dirs:
        if not test_dir.exists():
            raise FileNotFoundError(f"Test directory not found: {test_dir}")

    # Create output directory
    output_dir = base_dir / args.output
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Averaging metrics across {len(test_dirs)} test runs...")
    print(f"Tests: {[d.name for d in test_dirs]}")

    # Average metrics
    result_df, avg_df, std_df = average_metrics(test_dirs)

    # Print summary
    print("\n" + "="*80)
    print("AVERAGED COMPARISON")
    print("="*80)
    for method in avg_df.index:
        print(f"\n{method}:")
        print(f"  Time to Goal: {avg_df.loc[method, 'time_to_goal']:.2f} ± {std_df.loc[method, 'time_to_goal']:.2f} s")
        print(f"  Path Efficiency: {avg_df.loc[method, 'path_efficiency']:.3f} ± {std_df.loc[method, 'path_efficiency']:.3f}")
        print(f"  Avg Min Obstacle Dist: {avg_df.loc[method, 'avg_min_obs_dist']:.3f} ± {std_df.loc[method, 'avg_min_obs_dist']:.3f} m")
        print(f"  Violation Rate: {avg_df.loc[method, 'violation_rate']:.2f} ± {std_df.loc[method, 'violation_rate']:.2f}%")
        if not np.isnan(avg_df.loc[method, 'avg_solve_time']):
            print(f"  Avg Solve Time: {avg_df.loc[method, 'avg_solve_time']:.2f} ± {std_df.loc[method, 'avg_solve_time']:.2f} ms")

    print("\n" + "="*80)
    print("Generating visualizations...")

    # Generate plots
    plot_averaged_comparison(avg_df, std_df, output_dir, len(test_dirs))

    # Generate report
    create_averaged_report(result_df, avg_df, std_df, output_dir, len(test_dirs))

    print("\nAveraging complete!")
    print(f"Results saved to: {output_dir.absolute()}")


if __name__ == '__main__':
    main()
