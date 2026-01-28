#!/usr/bin/env python3
"""
Pareto Frontier Analysis for DWA vs MPC vs VLM-MPC

This script analyzes the trade-offs between:
1. Safety (minimum person distance) vs Efficiency (time to goal)
2. Smoothness (angular jerk) vs Efficiency (time to goal)

Usage:
    python pareto_analysis.py --sweep_results sweep_results/ --output plots/
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import List, Tuple
import json

# Set style
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (12, 5)


def load_csv_data(csv_file: str) -> pd.DataFrame:
    """Load data from CSV file and handle infinity values."""
    df = pd.read_csv(csv_file)

    # Replace extremely large numbers (DBL_MAX) with NaN
    for col in df.columns:
        if df[col].dtype in ['float64', 'float32', 'object']:
            df[col] = pd.to_numeric(df[col], errors='coerce')
            df.loc[df[col] > 1e100, col] = np.nan
            df.loc[df[col] < -1e100, col] = np.nan

    df = df.replace([np.inf, -np.inf], np.nan)
    return df


def calculate_sweep_metrics(df: pd.DataFrame, method_name: str, params: dict = None) -> dict:
    """Calculate metrics for a single run."""
    # Remove inf values
    df_clean = df.replace([np.inf, -np.inf], np.nan)

    # Time to goal
    timestamps = df['timestamp'].values
    time_to_goal = timestamps[-1] - timestamps[0]

    # Minimum person distance
    min_person_dist_values = df_clean['min_person_dist'].dropna()
    min_person_dist = min_person_dist_values.min() if len(min_person_dist_values) > 0 else np.nan
    avg_person_dist = min_person_dist_values.mean() if len(min_person_dist_values) > 0 else np.nan

    # Angular smoothness (lower is better)
    w_cmd = df['w_cmd'].values
    angular_changes = np.abs(np.diff(w_cmd))
    avg_angular_smoothness = np.mean(angular_changes)

    # Angular jerk (derivative of angular acceleration)
    if len(angular_changes) > 1:
        angular_jerk = np.abs(np.diff(angular_changes))
        avg_angular_jerk = np.mean(angular_jerk)
    else:
        avg_angular_jerk = np.nan

    # Path length
    x = df['x'].values
    y = df['y'].values
    path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))

    # Path efficiency
    goal_distance = df['goal_dist'].iloc[0]
    straight_line_dist = goal_distance - df['goal_dist'].iloc[-1]
    path_efficiency = straight_line_dist / path_length if path_length > 0 else 0

    # Velocity near pedestrians
    near_ped_mask = (df_clean['min_person_dist'] < 3.0) & ~df_clean['min_person_dist'].isna()
    avg_vel_near_peds = df['v_cmd'][near_ped_mask].mean() if near_ped_mask.sum() > 0 else np.nan

    result = {
        'method': method_name,
        'time_to_goal': time_to_goal,
        'min_person_dist': min_person_dist,
        'avg_person_dist': avg_person_dist,
        'avg_angular_smoothness': avg_angular_smoothness,
        'avg_angular_jerk': avg_angular_jerk,
        'path_length': path_length,
        'path_efficiency': path_efficiency,
        'avg_vel_near_peds': avg_vel_near_peds,
    }

    # Add parameters if provided
    if params:
        result.update(params)

    return result


def load_sweep_results(sweep_dir: Path) -> pd.DataFrame:
    """Load all sweep results from a directory."""
    results = []

    # Load DWA sweep results
    dwa_sweep_file = sweep_dir / 'dwa_sweep_results.json'
    if dwa_sweep_file.exists():
        with open(dwa_sweep_file, 'r') as f:
            dwa_results = json.load(f)
            results.extend(dwa_results)

    # Load DWA sweep CSV files
    for csv_file in sweep_dir.glob('dwa_*.csv'):
        df = load_csv_data(str(csv_file))
        # Extract parameters from filename if possible
        params = {'inflation_radius': None, 'max_velocity': None}
        metrics = calculate_sweep_metrics(df, 'DWA', params)
        results.append(metrics)

    # Load MPC baseline
    mpc_file = sweep_dir / 'mpc_baseline.csv'
    if mpc_file.exists():
        df = load_csv_data(str(mpc_file))
        metrics = calculate_sweep_metrics(df, 'MPC (No VLM)')
        results.append(metrics)

    # Load VLM-MPC baseline
    vlm_file = sweep_dir / 'vlm_mpc_baseline.csv'
    if vlm_file.exists():
        df = load_csv_data(str(vlm_file))
        metrics = calculate_sweep_metrics(df, 'VLM-MPC')
        results.append(metrics)

    return pd.DataFrame(results)


def plot_pareto_frontier(df: pd.DataFrame, x_metric: str, y_metric: str,
                         x_label: str, y_label: str, title: str,
                         output_file: Path, better_direction: Tuple[str, str] = ('lower', 'higher')):
    """
    Plot Pareto frontier showing trade-offs.

    Args:
        df: DataFrame with metrics
        x_metric: Column name for x-axis
        y_metric: Column name for y-axis
        x_label: Label for x-axis
        y_label: Label for y-axis
        title: Plot title
        output_file: Output file path
        better_direction: Tuple of ('lower'|'higher', 'lower'|'higher') for (x, y)
    """
    fig, ax = plt.subplots(figsize=(10, 7))

    # Separate methods
    dwa_data = df[df['method'] == 'DWA']
    mpc_data = df[df['method'] == 'MPC (No VLM)']
    vlm_data = df[df['method'] == 'VLM-MPC']

    # Plot DWA sweep points
    if len(dwa_data) > 0:
        ax.scatter(dwa_data[x_metric], dwa_data[y_metric],
                  alpha=0.6, s=100, c='lightblue', edgecolors='blue',
                  linewidth=1.5, label='DWA (Parameter Sweep)', zorder=2)

    # Plot MPC baseline
    if len(mpc_data) > 0:
        ax.scatter(mpc_data[x_metric], mpc_data[y_metric],
                  marker='s', s=200, c='#ff9999', edgecolors='red',
                  linewidth=2, label='MPC (No VLM)', zorder=3)

    # Plot VLM-MPC baseline
    if len(vlm_data) > 0:
        ax.scatter(vlm_data[x_metric], vlm_data[y_metric],
                  marker='D', s=200, c='#66b3ff', edgecolors='darkblue',
                  linewidth=2, label='VLM-MPC', zorder=3)

    # Calculate and plot Pareto frontier for DWA
    if len(dwa_data) > 0:
        pareto_points = find_pareto_frontier(
            dwa_data[[x_metric, y_metric]].values,
            better_direction
        )
        if len(pareto_points) > 0:
            pareto_sorted = pareto_points[np.argsort(pareto_points[:, 0])]
            ax.plot(pareto_sorted[:, 0], pareto_sorted[:, 1],
                   'b--', alpha=0.5, linewidth=2, label='DWA Pareto Frontier')

    ax.set_xlabel(x_label, fontsize=12, fontweight='bold')
    ax.set_ylabel(y_label, fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_file}")


def find_pareto_frontier(points: np.ndarray, better_direction: Tuple[str, str]) -> np.ndarray:
    """
    Find Pareto frontier points.

    Args:
        points: Nx2 array of (x, y) points
        better_direction: Tuple of ('lower'|'higher', 'lower'|'higher') for (x, y)

    Returns:
        Array of Pareto optimal points
    """
    if len(points) == 0:
        return np.array([])

    # Adjust signs based on better direction
    adjusted_points = points.copy()
    if better_direction[0] == 'higher':
        adjusted_points[:, 0] = -adjusted_points[:, 0]
    if better_direction[1] == 'lower':
        adjusted_points[:, 1] = -adjusted_points[:, 1]

    # Find Pareto frontier (minimize x, maximize y in adjusted space)
    pareto_mask = np.ones(len(points), dtype=bool)
    for i, point in enumerate(adjusted_points):
        # Check if any other point dominates this point
        dominated = np.any(
            (adjusted_points[:, 0] < point[0]) & (adjusted_points[:, 1] >= point[1]) |
            (adjusted_points[:, 0] <= point[0]) & (adjusted_points[:, 1] > point[1])
        )
        if dominated:
            pareto_mask[i] = False

    return points[pareto_mask]


def create_summary_table(df: pd.DataFrame, output_file: Path):
    """Create summary table of all configurations."""
    # Calculate statistics
    summary = df.groupby('method').agg({
        'time_to_goal': ['mean', 'std', 'min', 'max'],
        'min_person_dist': ['mean', 'std', 'min', 'max'],
        'avg_angular_smoothness': ['mean', 'std', 'min', 'max'],
        'path_efficiency': ['mean', 'std', 'min', 'max'],
    }).round(3)

    # Save to CSV
    summary.to_csv(output_file)
    print(f"Saved summary table: {output_file}")

    return summary


def plot_combined_comparison(df: pd.DataFrame, output_dir: Path):
    """Create combined plot with both trade-offs."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # Separate methods
    dwa_data = df[df['method'] == 'DWA']
    mpc_data = df[df['method'] == 'MPC (No VLM)']
    vlm_data = df[df['method'] == 'VLM-MPC']

    # Plot 1: Safety vs Efficiency
    if len(dwa_data) > 0:
        ax1.scatter(dwa_data['time_to_goal'], dwa_data['min_person_dist'],
                   alpha=0.6, s=100, c='lightblue', edgecolors='blue',
                   linewidth=1.5, label='DWA (Sweep)', zorder=2)

    if len(mpc_data) > 0:
        ax1.scatter(mpc_data['time_to_goal'], mpc_data['min_person_dist'],
                   marker='s', s=200, c='#ff9999', edgecolors='red',
                   linewidth=2, label='MPC (No VLM)', zorder=3)

    if len(vlm_data) > 0:
        ax1.scatter(vlm_data['time_to_goal'], vlm_data['min_person_dist'],
                   marker='D', s=200, c='#66b3ff', edgecolors='darkblue',
                   linewidth=2, label='VLM-MPC', zorder=3)

    # Pareto frontier for plot 1
    if len(dwa_data) > 0:
        pareto_points = find_pareto_frontier(
            dwa_data[['time_to_goal', 'min_person_dist']].values,
            ('lower', 'higher')
        )
        if len(pareto_points) > 0:
            pareto_sorted = pareto_points[np.argsort(pareto_points[:, 0])]
            ax1.plot(pareto_sorted[:, 0], pareto_sorted[:, 1],
                    'b--', alpha=0.5, linewidth=2, label='DWA Pareto Frontier')

    ax1.set_xlabel('Time to Goal (s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Minimum Person Distance (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Safety vs Efficiency Trade-off', fontsize=13, fontweight='bold')
    ax1.legend(loc='best', fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Plot 2: Smoothness vs Efficiency
    if len(dwa_data) > 0:
        ax2.scatter(dwa_data['time_to_goal'], dwa_data['avg_angular_smoothness'],
                   alpha=0.6, s=100, c='lightblue', edgecolors='blue',
                   linewidth=1.5, label='DWA (Sweep)', zorder=2)

    if len(mpc_data) > 0:
        ax2.scatter(mpc_data['time_to_goal'], mpc_data['avg_angular_smoothness'],
                   marker='s', s=200, c='#ff9999', edgecolors='red',
                   linewidth=2, label='MPC (No VLM)', zorder=3)

    if len(vlm_data) > 0:
        ax2.scatter(vlm_data['time_to_goal'], vlm_data['avg_angular_smoothness'],
                   marker='D', s=200, c='#66b3ff', edgecolors='darkblue',
                   linewidth=2, label='VLM-MPC', zorder=3)

    # Pareto frontier for plot 2
    if len(dwa_data) > 0:
        pareto_points = find_pareto_frontier(
            dwa_data[['time_to_goal', 'avg_angular_smoothness']].values,
            ('lower', 'lower')
        )
        if len(pareto_points) > 0:
            pareto_sorted = pareto_points[np.argsort(pareto_points[:, 0])]
            ax2.plot(pareto_sorted[:, 0], pareto_sorted[:, 1],
                    'b--', alpha=0.5, linewidth=2, label='DWA Pareto Frontier')

    ax2.set_xlabel('Time to Goal (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Angular Smoothness (avg |Δω|)', fontsize=12, fontweight='bold')
    ax2.set_title('Smoothness vs Efficiency Trade-off', fontsize=13, fontweight='bold')
    ax2.legend(loc='best', fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    output_file = output_dir / 'pareto_combined_comparison.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved combined plot: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Pareto frontier analysis for navigation methods')
    parser.add_argument('--sweep_results', type=str, required=True,
                       help='Directory containing sweep results')
    parser.add_argument('--output', type=str, default='pareto_plots',
                       help='Output directory for plots')

    args = parser.parse_args()

    sweep_dir = Path(args.sweep_results)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Loading sweep results...")
    df = load_sweep_results(sweep_dir)

    if len(df) == 0:
        print("No results found! Please run parameter sweep first.")
        return

    print(f"Loaded {len(df)} configurations:")
    print(f"  DWA: {len(df[df['method'] == 'DWA'])}")
    print(f"  MPC: {len(df[df['method'] == 'MPC (No VLM)'])}")
    print(f"  VLM-MPC: {len(df[df['method'] == 'VLM-MPC'])}")

    # Create summary table
    print("\nCreating summary table...")
    create_summary_table(df, output_dir / 'pareto_summary.csv')

    # Plot 1: Safety vs Efficiency
    print("\nGenerating Safety vs Efficiency plot...")
    plot_pareto_frontier(
        df, 'time_to_goal', 'min_person_dist',
        'Time to Goal (s)', 'Minimum Person Distance (m)',
        'Safety vs Efficiency Trade-off',
        output_dir / 'pareto_safety_efficiency.png',
        better_direction=('lower', 'higher')
    )

    # Plot 2: Smoothness vs Efficiency
    print("\nGenerating Smoothness vs Efficiency plot...")
    plot_pareto_frontier(
        df, 'time_to_goal', 'avg_angular_smoothness',
        'Time to Goal (s)', 'Angular Smoothness (avg |Δω|)',
        'Smoothness vs Efficiency Trade-off',
        output_dir / 'pareto_smoothness_efficiency.png',
        better_direction=('lower', 'lower')
    )

    # Combined plot
    print("\nGenerating combined comparison plot...")
    plot_combined_comparison(df, output_dir)

    print("\n" + "="*80)
    print("Pareto analysis complete!")
    print(f"Results saved to: {output_dir.absolute()}")
    print("="*80)


if __name__ == '__main__':
    main()
