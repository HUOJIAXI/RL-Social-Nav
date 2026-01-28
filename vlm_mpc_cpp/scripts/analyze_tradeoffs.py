#!/usr/bin/env python3
"""
Trade-off Analysis for Existing Test Results

Analyzes DWA, MPC, and VLM-MPC comparison using existing test1-3 data.
Creates visualizations showing:
1. Safety vs Efficiency trade-off
2. Smoothness vs Efficiency trade-off
3. Social Compliance vs Efficiency trade-off

Usage:
    python analyze_tradeoffs.py \
        --test_dirs test1/ test2/ test3/ \
        --output tradeoff_plots/
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import List
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

# Set style
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (14, 6)
plt.rcParams['font.size'] = 11


def load_test_results(test_dirs: List[Path]) -> pd.DataFrame:
    """Load all test results from summary CSV files."""
    all_results = []

    for test_dir in test_dirs:
        summary_file = test_dir / 'three_way_summary.csv'

        if not summary_file.exists():
            print(f"Warning: {summary_file} not found, skipping")
            continue

        df = pd.read_csv(summary_file)
        df['test_name'] = test_dir.name
        all_results.append(df)

    if not all_results:
        raise FileNotFoundError("No test summary files found!")

    return pd.concat(all_results, ignore_index=True)


def create_tradeoff_plot(df: pd.DataFrame, output_dir: Path):
    """Create comprehensive trade-off visualization."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Define method colors and markers
    method_styles = {
        'DWA': {'color': '#4CAF50', 'marker': 'o', 'size': 150},
        'MPC (No VLM)': {'color': '#FF6B6B', 'marker': 's', 'size': 200},
        'VLM-MPC': {'color': '#4A90E2', 'marker': 'D', 'size': 200}
    }

    methods = ['DWA', 'MPC (No VLM)', 'VLM-MPC']

    # Compute mean and std for each method
    summary = []
    for method in methods:
        method_data = df[df['method'] == method]
        if len(method_data) == 0:
            continue

        summary.append({
            'method': method,
            'time_to_goal_mean': method_data['time_to_goal'].mean(),
            'time_to_goal_std': method_data['time_to_goal'].std(),
            'min_person_dist_mean': method_data['closest_person_dist'].mean(),
            'min_person_dist_std': method_data['closest_person_dist'].std(),
            'avg_person_dist_mean': method_data['avg_min_person_dist'].mean(),
            'avg_person_dist_std': method_data['avg_min_person_dist'].std(),
            'angular_smoothness_mean': method_data['avg_angular_smoothness'].mean(),
            'angular_smoothness_std': method_data['avg_angular_smoothness'].std(),
            'personal_zone_time_mean': method_data['personal_zone_time_pct'].mean(),
            'personal_zone_time_std': method_data['personal_zone_time_pct'].std(),
            'avg_vel_near_peds_mean': method_data['avg_velocity_near_pedestrians'].mean(),
            'avg_vel_near_peds_std': method_data['avg_velocity_near_pedestrians'].std(),
        })

    summary_df = pd.DataFrame(summary)

    # Plot 1: Safety vs Efficiency (Min Person Distance)
    ax1 = axes[0]
    for method in methods:
        if method not in summary_df['method'].values:
            continue
        row = summary_df[summary_df['method'] == method].iloc[0]
        style = method_styles[method]

        ax1.errorbar(
            row['time_to_goal_mean'],
            row['min_person_dist_mean'],
            xerr=row['time_to_goal_std'],
            yerr=row['min_person_dist_std'],
            fmt=style['marker'],
            color=style['color'],
            markersize=10,
            capsize=5,
            capthick=2,
            elinewidth=2,
            label=method,
            zorder=3
        )

    ax1.set_xlabel('Time to Goal (s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Closest Person Distance (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Safety vs Efficiency Trade-off', fontsize=13, fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Add arrows showing better directions
    ax1.annotate('', xy=(ax1.get_xlim()[0], ax1.get_ylim()[1]),
                xytext=(ax1.get_xlim()[0] + (ax1.get_xlim()[1] - ax1.get_xlim()[0]) * 0.15,
                       ax1.get_ylim()[1]),
                arrowprops=dict(arrowstyle='->', lw=2, color='gray', alpha=0.5))
    ax1.text(ax1.get_xlim()[0] + (ax1.get_xlim()[1] - ax1.get_xlim()[0]) * 0.08,
            ax1.get_ylim()[1] * 0.98, 'Faster', fontsize=9, color='gray', ha='center')

    ax1.annotate('', xy=(ax1.get_xlim()[0], ax1.get_ylim()[1]),
                xytext=(ax1.get_xlim()[0],
                       ax1.get_ylim()[0] + (ax1.get_ylim()[1] - ax1.get_ylim()[0]) * 0.85),
                arrowprops=dict(arrowstyle='->', lw=2, color='gray', alpha=0.5))
    ax1.text(ax1.get_xlim()[0] * 1.02, ax1.get_ylim()[0] + (ax1.get_ylim()[1] - ax1.get_ylim()[0]) * 0.92,
            'Safer', fontsize=9, color='gray', rotation=90, va='center')

    # Plot 2: Smoothness vs Efficiency
    ax2 = axes[1]
    for method in methods:
        if method not in summary_df['method'].values:
            continue
        row = summary_df[summary_df['method'] == method].iloc[0]
        style = method_styles[method]

        ax2.errorbar(
            row['time_to_goal_mean'],
            row['angular_smoothness_mean'],
            xerr=row['time_to_goal_std'],
            yerr=row['angular_smoothness_std'],
            fmt=style['marker'],
            color=style['color'],
            markersize=10,
            capsize=5,
            capthick=2,
            elinewidth=2,
            label=method,
            zorder=3
        )

    ax2.set_xlabel('Time to Goal (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Angular Smoothness (avg |Δω|)', fontsize=12, fontweight='bold')
    ax2.set_title('Smoothness vs Efficiency Trade-off', fontsize=13, fontweight='bold')
    ax2.legend(loc='best', fontsize=10)
    ax2.grid(True, alpha=0.3)

    # Add arrows
    ax2.annotate('', xy=(ax2.get_xlim()[0], ax2.get_ylim()[0]),
                xytext=(ax2.get_xlim()[0] + (ax2.get_xlim()[1] - ax2.get_xlim()[0]) * 0.15,
                       ax2.get_ylim()[0]),
                arrowprops=dict(arrowstyle='->', lw=2, color='gray', alpha=0.5))
    ax2.text(ax2.get_xlim()[0] + (ax2.get_xlim()[1] - ax2.get_xlim()[0]) * 0.08,
            ax2.get_ylim()[0] * 1.05, 'Faster', fontsize=9, color='gray', ha='center')

    ax2.annotate('', xy=(ax2.get_xlim()[0], ax2.get_ylim()[0]),
                xytext=(ax2.get_xlim()[0],
                       ax2.get_ylim()[0] + (ax2.get_ylim()[1] - ax2.get_ylim()[0]) * 0.15),
                arrowprops=dict(arrowstyle='->', lw=2, color='gray', alpha=0.5))
    ax2.text(ax2.get_xlim()[0] * 1.02, ax2.get_ylim()[0] + (ax2.get_ylim()[1] - ax2.get_ylim()[0]) * 0.08,
            'Smoother', fontsize=9, color='gray', rotation=90, va='center')

    # Plot 3: Social Compliance vs Efficiency (Time in Personal Zone)
    ax3 = axes[2]
    for method in methods:
        if method not in summary_df['method'].values:
            continue
        row = summary_df[summary_df['method'] == method].iloc[0]
        style = method_styles[method]

        ax3.errorbar(
            row['time_to_goal_mean'],
            row['personal_zone_time_mean'],
            xerr=row['time_to_goal_std'],
            yerr=row['personal_zone_time_std'],
            fmt=style['marker'],
            color=style['color'],
            markersize=10,
            capsize=5,
            capthick=2,
            elinewidth=2,
            label=method,
            zorder=3
        )

    ax3.set_xlabel('Time to Goal (s)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Time in Personal Zone (%)', fontsize=12, fontweight='bold')
    ax3.set_title('Social Compliance vs Efficiency', fontsize=13, fontweight='bold')
    ax3.legend(loc='best', fontsize=10)
    ax3.grid(True, alpha=0.3)

    # Add arrows
    ax3.annotate('', xy=(ax3.get_xlim()[0], ax3.get_ylim()[0]),
                xytext=(ax3.get_xlim()[0] + (ax3.get_xlim()[1] - ax3.get_xlim()[0]) * 0.15,
                       ax3.get_ylim()[0]),
                arrowprops=dict(arrowstyle='->', lw=2, color='gray', alpha=0.5))
    ax3.text(ax3.get_xlim()[0] + (ax3.get_xlim()[1] - ax3.get_xlim()[0]) * 0.08,
            ax3.get_ylim()[0] * 1.05, 'Faster', fontsize=9, color='gray', ha='center')

    ax3.annotate('', xy=(ax3.get_xlim()[0], ax3.get_ylim()[0]),
                xytext=(ax3.get_xlim()[0],
                       ax3.get_ylim()[0] + (ax3.get_ylim()[1] - ax3.get_ylim()[0]) * 0.15),
                arrowprops=dict(arrowstyle='->', lw=2, color='gray', alpha=0.5))
    ax3.text(ax3.get_xlim()[0] * 1.02, ax3.get_ylim()[0] + (ax3.get_ylim()[1] - ax3.get_ylim()[0]) * 0.08,
            'More Social', fontsize=9, color='gray', rotation=90, va='center')

    plt.tight_layout()
    output_file = output_dir / 'method_tradeoff_comparison.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"✓ Saved: {output_file}")

    return summary_df


def create_summary_report(summary_df: pd.DataFrame, output_dir: Path):
    """Create markdown summary report."""
    report_file = output_dir / 'TRADEOFF_ANALYSIS_REPORT.md'

    with open(report_file, 'w') as f:
        f.write("# Trade-off Analysis Report\n\n")
        f.write("Analysis of DWA, MPC (No VLM), and VLM-MPC across 3 test runs.\n\n")
        f.write("---\n\n")

        f.write("## Key Trade-offs (Mean ± Std Dev)\n\n")

        f.write("### 1. Safety vs Efficiency\n\n")
        f.write("| Method | Time to Goal (s) | Closest Person Dist (m) | Interpretation |\n")
        f.write("|--------|------------------|------------------------|----------------|\n")

        for _, row in summary_df.iterrows():
            f.write(f"| {row['method']} | "
                   f"{row['time_to_goal_mean']:.1f} ± {row['time_to_goal_std']:.1f} | "
                   f"{row['min_person_dist_mean']:.2f} ± {row['min_person_dist_std']:.2f} | ")

            # Interpretation
            if row['time_to_goal_mean'] < 205 and row['min_person_dist_mean'] > 0.7:
                f.write("Fast & Safe |\n")
            elif row['time_to_goal_mean'] < 205:
                f.write("Fast, moderate safety |\n")
            elif row['min_person_dist_mean'] > 0.7:
                f.write("Slower, very safe |\n")
            else:
                f.write("Balanced |\n")

        f.write("\n### 2. Smoothness vs Efficiency\n\n")
        f.write("| Method | Time to Goal (s) | Angular Smoothness | Interpretation |\n")
        f.write("|--------|------------------|--------------------|----------------|\n")

        for _, row in summary_df.iterrows():
            f.write(f"| {row['method']} | "
                   f"{row['time_to_goal_mean']:.1f} ± {row['time_to_goal_std']:.1f} | "
                   f"{row['angular_smoothness_mean']:.4f} ± {row['angular_smoothness_std']:.4f} | ")

            # Interpretation
            if row['angular_smoothness_mean'] < 0.03:
                f.write("Very smooth |\n")
            elif row['angular_smoothness_mean'] < 0.05:
                f.write("Smooth |\n")
            else:
                f.write("Less smooth |\n")

        f.write("\n### 3. Social Compliance vs Efficiency\n\n")
        f.write("| Method | Time to Goal (s) | Personal Zone Time (%) | Interpretation |\n")
        f.write("|--------|------------------|------------------------|----------------|\n")

        for _, row in summary_df.iterrows():
            f.write(f"| {row['method']} | "
                   f"{row['time_to_goal_mean']:.1f} ± {row['time_to_goal_std']:.1f} | "
                   f"{row['personal_zone_time_mean']:.1f} ± {row['personal_zone_time_std']:.1f} | ")

            # Interpretation
            if row['personal_zone_time_mean'] < 3.0:
                f.write("Excellent social compliance |\n")
            elif row['personal_zone_time_mean'] < 4.0:
                f.write("Good social compliance |\n")
            else:
                f.write("Moderate social compliance |\n")

        f.write("\n---\n\n")
        f.write("## Overall Assessment\n\n")

        # Find best in each dimension
        best_speed = summary_df.loc[summary_df['time_to_goal_mean'].idxmin(), 'method']
        best_safety = summary_df.loc[summary_df['min_person_dist_mean'].idxmax(), 'method']
        best_smooth = summary_df.loc[summary_df['angular_smoothness_mean'].idxmin(), 'method']
        best_social = summary_df.loc[summary_df['personal_zone_time_mean'].idxmin(), 'method']

        f.write(f"- **Fastest**: {best_speed}\n")
        f.write(f"- **Safest (closest approach)**: {best_safety}\n")
        f.write(f"- **Smoothest**: {best_smooth}\n")
        f.write(f"- **Most Socially Compliant**: {best_social}\n\n")

        f.write("### Key Findings:\n\n")
        f.write("1. **Efficiency**: Which method reaches goal fastest with acceptable safety?\n")
        f.write("2. **Safety**: Which method maintains largest distance from pedestrians?\n")
        f.write("3. **Comfort**: Which method provides smoothest motion?\n")
        f.write("4. **Social Awareness**: Which method best respects personal space?\n\n")

    print(f"✓ Saved: {report_file}")


def main():
    parser = argparse.ArgumentParser(description='Analyze trade-offs from existing test results')
    parser.add_argument('--test_dirs', nargs='+', required=True,
                       help='Test directories (e.g., test1/ test2/ test3/)')
    parser.add_argument('--output', type=str, default='tradeoff_analysis',
                       help='Output directory for plots and reports')

    args = parser.parse_args()

    # Convert paths
    test_dirs = [Path(d) for d in args.test_dirs]
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("="*60)
    print("TRADE-OFF ANALYSIS")
    print("="*60)
    print(f"Test directories: {[str(d) for d in test_dirs]}")
    print(f"Output directory: {output_dir.absolute()}")
    print()

    # Load data
    print("Loading test results...")
    df = load_test_results(test_dirs)
    print(f"  Loaded {len(df)} test results")
    print()

    # Create visualizations
    print("Creating trade-off visualization...")
    summary_df = create_tradeoff_plot(df, output_dir)
    print()

    # Create report
    print("Generating summary report...")
    create_summary_report(summary_df, output_dir)
    print()

    print("="*60)
    print("ANALYSIS COMPLETE")
    print("="*60)
    print(f"Results saved to: {output_dir.absolute()}")
    print()


if __name__ == '__main__':
    main()
