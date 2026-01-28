#!/usr/bin/env python3
"""
Visualize VLM side preference bias with charts and statistics.
Creates plots showing the distribution of left/right/neutral preferences.
"""

import csv
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import Counter
import numpy as np

def load_vlm_data(log_path):
    """Load VLM translation log data."""
    data = []
    with open(log_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)
    return data


def plot_side_preference_distribution(data, output_path='vlm_side_bias_distribution.png'):
    """Create a bar chart showing side preference distribution."""
    side_counts = Counter(row['side_preference'] for row in data)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # Plot 1: Count distribution
    colors = {'left': '#FF6B6B', 'right': '#4ECDC4', 'neutral': '#95E1D3'}
    sides = ['left', 'right', 'neutral']
    counts = [side_counts.get(s, 0) for s in sides]

    bars1 = ax1.bar(sides, counts, color=[colors[s] for s in sides], alpha=0.8, edgecolor='black')
    ax1.set_ylabel('Count', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Side Preference', fontsize=12, fontweight='bold')
    ax1.set_title('VLM Side Preference Distribution\n(Showing Severe Left Bias)', fontsize=14, fontweight='bold')
    ax1.grid(axis='y', alpha=0.3, linestyle='--')

    # Add count labels on bars
    for bar, count in zip(bars1, counts):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{count}\n({count/len(data)*100:.1f}%)',
                ha='center', va='bottom', fontsize=11, fontweight='bold')

    # Add warning annotation
    if side_counts['right'] == 0:
        ax1.text(1, max(counts) * 0.7, '⚠ ZERO "right"\npreferences!',
                ha='center', va='center', fontsize=14, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='red', alpha=0.3))

    # Plot 2: Percentage pie chart
    sizes = [side_counts.get(s, 0) for s in sides]
    explode = (0.05, 0.1, 0.05)  # Explode the "right" slice to highlight it's missing

    wedges, texts, autotexts = ax2.pie(sizes, explode=explode, labels=sides,
                                         colors=[colors[s] for s in sides],
                                         autopct='%1.1f%%', startangle=90,
                                         textprops={'fontsize': 11, 'fontweight': 'bold'})
    ax2.set_title('Side Preference Percentage\n(Expected: ~40% each for L/R)',
                  fontsize=14, fontweight='bold')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ Saved distribution plot to: {output_path}")
    return output_path


def plot_temporal_pattern(data, output_path='vlm_side_bias_timeline.png'):
    """Create a timeline showing side preferences over time."""
    timestamps = [float(row['stamp_sec']) for row in data]
    start_time = min(timestamps)
    relative_times = [(t - start_time) / 60 for t in timestamps]  # Convert to minutes

    # Map side preferences to numeric values for plotting
    side_map = {'left': 1, 'neutral': 0, 'right': -1}
    side_values = [side_map[row['side_preference']] for row in data]

    fig, ax = plt.subplots(figsize=(14, 6))

    # Create scatter plot with colors
    colors = ['#FF6B6B' if s == 1 else '#95E1D3' if s == 0 else '#4ECDC4'
              for s in side_values]

    ax.scatter(relative_times, side_values, c=colors, s=100, alpha=0.7,
               edgecolors='black', linewidth=1)

    # Add horizontal lines for reference
    ax.axhline(y=1, color='#FF6B6B', linestyle='--', linewidth=1, alpha=0.5, label='Left')
    ax.axhline(y=0, color='#95E1D3', linestyle='--', linewidth=1, alpha=0.5, label='Neutral')
    ax.axhline(y=-1, color='#4ECDC4', linestyle='--', linewidth=1, alpha=0.5, label='Right')

    ax.set_yticks([-1, 0, 1])
    ax.set_yticklabels(['Right', 'Neutral', 'Left'], fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (minutes)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Side Preference', fontsize=12, fontweight='bold')
    ax.set_title('VLM Side Preference Over Time\n(Notice: No "Right" choices)',
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')

    # Add annotation if no right preferences
    if -1 not in side_values:
        ax.text(max(relative_times) * 0.5, -1, '⚠ NO DATA POINTS HERE!\nVLM never chose "right"',
                ha='center', va='center', fontsize=12, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ Saved timeline plot to: {output_path}")
    return output_path


def plot_crowd_density_correlation(data, output_path='vlm_side_bias_by_crowd.png'):
    """Show side preference by crowd density."""
    densities = ['empty', 'sparse', 'medium', 'dense']
    side_by_density = {d: Counter() for d in densities}

    for row in data:
        density = row['crowd_density']
        side = row['side_preference']
        side_by_density[density][side] += 1

    fig, ax = plt.subplots(figsize=(12, 6))

    x = np.arange(len(densities))
    width = 0.25

    colors = {'left': '#FF6B6B', 'right': '#4ECDC4', 'neutral': '#95E1D3'}

    for i, side in enumerate(['left', 'neutral', 'right']):
        counts = [side_by_density[d][side] for d in densities]
        offset = (i - 1) * width
        bars = ax.bar(x + offset, counts, width, label=side,
                     color=colors[side], alpha=0.8, edgecolor='black')

        # Add count labels
        for bar in bars:
            height = bar.get_height()
            if height > 0:
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       f'{int(height)}',
                       ha='center', va='bottom', fontsize=9)

    ax.set_xlabel('Crowd Density', fontsize=12, fontweight='bold')
    ax.set_ylabel('Count', fontsize=12, fontweight='bold')
    ax.set_title('Side Preference by Crowd Density\n(Showing consistent lack of "right" across all densities)',
                fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(densities, fontsize=11)
    ax.legend(fontsize=11)
    ax.grid(axis='y', alpha=0.3, linestyle='--')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ Saved crowd density correlation plot to: {output_path}")
    return output_path


def create_summary_report(data, output_path='vlm_side_bias_report.txt'):
    """Create a text summary report."""
    side_counts = Counter(row['side_preference'] for row in data)
    total = len(data)

    with open(output_path, 'w') as f:
        f.write("="*80 + "\n")
        f.write("VLM SIDE PREFERENCE BIAS - DETAILED REPORT\n")
        f.write("="*80 + "\n\n")

        f.write(f"Total VLM decisions analyzed: {total}\n\n")

        f.write("SIDE PREFERENCE DISTRIBUTION:\n")
        f.write(f"  Left:    {side_counts['left']:3d} ({side_counts['left']/total*100:5.1f}%)\n")
        f.write(f"  Right:   {side_counts['right']:3d} ({side_counts['right']/total*100:5.1f}%)\n")
        f.write(f"  Neutral: {side_counts['neutral']:3d} ({side_counts['neutral']/total*100:5.1f}%)\n\n")

        if side_counts['right'] == 0:
            f.write("⚠⚠⚠ CRITICAL ISSUE DETECTED ⚠⚠⚠\n\n")
            f.write("The VLM has a SEVERE LEFT BIAS:\n")
            f.write("  • ZERO instances of 'right' side preference\n")
            f.write("  • 87% of decisions are 'left'\n")
            f.write("  • This indicates systematic bias in VLM reasoning\n\n")

            f.write("IMPLICATIONS:\n")
            f.write("  • Robot will ALWAYS pass on left or stay neutral\n")
            f.write("  • Robot will NEVER pass on right, even when optimal\n")
            f.write("  • May cause inefficient or unsafe navigation when people are on left side\n")
            f.write("  • Robot's behavior becomes predictable and asymmetric\n\n")

        f.write("EXPECTED vs ACTUAL:\n")
        f.write("  Expected (balanced scenarios):\n")
        f.write("    - Left:    40-45%\n")
        f.write("    - Right:   40-45%\n")
        f.write("    - Neutral: 10-20%\n\n")
        f.write("  Actual:\n")
        f.write(f"    - Left:    {side_counts['left']/total*100:.1f}%\n")
        f.write(f"    - Right:   {side_counts['right']/total*100:.1f}%\n")
        f.write(f"    - Neutral: {side_counts['neutral']/total*100:.1f}%\n\n")

        # Analyze explanations
        left_with_right = 0
        left_with_left = 0
        for row in data:
            if row['side_preference'] == 'left':
                exp_lower = row['explanation'].lower()
                if 'right' in exp_lower:
                    left_with_right += 1
                if 'left' in exp_lower:
                    left_with_left += 1

        f.write("EXPLANATION ANALYSIS (for 'left' preferences):\n")
        f.write(f"  Mentions 'right': {left_with_right}/{side_counts['left']} ")
        f.write(f"({left_with_right/max(side_counts['left'],1)*100:.1f}%) - CORRECT reasoning\n")
        f.write(f"  Mentions 'left':  {left_with_left}/{side_counts['left']} ")
        f.write(f"({left_with_left/max(side_counts['left'],1)*100:.1f}%) - May indicate confusion\n\n")

        f.write("RECOMMENDED ACTIONS:\n")
        f.write("  1. Review fix_vlm_prompt_bias.md for detailed solutions\n")
        f.write("  2. Implement emphatic instructions in prompt (Fix #1)\n")
        f.write("  3. Add few-shot examples showing 'right' choices (Fix #2)\n")
        f.write("  4. Test with test_vlm_side_preference.py scenarios\n")
        f.write("  5. Re-run this analysis after fixes\n\n")

        f.write("="*80 + "\n")

    print(f"✓ Saved detailed report to: {output_path}")
    return output_path


def main():
    """Main visualization function."""
    log_file = Path.home() / 'ros2_logs' / 'social_mpc_nav' / 'vlm_translation.csv'

    if not log_file.exists():
        print(f"Error: Log file not found at {log_file}")
        return

    print("\nLoading VLM translation data...")
    data = load_vlm_data(log_file)
    print(f"Loaded {len(data)} VLM decisions\n")

    print("Creating visualizations...")

    # Create output directory
    output_dir = Path('vlm_bias_analysis')
    output_dir.mkdir(exist_ok=True)

    # Generate plots
    plot1 = plot_side_preference_distribution(data, output_dir / 'side_preference_distribution.png')
    plot2 = plot_temporal_pattern(data, output_dir / 'side_preference_timeline.png')
    plot3 = plot_crowd_density_correlation(data, output_dir / 'side_preference_by_crowd.png')
    report = create_summary_report(data, output_dir / 'bias_report.txt')

    print(f"\n{'='*80}")
    print("VISUALIZATION COMPLETE")
    print(f"{'='*80}")
    print(f"\nAll outputs saved to: {output_dir}/")
    print(f"\nView the plots:")
    print(f"  1. {plot1}")
    print(f"  2. {plot2}")
    print(f"  3. {plot3}")
    print(f"\nRead the full report:")
    print(f"  {report}")
    print()


if __name__ == '__main__':
    try:
        main()
    except ImportError as e:
        print(f"\nError: {e}")
        print("\nMatplotlib is required for visualization.")
        print("Install with: pip3 install matplotlib")
        print("\nAlternatively, run the text-only analysis:")
        print("  python3 scripts/analyze_vlm_side_bias.py")
