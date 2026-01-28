#!/usr/bin/env python3
"""
Generate summary report comparing VLM vs No-VLM performance.

Usage:
    python generate_summary_report.py --metrics metrics.json --output report.md
"""

import json
import argparse
from pathlib import Path
from typing import Dict, List
import numpy as np
from scipy import stats


def load_metrics(metrics_file: str) -> dict:
    """Load metrics from JSON file."""
    with open(metrics_file, 'r') as f:
        return json.load(f)


def compute_statistics(values: List[float]) -> Dict:
    """Compute mean, std, min, max."""
    if not values:
        return {'mean': None, 'std': None, 'min': None, 'max': None}
    return {
        'mean': float(np.mean(values)),
        'std': float(np.std(values)),
        'min': float(np.min(values)),
        'max': float(np.max(values))
    }


def extract_metric_values(data: dict, scenario: str, condition: str, metric_path: str) -> List[float]:
    """Extract metric values from nested structure."""
    key = f"{scenario}_{condition}"
    if key not in data or 'trials' not in data[key]:
        return []

    values = []
    for trial in data[key]['trials']:
        # Navigate nested structure (e.g., "safety.min_person_distance_m")
        category, metric = metric_path.split('.')
        if category in trial and metric in trial[category]:
            val = trial[category][metric]
            if val is not None:
                values.append(val)
    return values


def compare_conditions(no_vlm_values: List[float], vlm_values: List[float]) -> Dict:
    """Statistical comparison between conditions."""
    if not no_vlm_values or not vlm_values:
        return {'t_stat': None, 'p_value': None, 'significant': False, 'improvement': None}

    t_stat, p_value = stats.ttest_ind(no_vlm_values, vlm_values)
    significant = p_value < 0.05

    mean_no_vlm = np.mean(no_vlm_values)
    mean_vlm = np.mean(vlm_values)
    improvement_pct = ((mean_vlm - mean_no_vlm) / mean_no_vlm * 100) if mean_no_vlm != 0 else 0

    return {
        't_stat': float(t_stat),
        'p_value': float(p_value),
        'significant': significant,
        'improvement_pct': float(improvement_pct)
    }


def generate_markdown_report(data: dict, output_file: str):
    """Generate comprehensive markdown report."""
    scenarios = sorted(set(key.split('_')[0] for key in data.keys() if '_' in key))

    with open(output_file, 'w') as f:
        f.write("# VLM-MPC vs No-VLM Comparison Report\n\n")
        f.write(f"Generated: {Path(output_file).parent.name}\n\n")
        f.write("---\n\n")

        # Key metrics to analyze
        key_metrics = [
            ('safety.min_person_distance_m', 'Min Person Distance (m)', 'higher_better'),
            ('social_compliance.comfort_score', 'Comfort Score', 'higher_better'),
            ('social_compliance.personal_space_violations', 'Personal Space Violations', 'lower_better'),
            ('navigation_efficiency.time_to_goal_sec', 'Time to Goal (s)', 'lower_better'),
            ('navigation_efficiency.path_efficiency', 'Path Efficiency', 'higher_better'),
            ('motion_smoothness.avg_jerk', 'Average Jerk', 'lower_better'),
            ('computational.avg_solve_time_ms', 'Avg Solve Time (ms)', 'lower_better'),
        ]

        # Summary table
        f.write("## Executive Summary\n\n")
        f.write("| Metric | No-VLM Mean | VLM Mean | Improvement | Significant? |\n")
        f.write("|--------|-------------|----------|-------------|-------------|\n")

        overall_improvements = []

        for metric_path, metric_name, direction in key_metrics:
            # Aggregate across all scenarios
            all_no_vlm = []
            all_vlm = []

            for scenario in scenarios:
                all_no_vlm.extend(extract_metric_values(data, scenario, 'no_vlm', metric_path))
                all_vlm.extend(extract_metric_values(data, scenario, 'vlm', metric_path))

            if all_no_vlm and all_vlm:
                stats_no_vlm = compute_statistics(all_no_vlm)
                stats_vlm = compute_statistics(all_vlm)
                comparison = compare_conditions(all_no_vlm, all_vlm)

                improvement_str = f"{comparison['improvement_pct']:+.1f}%"
                sig_str = "✅ Yes" if comparison['significant'] else "❌ No"

                # Track if improvement is in desired direction
                if direction == 'higher_better' and comparison['improvement_pct'] > 0 and comparison['significant']:
                    overall_improvements.append(True)
                elif direction == 'lower_better' and comparison['improvement_pct'] < 0 and comparison['significant']:
                    overall_improvements.append(True)
                else:
                    overall_improvements.append(False)

                f.write(f"| {metric_name} | {stats_no_vlm['mean']:.3f} | {stats_vlm['mean']:.3f} | "
                       f"{improvement_str} | {sig_str} (p={comparison['p_value']:.3f}) |\n")

        f.write("\n")

        # Overall verdict
        success_rate = sum(overall_improvements) / len(overall_improvements) * 100 if overall_improvements else 0
        f.write(f"### Overall Verdict: {success_rate:.0f}% of key metrics improved\n\n")

        if success_rate >= 70:
            f.write("✅ **VLM integration shows strong positive impact**\n\n")
        elif success_rate >= 50:
            f.write("⚠️ **VLM integration shows moderate positive impact**\n\n")
        else:
            f.write("❌ **VLM integration needs improvement**\n\n")

        # Per-scenario breakdown
        f.write("---\n\n")
        f.write("## Per-Scenario Analysis\n\n")

        for scenario in scenarios:
            f.write(f"### {scenario}\n\n")
            f.write("| Metric | No-VLM | VLM | Change | p-value |\n")
            f.write("|--------|--------|-----|--------|--------|\n")

            for metric_path, metric_name, direction in key_metrics:
                no_vlm_vals = extract_metric_values(data, scenario, 'no_vlm', metric_path)
                vlm_vals = extract_metric_values(data, scenario, 'vlm', metric_path)

                if no_vlm_vals and vlm_vals:
                    stats_no_vlm = compute_statistics(no_vlm_vals)
                    stats_vlm = compute_statistics(vlm_vals)
                    comparison = compare_conditions(no_vlm_vals, vlm_vals)

                    f.write(f"| {metric_name} | "
                           f"{stats_no_vlm['mean']:.3f}±{stats_no_vlm['std']:.3f} | "
                           f"{stats_vlm['mean']:.3f}±{stats_vlm['std']:.3f} | "
                           f"{comparison['improvement_pct']:+.1f}% | "
                           f"{comparison['p_value']:.3f} |\n")

            f.write("\n")

        # Recommendations
        f.write("---\n\n")
        f.write("## Recommendations\n\n")
        f.write("Based on the results:\n\n")

        # Check specific metrics
        all_comfort = []
        all_time = []
        for scenario in scenarios:
            all_comfort.extend(extract_metric_values(data, scenario, 'vlm', 'social_compliance.comfort_score'))
            all_time.extend(extract_metric_values(data, scenario, 'vlm', 'navigation_efficiency.time_to_goal_sec'))

        if all_comfort and np.mean(all_comfort) > 0.7:
            f.write("- ✅ Social comfort is good (mean > 0.7)\n")
        else:
            f.write("- ⚠️ Consider increasing `w_vlm_personal` weight to improve social comfort\n")

        if all_time:
            time_comparison = compare_conditions(
                [v for s in scenarios for v in extract_metric_values(data, s, 'no_vlm', 'navigation_efficiency.time_to_goal_sec')],
                all_time
            )
            if time_comparison['improvement_pct'] > 20:
                f.write("- ⚠️ VLM significantly increases navigation time (+{:.1f}%). Consider:\n".format(time_comparison['improvement_pct']))
                f.write("  - Reducing `w_vlm_action` weight\n")
                f.write("  - Adjusting `speed_scale` threshold in VLM translator\n")
            else:
                f.write("- ✅ Navigation efficiency is preserved\n")

        f.write("\n---\n\n")
        f.write("*End of Report*\n")

    print(f"Report generated: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Generate VLM comparison summary report')
    parser.add_argument('--metrics', required=True, help='Path to metrics.json file')
    parser.add_argument('--output', required=True, help='Output markdown file path')
    args = parser.parse_args()

    data = load_metrics(args.metrics)
    generate_markdown_report(data, args.output)


if __name__ == '__main__':
    main()
