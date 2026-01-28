#!/usr/bin/env python3
"""
Extract evaluation metrics from MPC log files for VLM-MPC vs No-VLM comparison.

Usage:
    python extract_metrics.py --log-dir ~/ros2_logs/social_mpc_nav --output metrics.json
"""

import pandas as pd
import numpy as np
import json
import argparse
from pathlib import Path
from typing import Dict, List


class MPCMetricsExtractor:
    """Extract comprehensive metrics from MPC CSV logs."""

    def __init__(self, log_file: str):
        """Load MPC log file."""
        self.df = pd.read_csv(log_file)
        self.df['dt'] = self.df['timestamp'].diff().fillna(0)

    def extract_all_metrics(self) -> Dict:
        """Extract all metrics."""
        return {
            'navigation_efficiency': self.navigation_efficiency_metrics(),
            'safety': self.safety_metrics(),
            'social_compliance': self.social_compliance_metrics(),
            'motion_smoothness': self.motion_smoothness_metrics(),
            'computational': self.computational_metrics(),
        }

    def navigation_efficiency_metrics(self) -> Dict:
        """Navigation efficiency metrics."""
        # Time to goal
        time_to_goal = self.df['timestamp'].iloc[-1] - self.df['timestamp'].iloc[0]

        # Path length
        dx = self.df['x'].diff().fillna(0)
        dy = self.df['y'].diff().fillna(0)
        path_length = np.sqrt(dx**2 + dy**2).sum()

        # Euclidean distance (start to end)
        euclidean_dist = np.sqrt(
            (self.df['x'].iloc[-1] - self.df['x'].iloc[0])**2 +
            (self.df['y'].iloc[-1] - self.df['y'].iloc[0])**2
        )

        # Path efficiency
        path_efficiency = euclidean_dist / path_length if path_length > 0 else 0

        # Average velocity
        avg_velocity = self.df['v_cmd'].mean()

        # Number of stops (v < 0.05 m/s)
        num_stops = (self.df['v_cmd'] < 0.05).sum()

        # Goal reached (final distance < 0.5m)
        goal_reached = self.df['goal_dist'].iloc[-1] < 0.5

        return {
            'time_to_goal_sec': float(time_to_goal),
            'path_length_m': float(path_length),
            'euclidean_distance_m': float(euclidean_dist),
            'path_efficiency': float(path_efficiency),
            'avg_velocity_ms': float(avg_velocity),
            'num_stops': int(num_stops),
            'goal_reached': bool(goal_reached),
        }

    def safety_metrics(self) -> Dict:
        """Safety metrics."""
        # Minimum distances
        min_obs_dist = self.df['min_obs_dist'].min()
        min_person_dist = self.df['min_person_dist'].min()

        # Hard constraint violations (< 0.2m)
        hard_violations_obs = (self.df['min_obs_dist'] < 0.2).sum()
        hard_violations_person = (self.df['min_person_dist'] < 0.2).sum()

        # Soft constraint violations (< 0.3m for obstacles)
        soft_violations_obs = (self.df['min_obs_dist'] < 0.3).sum()

        # Average violation rate
        avg_violation_rate = self.df['violation_rate'].mean()

        # Time in danger zone (< 0.5m to obstacles or people)
        danger_time_ratio = ((self.df['min_obs_dist'] < 0.5) |
                            (self.df['min_person_dist'] < 0.5)).mean()

        return {
            'min_obstacle_distance_m': float(min_obs_dist),
            'min_person_distance_m': float(min_person_dist),
            'hard_violations_obstacles': int(hard_violations_obs),
            'hard_violations_people': int(hard_violations_person),
            'soft_violations_obstacles': int(soft_violations_obs),
            'avg_violation_rate': float(avg_violation_rate),
            'danger_time_ratio': float(danger_time_ratio),
        }

    def social_compliance_metrics(self) -> Dict:
        """Social compliance metrics."""
        # Filter timesteps when people are present (min_person_dist < 100)
        with_people = self.df[self.df['min_person_dist'] < 100]

        if len(with_people) == 0:
            return {
                'avg_personal_distance_m': None,
                'personal_space_violations': 0,
                'comfort_score': None,
            }

        # Average personal distance when people present
        avg_personal_dist = with_people['min_person_dist'].mean()

        # Personal space violations (< 0.5m)
        personal_violations = (with_people['min_person_dist'] < 0.5).sum()

        # Comfort score (1 - exp(-d/sigma))
        sigma = 1.0  # decay parameter
        comfort_scores = 1 - np.exp(-with_people['min_person_dist'] / sigma)
        avg_comfort = comfort_scores.mean()

        return {
            'avg_personal_distance_m': float(avg_personal_dist),
            'personal_space_violations': int(personal_violations),
            'comfort_score': float(avg_comfort),
        }

    def motion_smoothness_metrics(self) -> Dict:
        """Motion smoothness metrics."""
        # Velocity variance
        velocity_variance = self.df['v_cmd'].var()

        # Angular velocity variance
        angular_variance = self.df['w_cmd'].var()

        # Linear jerk (derivative of acceleration)
        v_diff = self.df['v_cmd'].diff().fillna(0)
        dt = self.df['dt'].replace(0, 0.1)  # Avoid division by zero
        v_acc = v_diff / dt
        v_jerk = v_acc.diff().fillna(0) / dt
        avg_jerk = v_jerk.abs().mean()

        # Angular jerk
        w_diff = self.df['w_cmd'].diff().fillna(0)
        w_acc = w_diff / dt
        w_jerk = w_acc.diff().fillna(0) / dt
        avg_angular_jerk = w_jerk.abs().mean()

        # Direction changes (yaw change > 30 degrees)
        yaw_diff = self.df['yaw'].diff().fillna(0)
        direction_changes = (yaw_diff.abs() > np.radians(30)).sum()

        return {
            'velocity_variance': float(velocity_variance),
            'angular_velocity_variance': float(angular_variance),
            'avg_linear_jerk': float(avg_jerk),
            'avg_angular_jerk': float(avg_angular_jerk),
            'num_direction_changes': int(direction_changes),
        }

    def computational_metrics(self) -> Dict:
        """Computational performance metrics."""
        # Average solve time
        avg_solve_time = self.df['solve_time_ms'].mean()
        max_solve_time = self.df['solve_time_ms'].max()

        # Control loop frequency
        avg_dt = self.df['dt'].replace(0, np.nan).mean()
        control_freq = 1.0 / avg_dt if avg_dt > 0 else 0

        return {
            'avg_solve_time_ms': float(avg_solve_time),
            'max_solve_time_ms': float(max_solve_time),
            'control_frequency_hz': float(control_freq),
        }


def extract_trial_metrics(log_dir: Path, scenario: str, condition: str, trial: int) -> Dict:
    """Extract metrics for a single trial."""
    log_file = log_dir / f"{scenario}_{condition}_trial{trial}_mpc_casadi_log.csv"

    if not log_file.exists():
        print(f"Warning: Log file not found: {log_file}")
        return None

    extractor = MPCMetricsExtractor(str(log_file))
    metrics = extractor.extract_all_metrics()

    # Add metadata
    metrics['metadata'] = {
        'scenario': scenario,
        'condition': condition,
        'trial': trial,
        'log_file': str(log_file),
    }

    return metrics


def aggregate_metrics(all_metrics: List[Dict]) -> Dict:
    """Aggregate metrics across trials."""
    # Flatten nested metrics
    flat_metrics = []
    for trial_metrics in all_metrics:
        if trial_metrics is None:
            continue
        flat = {}
        for category, metrics in trial_metrics.items():
            if category == 'metadata':
                continue
            for metric, value in metrics.items():
                if value is not None:
                    flat[f"{category}_{metric}"] = value
        flat_metrics.append(flat)

    if not flat_metrics:
        return {}

    df = pd.DataFrame(flat_metrics)

    # Compute statistics
    stats = {
        'mean': df.mean().to_dict(),
        'std': df.std().to_dict(),
        'min': df.min().to_dict(),
        'max': df.max().to_dict(),
        'median': df.median().to_dict(),
    }

    return stats


def main():
    parser = argparse.ArgumentParser(description='Extract MPC evaluation metrics')
    parser.add_argument('--log-dir', type=str, required=True, help='Directory containing log files')
    parser.add_argument('--output', type=str, default='metrics.json', help='Output JSON file')
    parser.add_argument('--scenarios', nargs='+', default=['S1', 'S2', 'S3'], help='Scenario IDs')
    parser.add_argument('--conditions', nargs='+', default=['no_vlm', 'vlm'], help='Conditions')
    parser.add_argument('--trials', type=int, default=10, help='Number of trials per scenario')

    args = parser.parse_args()
    log_dir = Path(args.log_dir).expanduser()

    # Extract metrics for all trials
    all_results = {}
    for scenario in args.scenarios:
        for condition in args.conditions:
            key = f"{scenario}_{condition}"
            trial_metrics = []

            for trial in range(args.trials):
                metrics = extract_trial_metrics(log_dir, scenario, condition, trial)
                if metrics:
                    trial_metrics.append(metrics)

            if trial_metrics:
                all_results[key] = {
                    'trials': trial_metrics,
                    'aggregate': aggregate_metrics(trial_metrics),
                }

    # Save results
    with open(args.output, 'w') as f:
        json.dump(all_results, f, indent=2)

    print(f"Metrics extracted and saved to {args.output}")
    print(f"Total configurations: {len(all_results)}")


if __name__ == '__main__':
    main()
