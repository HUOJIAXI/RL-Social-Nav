#!/usr/bin/env python3
"""
Parameter Sweep for DWA vs MPC vs VLM-MPC Comparison

This script runs a parameter sweep experiment:
1. DWA: Grid search over inflation radius and max velocity
2. MPC (No VLM): Single baseline run with fixed parameters
3. VLM-MPC: Single baseline run with fixed parameters

All methods use the same recorded pedestrian trajectories for fair comparison.

Usage:
    python run_parameter_sweep.py --output_dir sweep_results/ --scenario_file pedestrian_scenario.yaml
"""

import argparse
import subprocess
import json
import time
import shutil
from pathlib import Path
from typing import List, Dict, Tuple
import yaml
import numpy as np


class ParameterSweepRunner:
    def __init__(self, output_dir: Path, scenario_file: str = None):
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.scenario_file = scenario_file

        # DWA parameter grid
        self.inflation_radii = [0.3, 0.4, 0.5, 0.6, 0.7]  # meters
        self.max_velocities = [0.4, 0.5, 0.6, 0.7]  # m/s

        # MPC baseline parameters (fixed)
        self.mpc_params = {
            'use_vlm': False,
            'prediction_horizon': 20,
            'control_horizon': 10,
        }

        # VLM-MPC baseline parameters (fixed)
        self.vlm_mpc_params = {
            'use_vlm': True,
            'prediction_horizon': 20,
            'control_horizon': 10,
        }

        # Results storage
        self.sweep_results = []

    def generate_dwa_config(self, inflation_radius: float, max_velocity: float) -> Dict:
        """Generate DWA configuration for given parameters."""
        return {
            'method': 'DWA',
            'inflation_radius': inflation_radius,
            'max_velocity': max_velocity,
            'min_velocity': 0.0,
            'max_angular_velocity': 1.0,
            'velocity_resolution': 0.01,
            'angular_velocity_resolution': 0.1,
            'sim_time': 2.0,
            'sim_granularity': 0.05,
        }

    def run_dwa_configuration(self, inflation_radius: float, max_velocity: float,
                             run_index: int) -> Tuple[str, Dict]:
        """
        Run DWA with specific parameters.

        Returns:
            Tuple of (csv_file_path, parameters_dict)
        """
        print(f"\nRunning DWA - Inflation: {inflation_radius}m, Max Vel: {max_velocity}m/s")

        config = self.generate_dwa_config(inflation_radius, max_velocity)

        # Output file
        output_file = self.output_dir / f"dwa_r{inflation_radius:.1f}_v{max_velocity:.1f}.csv"

        # Command to run DWA with these parameters
        # This assumes you have a ROS2 launch file or script that accepts these parameters
        cmd = [
            'ros2', 'launch', 'social_mpc_nav', 'dwa_navigation.launch.py',
            f'inflation_radius:={inflation_radius}',
            f'max_velocity:={max_velocity}',
            f'output_file:={output_file}',
        ]

        # Add scenario file if provided
        if self.scenario_file:
            cmd.append(f'scenario_file:={self.scenario_file}')

        try:
            print(f"  Command: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)

            if result.returncode != 0:
                print(f"  WARNING: DWA run failed with code {result.returncode}")
                print(f"  stderr: {result.stderr}")
                return None, config

            # Verify output file exists
            if not output_file.exists():
                print(f"  WARNING: Output file not created: {output_file}")
                return None, config

            print(f"  ✓ Success: {output_file}")
            return str(output_file), config

        except subprocess.TimeoutExpired:
            print(f"  WARNING: DWA run timed out")
            return None, config
        except Exception as e:
            print(f"  ERROR: {e}")
            return None, config

    def run_mpc_baseline(self, use_vlm: bool) -> str:
        """
        Run MPC baseline (with or without VLM).

        Returns:
            Path to output CSV file
        """
        method_name = "VLM-MPC" if use_vlm else "MPC (No VLM)"
        print(f"\n{'='*60}")
        print(f"Running {method_name} Baseline")
        print(f"{'='*60}")

        output_file = self.output_dir / ('vlm_mpc_baseline.csv' if use_vlm else 'mpc_baseline.csv')

        # Command to run MPC
        cmd = [
            'ros2', 'launch', 'social_mpc_nav', 'mpc_navigation.launch.py',
            f'use_vlm:={str(use_vlm).lower()}',
            f'output_file:={output_file}',
        ]

        # Add scenario file if provided
        if self.scenario_file:
            cmd.append(f'scenario_file:={self.scenario_file}')

        try:
            print(f"Command: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)

            if result.returncode != 0:
                print(f"WARNING: {method_name} failed with code {result.returncode}")
                print(f"stderr: {result.stderr}")
                return None

            if not output_file.exists():
                print(f"WARNING: Output file not created: {output_file}")
                return None

            print(f"✓ Success: {output_file}")
            return str(output_file)

        except subprocess.TimeoutExpired:
            print(f"WARNING: {method_name} timed out")
            return None
        except Exception as e:
            print(f"ERROR: {e}")
            return None

    def run_sweep(self, skip_baselines: bool = False):
        """Run the complete parameter sweep."""
        print("="*80)
        print("PARAMETER SWEEP EXPERIMENT")
        print("="*80)
        print(f"Output directory: {self.output_dir.absolute()}")
        print(f"Scenario file: {self.scenario_file or 'Default'}")
        print(f"\nDWA Parameter Grid:")
        print(f"  Inflation Radii: {self.inflation_radii}")
        print(f"  Max Velocities: {self.max_velocities}")
        print(f"  Total DWA configurations: {len(self.inflation_radii) * len(self.max_velocities)}")
        print("="*80)

        # Run DWA parameter sweep
        print("\n" + "="*80)
        print("PHASE 1: DWA PARAMETER SWEEP")
        print("="*80)

        dwa_results = []
        run_index = 0

        for inflation_radius in self.inflation_radii:
            for max_velocity in self.max_velocities:
                run_index += 1
                print(f"\n[{run_index}/{len(self.inflation_radii) * len(self.max_velocities)}]", end=" ")

                csv_file, config = self.run_dwa_configuration(
                    inflation_radius, max_velocity, run_index
                )

                if csv_file:
                    dwa_results.append({
                        'csv_file': csv_file,
                        'inflation_radius': inflation_radius,
                        'max_velocity': max_velocity,
                    })

                # Brief pause between runs
                time.sleep(1)

        # Save DWA results
        dwa_results_file = self.output_dir / 'dwa_sweep_results.json'
        with open(dwa_results_file, 'w') as f:
            json.dump(dwa_results, f, indent=2)
        print(f"\n✓ DWA sweep results saved to: {dwa_results_file}")

        # Run baselines if not skipped
        if not skip_baselines:
            # Run MPC baseline
            mpc_file = self.run_mpc_baseline(use_vlm=False)

            # Run VLM-MPC baseline
            vlm_mpc_file = self.run_mpc_baseline(use_vlm=True)

        # Final summary
        print("\n" + "="*80)
        print("SWEEP COMPLETE")
        print("="*80)
        print(f"DWA configurations completed: {len(dwa_results)}/{len(self.inflation_radii) * len(self.max_velocities)}")
        print(f"Results directory: {self.output_dir.absolute()}")
        print("\nNext steps:")
        print(f"  1. Run Pareto analysis:")
        print(f"     python pareto_analysis.py --sweep_results {self.output_dir} --output pareto_plots/")
        print("="*80)


def create_example_scenario_file(output_path: Path):
    """Create an example scenario configuration file."""
    scenario = {
        'name': 'parameter_sweep_scenario',
        'goal': {
            'x': 10.0,
            'y': 0.0,
            'theta': 0.0,
        },
        'pedestrians': [
            {
                'id': 0,
                'trajectory_type': 'waypoint',
                'waypoints': [
                    {'x': 5.0, 'y': -2.0, 't': 0.0},
                    {'x': 5.0, 'y': 2.0, 't': 10.0},
                    {'x': 5.0, 'y': -2.0, 't': 20.0},
                ]
            },
            {
                'id': 1,
                'trajectory_type': 'waypoint',
                'waypoints': [
                    {'x': 7.0, 'y': 2.0, 't': 0.0},
                    {'x': 7.0, 'y': -2.0, 't': 15.0},
                    {'x': 7.0, 'y': 2.0, 't': 30.0},
                ]
            },
        ],
        'static_obstacles': [
            {'x': 3.0, 'y': 1.5, 'radius': 0.3},
            {'x': 8.0, 'y': -1.5, 'radius': 0.3},
        ]
    }

    with open(output_path, 'w') as f:
        yaml.dump(scenario, f, default_flow_style=False)

    print(f"Example scenario file created: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Run parameter sweep for DWA vs MPC comparison',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run full sweep with custom scenario
  python run_parameter_sweep.py --output_dir sweep_results/ --scenario my_scenario.yaml

  # Run only DWA sweep (skip baselines)
  python run_parameter_sweep.py --output_dir sweep_results/ --skip_baselines

  # Create example scenario file
  python run_parameter_sweep.py --create_scenario example_scenario.yaml --dry_run
        """
    )

    parser.add_argument('--output_dir', type=str, default='sweep_results',
                       help='Output directory for results')
    parser.add_argument('--scenario_file', type=str, default=None,
                       help='YAML file with pedestrian scenario')
    parser.add_argument('--skip_baselines', action='store_true',
                       help='Skip MPC and VLM-MPC baseline runs')
    parser.add_argument('--create_scenario', type=str, default=None,
                       help='Create example scenario file at given path')
    parser.add_argument('--dry_run', action='store_true',
                       help='Print configuration without running')

    args = parser.parse_args()

    # Create example scenario if requested
    if args.create_scenario:
        create_example_scenario_file(Path(args.create_scenario))
        if args.dry_run:
            return

    # Create sweep runner
    output_dir = Path(args.output_dir)
    runner = ParameterSweepRunner(output_dir, args.scenario_file)

    if args.dry_run:
        print("DRY RUN - Configuration:")
        print(f"  Output: {output_dir.absolute()}")
        print(f"  Scenario: {args.scenario_file or 'Default'}")
        print(f"  DWA configurations: {len(runner.inflation_radii) * len(runner.max_velocities)}")
        print(f"  Skip baselines: {args.skip_baselines}")
        return

    # Run sweep
    runner.run_sweep(skip_baselines=args.skip_baselines)


if __name__ == '__main__':
    main()
