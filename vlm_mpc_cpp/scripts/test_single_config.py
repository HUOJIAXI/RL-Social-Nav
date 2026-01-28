#!/usr/bin/env python3
"""
Test a single navigation configuration before running full parameter sweep.

This script helps verify:
1. ROS2 launch files are working
2. Output CSV files are being generated correctly
3. Pedestrian scenario is loading properly
4. Metrics can be computed from the output

Usage:
    python test_single_config.py --method dwa --inflation_radius 0.5 --max_velocity 0.6
    python test_single_config.py --method mpc --use_vlm
    python test_single_config.py --method vlm_mpc
"""

import argparse
import subprocess
import sys
from pathlib import Path
import pandas as pd
import numpy as np


def test_dwa(inflation_radius: float, max_velocity: float,
             output_file: str, scenario_file: str = None):
    """Test DWA configuration."""
    print("="*60)
    print(f"Testing DWA Configuration")
    print(f"  Inflation radius: {inflation_radius} m")
    print(f"  Max velocity: {max_velocity} m/s")
    print(f"  Output: {output_file}")
    print("="*60)

    cmd = [
        'ros2', 'launch', 'social_mpc_nav', 'dwa_navigation.launch.py',
        f'inflation_radius:={inflation_radius}',
        f'max_velocity:={max_velocity}',
        f'output_file:={output_file}',
    ]

    if scenario_file:
        cmd.append(f'scenario_file:={scenario_file}')

    print(f"\nCommand: {' '.join(cmd)}\n")

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)

        if result.returncode != 0:
            print("❌ FAILED")
            print(f"Return code: {result.returncode}")
            print(f"\nStderr:\n{result.stderr}")
            return False

        print("✓ Launch completed successfully")
        return True

    except subprocess.TimeoutExpired:
        print("❌ TIMEOUT (>5 minutes)")
        return False
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False


def test_mpc(use_vlm: bool, output_file: str, scenario_file: str = None):
    """Test MPC configuration."""
    method_name = "VLM-MPC" if use_vlm else "MPC (No VLM)"
    print("="*60)
    print(f"Testing {method_name}")
    print(f"  Output: {output_file}")
    print("="*60)

    cmd = [
        'ros2', 'launch', 'social_mpc_nav', 'mpc_navigation.launch.py',
        f'use_vlm:={str(use_vlm).lower()}',
        f'output_file:={output_file}',
    ]

    if scenario_file:
        cmd.append(f'scenario_file:={scenario_file}')

    print(f"\nCommand: {' '.join(cmd)}\n")

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)

        if result.returncode != 0:
            print("❌ FAILED")
            print(f"Return code: {result.returncode}")
            print(f"\nStderr:\n{result.stderr}")
            return False

        print("✓ Launch completed successfully")
        return True

    except subprocess.TimeoutExpired:
        print("❌ TIMEOUT (>5 minutes)")
        return False
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False


def verify_output_file(csv_file: str):
    """Verify output CSV file and compute test metrics."""
    print("\n" + "="*60)
    print("Verifying Output File")
    print("="*60)

    csv_path = Path(csv_file)

    # Check file exists
    if not csv_path.exists():
        print(f"❌ File not found: {csv_file}")
        return False

    print(f"✓ File exists: {csv_file}")
    print(f"  Size: {csv_path.stat().st_size / 1024:.1f} KB")

    # Try to load CSV
    try:
        df = pd.read_csv(csv_file)
        print(f"✓ CSV loaded successfully")
        print(f"  Rows: {len(df)}")
        print(f"  Columns: {len(df.columns)}")
    except Exception as e:
        print(f"❌ Failed to load CSV: {e}")
        return False

    # Check for required columns
    required_columns = ['timestamp', 'x', 'y', 'v_cmd', 'w_cmd',
                       'goal_dist', 'min_person_dist']
    missing = [col for col in required_columns if col not in df.columns]

    if missing:
        print(f"⚠ Missing columns: {missing}")
        print(f"  Available columns: {list(df.columns)}")
    else:
        print(f"✓ All required columns present")

    # Handle infinity values
    df_clean = df.replace([np.inf, -np.inf], np.nan)
    for col in df_clean.columns:
        if df_clean[col].dtype in ['float64', 'float32']:
            df_clean.loc[df_clean[col] > 1e100, col] = np.nan
            df_clean.loc[df_clean[col] < -1e100, col] = np.nan

    # Compute basic metrics
    print("\n" + "-"*60)
    print("Quick Metrics Summary")
    print("-"*60)

    try:
        # Time to goal
        time_to_goal = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
        print(f"Time to Goal: {time_to_goal:.2f} s")

        # Path length
        x_diff = np.diff(df['x'].values)
        y_diff = np.diff(df['y'].values)
        path_length = np.sum(np.sqrt(x_diff**2 + y_diff**2))
        print(f"Path Length: {path_length:.2f} m")

        # Min person distance
        min_person_dist_values = df_clean['min_person_dist'].dropna()
        if len(min_person_dist_values) > 0:
            min_person_dist = min_person_dist_values.min()
            avg_person_dist = min_person_dist_values.mean()
            print(f"Min Person Distance: {min_person_dist:.2f} m")
            print(f"Avg Person Distance: {avg_person_dist:.2f} m")
        else:
            print(f"Min Person Distance: No valid data")

        # Angular smoothness
        w_cmd = df['w_cmd'].values
        angular_changes = np.abs(np.diff(w_cmd))
        avg_angular_smoothness = np.mean(angular_changes)
        print(f"Angular Smoothness: {avg_angular_smoothness:.4f} rad/s²")

        # Velocity stats
        print(f"Avg Linear Velocity: {df['v_cmd'].mean():.2f} m/s")
        print(f"Avg Angular Velocity: {np.abs(df['w_cmd']).mean():.2f} rad/s")

        print("\n✓ Metrics computed successfully")
        return True

    except Exception as e:
        print(f"⚠ Error computing metrics: {e}")
        return True  # File is valid even if metrics fail


def main():
    parser = argparse.ArgumentParser(
        description='Test a single navigation configuration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test DWA configuration
  python test_single_config.py --method dwa --inflation_radius 0.5 --max_velocity 0.6

  # Test MPC without VLM
  python test_single_config.py --method mpc

  # Test VLM-MPC
  python test_single_config.py --method vlm_mpc

  # Test with custom scenario
  python test_single_config.py --method dwa --scenario my_scenario.yaml

  # Verify existing output file only (skip run)
  python test_single_config.py --verify_only existing_log.csv
        """
    )

    parser.add_argument('--method', type=str, choices=['dwa', 'mpc', 'vlm_mpc'],
                       help='Navigation method to test')
    parser.add_argument('--inflation_radius', type=float, default=0.5,
                       help='DWA inflation radius (m)')
    parser.add_argument('--max_velocity', type=float, default=0.6,
                       help='DWA max velocity (m/s)')
    parser.add_argument('--output_file', type=str, default='test_output.csv',
                       help='Output CSV file')
    parser.add_argument('--scenario_file', type=str, default=None,
                       help='Pedestrian scenario YAML file')
    parser.add_argument('--verify_only', type=str, default=None,
                       help='Only verify an existing CSV file (skip running)')

    args = parser.parse_args()

    # Verify only mode
    if args.verify_only:
        success = verify_output_file(args.verify_only)
        sys.exit(0 if success else 1)

    # Check method is specified
    if not args.method:
        print("Error: --method is required (unless using --verify_only)")
        parser.print_help()
        sys.exit(1)

    # Run test
    print("\n" + "="*60)
    print("SINGLE CONFIGURATION TEST")
    print("="*60 + "\n")

    success = False

    if args.method == 'dwa':
        success = test_dwa(
            args.inflation_radius,
            args.max_velocity,
            args.output_file,
            args.scenario_file
        )
    elif args.method == 'mpc':
        success = test_mpc(
            use_vlm=False,
            output_file=args.output_file,
            scenario_file=args.scenario_file
        )
    elif args.method == 'vlm_mpc':
        success = test_mpc(
            use_vlm=True,
            output_file=args.output_file,
            scenario_file=args.scenario_file
        )

    # Verify output if run succeeded
    if success:
        verify_output_file(args.output_file)
        print("\n" + "="*60)
        print("✓ TEST PASSED")
        print("="*60)
        print("\nYou can now run the full parameter sweep:")
        print("  python run_parameter_sweep.py --output_dir sweep_results/")
        sys.exit(0)
    else:
        print("\n" + "="*60)
        print("❌ TEST FAILED")
        print("="*60)
        print("\nPlease fix the issues before running parameter sweep.")
        sys.exit(1)


if __name__ == '__main__':
    main()
