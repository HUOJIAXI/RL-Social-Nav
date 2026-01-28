#!/bin/bash
# Complete VLM vs No-VLM comparison pipeline
#
# Usage: ./run_full_comparison.sh [num_trials] [scenarios...]
# Example: ./run_full_comparison.sh 10 S1 S2 S3

set -e

# Configuration
NUM_TRIALS=${1:-10}
shift
SCENARIOS=("${@:-S1 S2 S3}")
LOG_DIR="$HOME/ros2_logs/social_mpc_nav/experiments"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
EXPERIMENT_DIR="$LOG_DIR/comparison_$TIMESTAMP"

mkdir -p "$EXPERIMENT_DIR"

echo "========================================="
echo "VLM-MPC Complete Comparison Pipeline"
echo "========================================="
echo "Trials per condition: $NUM_TRIALS"
echo "Scenarios: ${SCENARIOS[*]}"
echo "Output directory: $EXPERIMENT_DIR"
echo "========================================="

# Run all experiments
for scenario in "${SCENARIOS[@]}"; do
    echo ""
    echo ">>> Testing scenario: $scenario"
    echo ""

    # Test WITHOUT VLM
    echo "  -> Running NO-VLM condition..."
    ./scripts/run_experiments.sh "$scenario" "no_vlm" "$NUM_TRIALS"

    # Test WITH VLM
    echo "  -> Running VLM condition..."
    ./scripts/run_experiments.sh "$scenario" "vlm" "$NUM_TRIALS"
done

# Extract metrics
echo ""
echo ">>> Extracting metrics..."
python scripts/extract_metrics.py \
    --log-dir "$LOG_DIR" \
    --output "$EXPERIMENT_DIR/metrics.json"

# Generate visualizations
echo ""
echo ">>> Generating visualizations..."
mkdir -p "$EXPERIMENT_DIR/plots"
python scripts/visualize_comparison.py \
    --metrics "$EXPERIMENT_DIR/metrics.json" \
    --output "$EXPERIMENT_DIR/plots/"

# Generate summary report
echo ""
echo ">>> Generating summary report..."
python scripts/generate_summary_report.py \
    --metrics "$EXPERIMENT_DIR/metrics.json" \
    --output "$EXPERIMENT_DIR/report.md"

echo ""
echo "========================================="
echo "Comparison complete!"
echo "========================================="
echo "Results saved to: $EXPERIMENT_DIR"
echo "View report: $EXPERIMENT_DIR/report.md"
echo "View plots: $EXPERIMENT_DIR/plots/"
echo "========================================="
