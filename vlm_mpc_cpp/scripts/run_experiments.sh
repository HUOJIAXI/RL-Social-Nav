#!/bin/bash
# Automated experiment runner for VLM-MPC comparison
#
# Usage: ./run_experiments.sh [scenario_id] [condition] [num_trials]
#
# Example: ./run_experiments.sh S1 vlm 10

set -e  # Exit on error

# Configuration
SCENARIO=${1:-"S1"}
CONDITION=${2:-"vlm"}  # "vlm" or "no_vlm"
NUM_TRIALS=${3:-10}
LOG_DIR="$HOME/ros2_logs/social_mpc_nav/experiments"
TIMEOUT=120  # seconds

# Create log directory
mkdir -p "$LOG_DIR"

echo "========================================="
echo "VLM-MPC Comparison Experiment Runner"
echo "========================================="
echo "Scenario: $SCENARIO"
echo "Condition: $CONDITION"
echo "Trials: $NUM_TRIALS"
echo "Log directory: $LOG_DIR"
echo "========================================="

# Determine enable_vlm flag
if [ "$CONDITION" = "vlm" ]; then
    ENABLE_VLM="true"
else
    ENABLE_VLM="false"
fi

# Run trials
for ((trial=0; trial<NUM_TRIALS; trial++)); do
    echo ""
    echo ">>> Running trial $trial/$NUM_TRIALS for ${SCENARIO}_${CONDITION}"
    echo ""

    # Set trial-specific log directory
    TRIAL_LOG_DIR="$LOG_DIR/trial_${SCENARIO}_${CONDITION}_${trial}"
    mkdir -p "$TRIAL_LOG_DIR"

    # Launch Gazebo + navigation in background
    ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
        enable_vlm:=$ENABLE_VLM \
        log_directory:=$TRIAL_LOG_DIR \
        log_mpc_to_csv:=true \
        enable_debug_logging:=false &

    LAUNCH_PID=$!
    echo "Launched with PID: $LAUNCH_PID"

    # Wait for initialization (10 seconds)
    sleep 10

    # Publish goal via RViz2 goal pose (or use goal topic)
    # TODO: Set scenario-specific goal based on $SCENARIO
    # For now, use fixed goal for each scenario
    case $SCENARIO in
        S1|S2)
            GOAL_X=10.0
            GOAL_Y=0.0
            ;;
        S3|S4|S5)
            GOAL_X=15.0
            GOAL_Y=0.0
            ;;
        S6|S7)
            GOAL_X=8.0
            GOAL_Y=5.0
            ;;
        *)
            GOAL_X=10.0
            GOAL_Y=5.0
            ;;
    esac

    echo "Publishing goal: ($GOAL_X, $GOAL_Y)"
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}}}"

    # Wait for trial completion or timeout
    echo "Waiting for completion (timeout: ${TIMEOUT}s)..."
    sleep $TIMEOUT

    # Kill launch
    echo "Stopping trial..."
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 3

    # Rename log file with trial info
    if [ -f "$TRIAL_LOG_DIR/mpc_casadi_log.csv" ]; then
        mv "$TRIAL_LOG_DIR/mpc_casadi_log.csv" \
           "$LOG_DIR/${SCENARIO}_${CONDITION}_trial${trial}_mpc_casadi_log.csv"
        echo "✓ Log saved: ${SCENARIO}_${CONDITION}_trial${trial}_mpc_casadi_log.csv"
    else
        echo "✗ Warning: Log file not found for trial $trial"
    fi

    # Clean up
    rm -rf "$TRIAL_LOG_DIR"

    # Wait between trials
    echo "Waiting 5s before next trial..."
    sleep 5
done

echo ""
echo "========================================="
echo "Experiment completed!"
echo "Logs saved to: $LOG_DIR"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Run experiments for other scenarios/conditions"
echo "2. Extract metrics: python scripts/extract_metrics.py --log-dir $LOG_DIR"
echo "3. Visualize results: python scripts/visualize_comparison.py --metrics metrics.json"
