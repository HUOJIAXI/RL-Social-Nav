#!/bin/bash
# Test script to verify that min_obs_dist and min_person_dist are being logged correctly

set -e

TEST_LOG_DIR="/tmp/test_mpc_logging"
rm -rf "$TEST_LOG_DIR"
mkdir -p "$TEST_LOG_DIR"

echo "========================================="
echo "Testing CSV Logging Fix"
echo "========================================="
echo "Log directory: $TEST_LOG_DIR"
echo ""

# Launch system in background
echo "Launching MPC system..."
ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
    enable_vlm:=false \
    log_directory:="$VLM_TEST_LOG_DIR" \
    log_mpc_to_csv:=true &

LAUNCH_PID=$!
echo "Launched with PID: $LAUNCH_PID"

# Wait for initialization
echo "Waiting for initialization (10s)..."
sleep 10

# Publish test goal
echo "Publishing test goal..."
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}}}"

# Run for 20 seconds
echo "Running for 20 seconds..."
sleep 20

# Kill launch
echo "Stopping system..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# Check CSV file
CSV_FILE="$TEST_LOG_DIR/mpc_casadi_log.csv"

if [ ! -f "$CSV_FILE" ]; then
    echo "❌ FAILED: CSV file not found at $CSV_FILE"
    exit 1
fi

echo ""
echo "========================================="
echo "CSV File Analysis"
echo "========================================="

# Show header
echo "CSV Header:"
head -1 "$CSV_FILE"
echo ""

# Show first 5 data rows
echo "First 5 data rows:"
tail -n +2 "$CSV_FILE" | head -5
echo ""

# Count non-zero min_obs_dist values
NON_ZERO_OBS=$(tail -n +2 "$CSV_FILE" | awk -F',' '{if ($11 > 0.001) count++} END {print count+0}')
TOTAL_ROWS=$(tail -n +2 "$CSV_FILE" | wc -l)

echo "Statistics:"
echo "  Total data rows: $TOTAL_ROWS"
echo "  Non-zero min_obs_dist values: $NON_ZERO_OBS"

# Get min and max values
MIN_OBS=$(tail -n +2 "$CSV_FILE" | awk -F',' '{print $11}' | sort -n | head -1)
MAX_OBS=$(tail -n +2 "$CSV_FILE" | awk -F',' '{print $11}' | sort -n | tail -1)
MIN_PERSON=$(tail -n +2 "$CSV_FILE" | awk -F',' '{print $12}' | sort -n | head -1)
MAX_PERSON=$(tail -n +2 "$CSV_FILE" | awk -F',' '{print $12}' | sort -n | tail -1)

echo "  min_obs_dist range: [$MIN_OBS, $MAX_OBS]"
echo "  min_person_dist range: [$MIN_PERSON, $MAX_PERSON]"
echo ""

# Verify fix
if [ "$NON_ZERO_OBS" -gt 0 ]; then
    echo "✅ SUCCESS: min_obs_dist contains non-zero values!"
    echo "   The logging fix is working correctly."
else
    echo "❌ FAILED: All min_obs_dist values are still zero"
    echo "   The bug may not be fixed or no obstacles were detected"
fi

echo ""
echo "Full log saved to: $CSV_FILE"
echo "========================================="
