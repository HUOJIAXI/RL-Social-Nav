# ✅ Coordinate Frame Fix - Implementation Status

## Implementation Complete

The TF2 transformation code has been successfully added to `mpc_controller_node.cpp` to transform robot positions from `odom` frame to `map` frame.

---

## Changes Made

### 1. **Added TF2 Includes** (mpc_controller_node.cpp:19-28)
```cpp
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
```

### 2. **Added Member Variables** (mpc_controller_node.cpp:1051-1055)
```cpp
// TF2 for coordinate frame transformations
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::string map_frame_;
std::string odom_frame_;
```

### 3. **Initialized TF2 in Constructor** (mpc_controller_node.cpp:122-128)
```cpp
// Initialize TF2 for coordinate frame transformations
tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// Declare coordinate frame parameters
map_frame_ = declare_parameter<std::string>("map_frame", "map");
odom_frame_ = declare_parameter<std::string>("odom_frame", "tiago_official/odom");
```

### 4. **Modified getLatestData() Function** (mpc_controller_node.cpp:190-250)
```cpp
bool getLatestData(...)
{
  // Get robot position in odom frame
  double x_odom = latest_odom_->pose.pose.position.x;
  double y_odom = latest_odom_->pose.pose.position.y;
  double yaw_odom = tf2::getYaw(latest_odom_->pose.pose.orientation);

  // Transform robot position from odom frame to map frame
  try {
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(
        map_frame_,                    // target frame: map
        odom_frame_,                   // source frame: odom
        latest_odom_->header.stamp,
        rclcpp::Duration::from_seconds(0.1));

    // Create pose in odom frame and transform to map frame
    geometry_msgs::msg::PoseStamped pose_odom;
    pose_odom.header.frame_id = odom_frame_;
    pose_odom.pose = latest_odom_->pose.pose;

    geometry_msgs::msg::PoseStamped pose_map;
    tf2::doTransform(pose_odom, pose_map, transform_stamped);

    // Use transformed position
    state.x = pose_map.pose.position.x;
    state.y = pose_map.pose.position.y;
    state.yaw = tf2::getYaw(pose_map.pose.orientation);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(..., "Could not transform: %s", ex.what());
    // Fallback to odom position
    state.x = x_odom;
    state.y = y_odom;
    state.yaw = yaw_odom;
  }

  return true;
}
```

---

## Build Status

✅ **Package rebuilt successfully**
```bash
colcon build --packages-select social_mpc_nav --symlink-install
Summary: 1 package finished [51.7s]
```

All dependencies (tf2, tf2_ros, tf2_geometry_msgs) were already present in CMakeLists.txt and package.xml.

---

## Current Issue: Missing TF Transform

### Problem Detected
When running the MPC controller, the following warning appears:

```
[WARN] [mpc_controller_node]: Could not transform from tiago_official/odom to map:
  Could not find a connection between 'map' and 'tiago_official/odom'
  because they are not part of the same tree. Tf has two or more unconnected trees.
```

### Root Cause
The TF tree is **disconnected**:

**Current TF Tree Status:**
```
Tree 1: map → ? (people positions are here)
Tree 2: tiago_official/odom → base_link (robot odometry is here)
```

These two trees are NOT connected, so there's no transform path from `tiago_official/odom` to `map`.

### Why This Happens
1. **People data**: Published in `map` frame by `task_generator_node`
2. **Robot odometry**: Published in `tiago_official/odom` frame by `task_generator_node`
3. **Missing link**: No transform published connecting these two frames

---

## Solutions

### Solution 1: Use Ground-Truth Localization Node (Recommended for Simulation)

The `gt_localization_node` already exists in the codebase and can publish the `map → base_link` transform from Gazebo ground-truth.

**How to enable:**
```bash
# Run gt_localization_node to publish map → base_link transform
ros2 run social_mpc_nav gt_localization_node \
  --ros-args \
  -p robot_model_name:=tiago \
  -p map_frame:=map \
  -p base_link_frame:=base_link \
  -p model_states_topic:=/gazebo/model_states
```

**This creates the TF tree:**
```
map → base_link ← odom
```

Then the MPC controller can transform: `tiago_official/odom → base_link → map`

### Solution 2: Update MPC to Subscribe to Ground-Truth Pose Directly

Instead of subscribing to `/task_generator_node/tiago_official/odom` (which is in odom frame), subscribe to a ground-truth pose topic that's already in the map frame.

**Modify mpc_controller_node.cpp:**
```cpp
// Option: Subscribe to ground-truth pose in map frame
gt_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
  "/ground_truth_pose", rclcpp::QoS(10),
  std::bind(&MPCControllerNode::onGroundTruthPose, this, std::placeholders::_1));
```

### Solution 3: Publish Static Transform (Quick Fix for Testing)

Publish a static transform between map and odom frames:

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map tiago_official/odom
```

**Note**: This assumes the frames are perfectly aligned (which may be true in Gazebo initially, but will drift over time).

---

## Recommended Action Plan

### Immediate (For Testing):
1. ✅ **TF2 transformation code added** (Done - this document)
2. 🔧 **Add gt_localization_node to launch file**
3. ✅ **Test with connected TF tree**

### Implementation:

**Update `mpc_with_global_planner.launch.py`:**
```python
def generate_launch_description():
    return LaunchDescription([
        # ... existing nodes ...

        # Add ground-truth localization
        Node(
            package='social_mpc_nav',
            executable='gt_localization_node',
            name='gt_localization_node',
            parameters=[{
                'robot_model_name': 'tiago',
                'map_frame': 'map',
                'base_link_frame': 'base_link',
                'model_states_topic': '/gazebo/model_states'
            }],
            output='screen'
        ),
    ])
```

---

## Verification

### After fixing TF tree, verify with:

**1. Check TF tree is connected:**
```bash
ros2 run tf2_tools view_frames
# Should show: map → base_link ← odom (all connected)
```

**2. Check transform is available:**
```bash
ros2 run tf2_ros tf2_echo map tiago_official/odom
# Should show transform without errors
```

**3. Check MPC controller logs:**
```bash
ros2 launch social_mpc_nav mpc_with_global_planner.launch.py enable_debug_logging:=true
# Should NOT see "Could not transform" warnings
```

**4. Run verification script:**
```bash
python3 verify_coordinates.py
# Should show robot position in map frame (after internal transformation)
```

---

## Summary

| Component | Status | Notes |
|-----------|--------|-------|
| **TF2 Code** | ✅ Complete | Transformation logic implemented |
| **Build** | ✅ Success | Package compiles without errors |
| **Dependencies** | ✅ Present | tf2_ros already in CMakeLists.txt |
| **TF Tree** | ❌ Disconnected | Need to run gt_localization_node |
| **Testing** | ⏳ Pending | Requires TF tree fix first |

---

## Next Steps

1. Update launch file to include `gt_localization_node`
2. Test complete system with connected TF tree
3. Verify robot and people positions are in same frame
4. Update COORDINATE_FRAME_ISSUE.md with final results

---

**Status**: Implementation complete, pending TF tree connection for full functionality.
