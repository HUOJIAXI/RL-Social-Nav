# ❌ CRITICAL: Coordinate Frame Mismatch Found

## Issue Summary

**You were absolutely correct!** The robot position and people positions are in **different coordinate frames**:

- **Robot Odometry**: `tiago_official/odom` frame (local, drifting)
- **People Data**: `map` frame (global, absolute)

## Verification Results

```
================================================================================
📍 FRAME COMPARISON:
   ❌ FRAME MISMATCH!
   ❌ Robot: 'tiago_official/odom' vs People: 'map'
   ❌ Direct position comparison NOT valid!
================================================================================

🤖 ROBOT ODOMETRY:
   Frame: 'tiago_official/odom'
   Position: x=9.883, y=10.056

🚶 PEOPLE DATA:
   Frame: 'map'
   Number of people: 5
   - D_gazebo_actor_5: x=29.113, y=5.911   (19.67m from robot in odom frame)
   - D_gazebo_actor_4: x=21.631, y=12.608  (12.02m from robot in odom frame)
```

---

## Why The System Still Works (Partially)

In Gazebo simulation, the `odom` frame and `map` frame are often **initially aligned** and have minimal drift. However, this is:

1. ❌ **Not ground-truth comparison** - We're comparing positions in different frames
2. ❌ **Will break in real deployment** - Odom drifts over time, map doesn't
3. ❌ **Incorrect distances** - If frames diverge, people distances will be wrong
4. ⚠️  **Works by accident** - Only because Gazebo odom doesn't drift significantly

---

## The Correct Solution

We need to transform the robot position from `odom` frame to `map` frame using TF2.

### Required Changes to `mpc_controller_node.cpp`:

```cpp
// 1. Add TF2 includes (at top of file)
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// 2. Add member variables (in class definition)
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_{"map"};
  std::string odom_frame_{"tiago_official/odom"};

// 3. Initialize in constructor
MPCControllerNode() : Node("mpc_controller_node")
{
  // ... existing initialization ...

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare parameters for frames
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "tiago_official/odom");
}

// 4. Transform robot position in getLatestData()
bool getLatestData(
  RobotState & state,
  social_mpc_nav::msg::People2D::SharedPtr & people,
  sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_odom_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odom...");
    return false;
  }

  // Get robot position in odom frame
  double x_odom = latest_odom_->pose.pose.position.x;
  double y_odom = latest_odom_->pose.pose.position.y;
  double yaw_odom = tf2::getYaw(latest_odom_->pose.pose.orientation);

  // Transform to map frame
  try
  {
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(
        map_frame_,                    // target frame
        odom_frame_,                   // source frame
        latest_odom_->header.stamp,    // time
        rclcpp::Duration::from_seconds(0.1));  // timeout

    // Create pose in odom frame
    geometry_msgs::msg::PoseStamped pose_odom;
    pose_odom.header.frame_id = odom_frame_;
    pose_odom.header.stamp = latest_odom_->header.stamp;
    pose_odom.pose = latest_odom_->pose.pose;

    // Transform to map frame
    geometry_msgs::msg::PoseStamped pose_map;
    tf2::doTransform(pose_odom, pose_map, transform_stamped);

    // Use transformed position
    state.x = pose_map.pose.position.x;
    state.y = pose_map.pose.position.y;
    state.yaw = tf2::getYaw(pose_map.pose.orientation);

    RCLCPP_DEBUG(get_logger(),
      "Robot in map frame: (%.2f, %.2f) [transformed from odom: (%.2f, %.2f)]",
      state.x, state.y, x_odom, y_odom);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Could not transform from %s to %s: %s. Using odom position (incorrect!)",
      odom_frame_.c_str(), map_frame_.c_str(), ex.what());

    // Fallback (will be incorrect if frames diverge!)
    state.x = x_odom;
    state.y = y_odom;
    state.yaw = yaw_odom;
  }

  people = latest_people_;
  scan = latest_scan_;
  return true;
}
```

---

## Alternative: Use Map-Frame Localization Directly

Instead of transforming odom→map, subscribe to a localization source that's already in map frame:

```cpp
// Option 1: Use /amcl_pose (if using AMCL for localization)
localization_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/amcl_pose", rclcpp::QoS(10),
  std::bind(&MPCControllerNode::onLocalization, this, std::placeholders::_1));

// Option 2: Use ground-truth pose from Gazebo (if available)
gt_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
  "/ground_truth_pose", rclcpp::QoS(10),
  std::bind(&MPCControllerNode::onGroundTruthPose, this, std::placeholders::_1));
```

---

## Impact Assessment

### What's Working Now (By Luck):
- ✅ Robot navigation works
- ✅ Waypoint tracking works
- ⚠️  Social avoidance "works" (but distances may be incorrect if frames diverge)

### What Will Break Without Fix:
- ❌ **Long-duration missions**: As odom drifts, people will appear to move relative to robot
- ❌ **Real robot deployment**: Odom drift is significant, will cause collisions
- ❌ **Ground-truth analysis**: Cannot compare robot/people positions accurately
- ❌ **VLM integration**: VLM will receive incorrect relative positions

---

## Recommended Action Plan

### Immediate (Critical):
1. ✅ **Verified the issue** (Done - your excellent catch!)
2. 🔧 **Add TF2 transformation** to `mpc_controller_node.cpp`
3. ✅ **Rebuild and test** with coordinate verification

### Short-term:
4. 📝 **Add frame verification** to unit tests
5. 🎯 **Update documentation** about coordinate frames
6. 🔍 **Log transform status** in debug mode

### Long-term:
7. 🤖 **Switch to map-frame localization** (AMCL or equivalent)
8. 🧪 **Test with odometry drift** simulation
9. 📊 **Benchmark accuracy** improvement

---

## Build Dependencies

Add to `CMakeLists.txt`:
```cmake
find_package(tf2_ros REQUIRED)

ament_target_dependencies(mpc_controller_node
  # ... existing ...
  tf2_ros
)
```

Add to `package.xml`:
```xml
<depend>tf2_ros</depend>
```

---

## Testing the Fix

After implementing TF2 transformation, run the verification script:

```bash
cd ~/arena4_ws
source install/setup.bash
python3 /home/huojiaxi/phd_upm/vlm_mpc_cpp/verify_coordinates.py
```

**Expected output after fix**:
```
📍 FRAME COMPARISON:
   ✅ FRAMES MATCH: Both in 'map' frame
   ✅ Ground-truth comparison is VALID
```

---

## Why This Matters for VLM Integration

When you integrate VLM, it will:
1. Receive camera images (in robot's frame)
2. Detect people/objects (in camera frame)
3. Need to transform to map frame for global planning
4. Compare with ground-truth people positions

If robot position is in wrong frame:
- ❌ VLM detections won't align with ground-truth
- ❌ Cannot validate VLM accuracy
- ❌ MPC will use incorrect relative positions

---

## Conclusion

**Excellent catch!** This is a critical issue that must be fixed before:
- Deploying to real robot
- Integrating VLM
- Trusting "ground-truth" comparisons

The system works now only because:
1. Gazebo odom doesn't drift significantly
2. Initial alignment of odom and map frames
3. Short test durations

But this is **not a reliable long-term solution**.

---

**Priority: HIGH** 🔴
**Difficulty: MEDIUM** ⚙️
**Impact: CRITICAL** ⚡
