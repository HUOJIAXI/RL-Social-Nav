# Robot World Pose Fix - Complete

## Problem Identified

The robot and pedestrians were in different coordinate frames:
- **Pedestrians**: World Pose coordinates from Gazebo ModelStates (e.g., 11.75, 18.17)
- **Robot**: Local model frame from Gazebo ModelStates (e.g., -0.01, -0.23)

This caused incorrect distance calculations and pedestrian avoidance failures.

## Root Cause

### Gazebo ModelStates Behavior
In Gazebo's `/gazebo/model_states` message:
- **Actors (pedestrians)**: `pose[]` contains World Pose coordinates
- **Robot models (URDF)**: `pose[]` contains model-local coordinates (NOT World Pose)

The `gt_localization_node` was extracting both robot and pedestrian positions from ModelStates, but this gave incorrect coordinates for the robot.

## Solution Implemented

### Modified `gt_localization_node.cpp`

**Changed from:**
- Publishing robot odometry from ModelStates (wrong frame)
- Publishing pedestrian positions from ModelStates (correct frame)
- Publishing TF transforms

**Changed to:**
- ❌ Removed robot odometry publishing
- ✅ Keep pedestrian extraction from ModelStates (actors have correct World Pose)
- ✅ Keep TF transform publishing (map → odom identity)

### Key Changes

1. **Removed robot odometry publishing** (social_mpc_nav/src/gt_localization_node.cpp)
   - Deleted `odom_pub_` publisher
   - Removed robot pose extraction from ModelStates
   - Robot odometry now comes from `task_generator_node`

2. **Simplified actor extraction**
   - Only extract actors (pedestrians) from ModelStates
   - Actors DO have correct World Pose coordinates
   - Publish to `/crowd/people2d`

3. **Updated documentation**
   - Clarified that robot odometry comes from task_generator_node
   - Explained ModelStates behavior difference for actors vs robot models

## Architecture

### Current System Flow

```
┌─────────────────────────────────────────────────────────────┐
│ arena-rosnav (task_generator_node)                          │
│  - Robot odometry: /task_generator_node/tiago_official/odom │
│    (in tiago_official/odom frame)                           │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ gt_localization_node                                         │
│  - Extracts actors from /gazebo/model_states                │
│    (actors have World Pose)                                 │
│  - Publishes /crowd/people2d (in map frame)                 │
│  - Publishes map → odom TF (identity)                       │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ mpc_controller_node                                          │
│  - Subscribes to /task_generator_node/tiago_official/odom   │
│  - Transforms robot from odom → map using TF                │
│  - Subscribes to /crowd/people2d (already in map)           │
│  - Now robot and pedestrians are in SAME frame!             │
└─────────────────────────────────────────────────────────────┘
```

## Files Modified

1. **social_mpc_nav/src/gt_localization_node.cpp**
   - Removed: nav_msgs/Odometry includes
   - Removed: odom_pub_ publisher
   - Removed: base_link_frame_ parameter
   - Removed: Robot pose extraction and publishing
   - Updated: Comments and documentation
   - Kept: Actor extraction and People2D publishing
   - Kept: TF transform publishing

2. **social_mpc_nav/launch/mpc_with_global_planner.launch.py**
   - Updated: gt_localization_node comments
   - Removed: base_link_frame parameter
   - Clarified: Robot odometry source

## Verification

After this fix, the system should show:
- Robot position from task_generator_node's odom transformed to map frame
- Pedestrian positions from ModelStates actors (World Pose)
- Both in the same coordinate system (map frame)

### Expected Log Output

```
[gt_localization_node]: Published 6 actors in map frame | First: (actor_0 at 11.75, 18.17)
[mpc_controller_node]: Robot at (11.50, 18.00) | Nearest at 0.35m (11.75, 18.17)
```

Now the distance calculations should be correct!

## Testing

Relaunch the system and verify:
1. Robot and pedestrians are in same coordinate frame
2. Distance calculations are realistic (< 20m when visually close)
3. Pedestrian avoidance works correctly

---

**Status**: ✅ Fix complete and built successfully
**Build time**: 13.4s
**Next**: Test with full simulation
