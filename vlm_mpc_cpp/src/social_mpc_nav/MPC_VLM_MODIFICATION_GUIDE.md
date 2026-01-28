# MPC Controller VLM Integration Guide

This guide shows how to create `mpc_controller_vlm_node.cpp` from the original `mpc_controller_node.cpp`.

## Quick Creation Method

```bash
# Copy the original to create VLM version
cp social_mpc_nav/src/mpc_controller_node.cpp social_mpc_nav/src/mpc_controller_vlm_node.cpp
```

Then apply the following modifications:

---

## Modification 1: Add VLM Includes (After line ~20)

**Location**: Near top of file, after existing includes

```cpp
// ADD THESE LINES:
#include "social_mpc_nav/msg/vlm_parameters.hpp"
#include "social_mpc_nav/mpc_vlm_helpers.hpp"
```

---

## Modification 2: Add VLM Parameters Declaration (In constructor, after line ~90)

**Location**: In constructor, after existing parameter declarations

```cpp
// ADD THESE LINES (after line ~90, after min_obstacle_distance parameter):
// VLM cost weights
w_vlm_directional_ = declare_parameter<double>("w_vlm_directional", 1.0);
w_vlm_action_ = declare_parameter<double>("w_vlm_action", 2.0);
w_vlm_scene_ = declare_parameter<double>("w_vlm_scene", 1.5);
w_vlm_personal_ = declare_parameter<double>("w_vlm_personal", 5.0);
```

---

## Modification 3: Add VLM Subscription (In constructor, after line ~150)

**Location**: After creating other subscriptions (scan_sub_, path_sub_, etc.)

```cpp
// ADD THESE LINES:
// VLM parameters subscription
vlm_params_sub_ = create_subscription<social_mpc_nav::msg::VLMParameters>(
  "/vlm/mpc_parameters", rclcpp::QoS(10),
  std::bind(&MPCControllerNode::onVLMParameters, this, std::placeholders::_1));

RCLCPP_INFO(get_logger(), "Subscribed to VLM parameters: /vlm/mpc_parameters");
```

---

## Modification 4: Add VLM Callback Method (In private section, around line ~220)

**Location**: After other callback methods (onOdom, onCrowd, onScan, onPath)

```cpp
// ADD THIS METHOD:
void onVLMParameters(const social_mpc_nav::msg::VLMParameters::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(vlm_params_mutex_);
  latest_vlm_params_ = msg;

  RCLCPP_DEBUG(get_logger(),
               "Received VLM parameters: speed_scale=%.2f, min_dist=%.2f, source=%s",
               msg->speed_scale, msg->min_personal_distance, msg->source.c_str());
}
```

---

## Modification 5: Apply VLM Modulation to Social Contract (In controlLoop, after line ~370)

**Location**: In `controlLoop()` method, immediately after `contract_helper_.compute()`

**FIND** this code (around line 370):
```cpp
auto contract = contract_helper_.compute(
  robot.x, robot.y, robot.yaw, default_v_max_, people);
```

**REPLACE** with:
```cpp
auto contract = contract_helper_.compute(
  robot.x, robot.y, robot.yaw, default_v_max_, people);

// ========== VLM MODULATION ==========
// Apply VLM modulation to social contract if available
{
  std::lock_guard<std::mutex> lock(vlm_params_mutex_);
  if (latest_vlm_params_) {
    const auto& vlm = latest_vlm_params_;

    double original_v_max = contract.v_max;
    double original_w_social = contract.w_social;

    // 1. Speed modulation
    contract.v_max *= vlm->speed_scale;

    // 2. Distance-based social weight adjustment
    double distance_ratio = vlm->min_personal_distance / 1.0;  // 1.0m baseline
    contract.w_social *= distance_ratio;

    // 3. Emergency wait
    if (vlm->need_to_wait) {
      contract.v_max = 0.0;
      contract.w_social = 10.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "VLM requested stop: %s", vlm->explanation.c_str());
    }

    RCLCPP_DEBUG(get_logger(),
                 "VLM modulation: v_max %.2f->%.2f, w_social %.2f->%.2f (source: %s)",
                 original_v_max, contract.v_max,
                 original_w_social, contract.w_social,
                 vlm->source.c_str());
  }
}
// ========== END VLM MODULATION ==========
```

---

## Modification 6: Add VLM Cost Terms to evaluateSequence() (After line ~580)

**Location**: In `evaluateSequence()` method, after the smoothness cost, before terminal goal cost

**FIND** this code (around line 583-586):
```cpp
// Smoothness
cost += smooth_weight_ * (std::pow(v - prev_v, 2) + std::pow(w - prev_w, 2));
prev_v = v;
prev_w = w;
```

**ADD** these lines immediately after:
```cpp
// ========== VLM COST TERMS ==========
// Get VLM parameters (thread-safe)
social_mpc_nav::msg::VLMParameters::SharedPtr vlm_params;
{
  std::lock_guard<std::mutex> lock(vlm_params_mutex_);
  vlm_params = latest_vlm_params_;
}

if (vlm_params) {
  // Convert predicted state to helper format
  vlm_helpers::RobotState pred_state;
  pred_state.x = pred.x;
  pred_state.y = pred.y;
  pred_state.yaw = pred.yaw;

  // VLM directional preference cost
  cost += vlm_helpers::computeVLMDirectionalCost(
    pred_state, robot.x, robot.y, goal_x_, goal_y_,
    *vlm_params, w_vlm_directional_);

  // VLM action-based cost
  cost += vlm_helpers::computeVLMActionCost(
    v, pred_state, people, *vlm_params, w_vlm_action_);

  // VLM scene-specific cost
  cost += vlm_helpers::computeVLMSceneCost(
    v, pred_state, *vlm_params, w_vlm_scene_);
}
// ========== END VLM COST TERMS ==========
```

---

## Modification 7: Enhanced Social Cost with VLM Personal Distance (In evaluateSequence, around line 552-563)

**FIND** this code (social cost loop):
```cpp
// Social cost - repel from people
if (people)
{
  for (const auto & person : people->people)
  {
    const double dx = pred.x - static_cast<double>(person.x);
    const double dy = pred.y - static_cast<double>(person.y);
    const double dist = std::hypot(dx, dy);
    const double capped = std::max(dist, 0.1);
    cost += contract.w_social * (1.0 / (capped + eps));
  }
}
```

**REPLACE** with:
```cpp
// Social cost - repel from people (enhanced with VLM personal distance)
if (people)
{
  // Get VLM personal distance threshold
  double personal_distance_threshold = 1.0;  // Default
  social_mpc_nav::msg::VLMParameters::SharedPtr vlm_params;
  {
    std::lock_guard<std::mutex> lock(vlm_params_mutex_);
    vlm_params = latest_vlm_params_;
    if (vlm_params) {
      personal_distance_threshold = vlm_params->min_personal_distance;
    }
  }

  for (const auto & person : people->people)
  {
    const double dx = pred.x - static_cast<double>(person.x);
    const double dy = pred.y - static_cast<double>(person.y);
    const double dist = std::hypot(dx, dy);

    // Original inverse distance cost
    const double capped = std::max(dist, 0.1);
    cost += contract.w_social * (1.0 / (capped + eps));

    // VLM personal distance violation cost
    if (vlm_params) {
      cost += vlm_helpers::computeVLMPersonalDistanceCost(
        dist, *vlm_params, w_vlm_personal_);
    }
  }
}
```

---

## Modification 8: Add Member Variables (In private section, around line ~800)

**Location**: At the end of the private section, after existing member variables

```cpp
// ADD THESE LINES:
// ========== VLM Integration ==========
rclcpp::Subscription<social_mpc_nav::msg::VLMParameters>::SharedPtr vlm_params_sub_;
social_mpc_nav::msg::VLMParameters::SharedPtr latest_vlm_params_;
std::mutex vlm_params_mutex_;

// VLM cost weights
double w_vlm_directional_;
double w_vlm_action_;
double w_vlm_scene_;
double w_vlm_personal_;
// ========== END VLM Integration ==========
```

---

## Summary of Changes

1. **Includes**: Added VLM message and helpers
2. **Parameters**: 4 new VLM cost weight parameters
3. **Subscription**: Subscribe to `/vlm/mpc_parameters`
4. **Callback**: Handle VLM parameter updates
5. **Social Contract Modulation**: Apply speed_scale, distance, and emergency wait
6. **Cost Function**: 4 new VLM-specific cost terms
7. **Enhanced Social Cost**: Additional personal distance violation penalty
8. **Member Variables**: VLM subscription, cached params, mutex, cost weights

---

## Build After Modification

```bash
colcon build --packages-select social_mpc_nav
source install/setup.bash
```

---

## Testing

Run the VLM-enhanced MPC:
```bash
ros2 run social_mpc_nav mpc_controller_vlm_node --ros-args --params-file config/navigation_params.yaml
```

Compare with original simple MPC:
```bash
ros2 run social_mpc_nav mpc_controller_node --ros-args --params-file config/navigation_params.yaml
```

---

## Verification Checklist

- [ ] VLM parameters subscription created
- [ ] VLM callback receives messages
- [ ] Social contract modulation applied
- [ ] VLM cost terms added to objective
- [ ] Personal distance violations penalized
- [ ] Original MPC controller still works unchanged
- [ ] Both versions buildsuccessfully
