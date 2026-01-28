#pragma once

#include <cmath>
#include <limits>
#include "social_mpc_nav/msg/vlm_parameters.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"

namespace social_mpc_nav
{

/**
 * @brief Helper functions for VLM-enhanced MPC cost computation
 *
 * These functions compute additional cost terms based on VLM semantic understanding.
 * They augment the base MPC objective function with scene-aware, action-based,
 * and directional preference costs.
 */
namespace vlm_helpers
{

/**
 * @brief Robot state for trajectory prediction
 */
struct RobotState
{
  double x;
  double y;
  double yaw;
};

/**
 * @brief Compute lateral deviation from preferred corridor side
 *
 * @param pred Predicted robot state
 * @param robot_x Current robot x position
 * @param robot_y Current robot y position
 * @param goal_x Goal x position
 * @param goal_y Goal y position
 * @param side_pref Side preference ("left", "right", or "neutral")
 * @return Lateral deviation cost (0.0 if neutral)
 */
inline double computeLateralDeviation(
  const RobotState & pred,
  double robot_x,
  double robot_y,
  double goal_x,
  double goal_y,
  const std::string & side_pref)
{
  if (side_pref == "neutral" || side_pref.empty()) {
    return 0.0;
  }

  // Corridor orientation from robot to goal
  double corridor_yaw = std::atan2(goal_y - robot_y, goal_x - robot_x);

  // Lateral offset (perpendicular to corridor axis)
  // Positive = right of corridor, negative = left of corridor
  double lateral = (pred.y - robot_y) * std::cos(corridor_yaw) -
                   (pred.x - robot_x) * std::sin(corridor_yaw);

  // Desired offset: left = -0.5m (negative = left), right = +0.5m (positive = right)
  double desired_offset = (side_pref == "left") ? -0.5 : 0.5;

  return std::abs(lateral - desired_offset);
}

/**
 * @brief Compute minimum frontal distance to people
 *
 * Checks if any person is in front of the robot (within 60-degree cone)
 * and returns the minimum distance.
 *
 * @param pred Predicted robot state
 * @param people Array of detected people
 * @return Minimum frontal distance (infinity if no people in front)
 */
inline double computeMinFrontalDistance(
  const RobotState & pred,
  const msg::People2D::SharedPtr & people)
{
  if (!people || people->people.empty()) {
    return std::numeric_limits<double>::max();
  }

  double min_dist = std::numeric_limits<double>::max();

  for (const auto & person : people->people) {
    double dx = person.x - pred.x;
    double dy = person.y - pred.y;
    double dist = std::hypot(dx, dy);

    // Check if person is in front (within 60-degree cone)
    double angle_to_person = std::atan2(dy, dx);
    double relative_angle = angle_to_person - pred.yaw;

    // Normalize to [-pi, pi]
    while (relative_angle > M_PI) relative_angle -= 2.0 * M_PI;
    while (relative_angle < -M_PI) relative_angle += 2.0 * M_PI;

    if (std::abs(relative_angle) < M_PI / 3.0) {  // 60-degree cone
      min_dist = std::min(min_dist, dist);
    }
  }

  return min_dist;
}

/**
 * @brief Check if predicted state is in crossing center (placeholder)
 *
 * This requires semantic map knowledge. For now, always returns false.
 * Future: Implement with semantic map or zone detection.
 *
 * @param pred Predicted robot state
 * @return true if in crossing center, false otherwise
 */
inline bool isInCrossingCenter(const RobotState & pred)
{
  // Placeholder: Requires semantic map
  (void)pred;  // Suppress unused warning
  return false;
}

/**
 * @brief Compute VLM directional preference cost
 *
 * Penalizes trajectories that deviate from VLM's recommended side preference.
 *
 * @param pred Predicted state
 * @param robot_x Current robot x
 * @param robot_y Current robot y
 * @param goal_x Goal x
 * @param goal_y Goal y
 * @param vlm VLM parameters
 * @param w_vlm_directional Weight for directional cost
 * @return Directional preference cost
 */
inline double computeVLMDirectionalCost(
  const RobotState & pred,
  double robot_x,
  double robot_y,
  double goal_x,
  double goal_y,
  const msg::VLMParameters & vlm,
  double w_vlm_directional)
{
  if (vlm.side_preference == "left" || vlm.side_preference == "right") {
    double lateral_dev = computeLateralDeviation(pred, robot_x, robot_y,
                                                  goal_x, goal_y, vlm.side_preference);
    return w_vlm_directional * lateral_dev * lateral_dev;
  }

  return 0.0;
}

/**
 * @brief Compute VLM action-based cost
 *
 * Penalizes actions inconsistent with VLM's recommended action.
 *
 * @param v Current linear velocity
 * @param pred Predicted state
 * @param people Detected people
 * @param vlm VLM parameters
 * @param w_vlm_action Weight for action cost
 * @return Action-based cost
 */
inline double computeVLMActionCost(
  double v,
  const RobotState & pred,
  const msg::People2D::SharedPtr & people,
  const msg::VLMParameters & vlm,
  double w_vlm_action)
{
  double cost = 0.0;

  if (vlm.recommended_action == "stop_and_wait") {
    // Penalize forward motion
    cost += w_vlm_action * v * v;

  } else if (vlm.recommended_action == "yield_to_pedestrian") {
    // Penalize being in front of nearest person
    double min_frontal_dist = computeMinFrontalDistance(pred, people);
    if (min_frontal_dist < 2.0) {  // Within 2m in front
      cost += w_vlm_action * (2.0 - min_frontal_dist);
    }
  }

  return cost;
}

/**
 * @brief Compute VLM scene-specific cost
 *
 * Applies scene-specific penalties based on VLM's scene understanding.
 *
 * @param v Current linear velocity
 * @param pred Predicted state
 * @param vlm VLM parameters
 * @param w_vlm_scene Weight for scene cost
 * @return Scene-specific cost
 */
inline double computeVLMSceneCost(
  double v,
  const RobotState & pred,
  const msg::VLMParameters & vlm,
  double w_vlm_scene)
{
  double cost = 0.0;

  if (vlm.scene_type == "doorway") {
    // Penalize stopping in doorways (maintain flow)
    if (v < 0.1) {  // Nearly stopped
      cost += w_vlm_scene * 10.0;
    }

  } else if (vlm.scene_type == "crossing") {
    // Penalize cutting through middle of crossing
    if (isInCrossingCenter(pred)) {
      cost += w_vlm_scene * 5.0;
    }
  }
  // Note: "queue" scene handled by speed_scale modulation, no explicit cost

  return cost;
}

/**
 * @brief Compute VLM personal distance violation cost
 *
 * Additional barrier cost for violating VLM-specified personal distance.
 *
 * @param dist Distance to person
 * @param vlm VLM parameters
 * @param w_vlm_personal Weight for personal distance violation
 * @return Personal distance violation cost
 */
inline double computeVLMPersonalDistanceCost(
  double dist,
  const msg::VLMParameters & vlm,
  double w_vlm_personal)
{
  double personal_distance_threshold = vlm.min_personal_distance;

  if (dist < personal_distance_threshold) {
    double violation = personal_distance_threshold - dist;
    return w_vlm_personal * violation * violation;
  }

  return 0.0;
}

}  // namespace vlm_helpers
}  // namespace social_mpc_nav
