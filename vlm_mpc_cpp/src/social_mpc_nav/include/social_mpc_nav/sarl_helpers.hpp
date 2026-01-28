#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "social_mpc_nav/msg/sarl_output.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/msg/vlm_parameters.hpp"

namespace social_mpc_nav {
namespace sarl_helpers {

/**
 * @brief Scene-dependent multiplier for SARL cost weights.
 *
 * VLM scene_type and crowd_density condition how much we trust SARL's
 * attention-based reasoning. In constrained spaces (corridors, doorways),
 * per-person reasoning matters more. In open spaces, less so.
 */
struct SceneMultiplier
{
  double attention_mult;   // Multiplier for attention-weighted social cost
  double terminal_mult;    // Multiplier for SARL terminal value cost
};

inline SceneMultiplier getSceneMultiplier(
  const std::string & scene_type,
  const std::string & crowd_density)
{
  SceneMultiplier sm{1.0, 0.5};  // defaults

  if (scene_type == "doorway")
  {
    sm.attention_mult = 2.5;
    sm.terminal_mult = 1.2;
  }
  else if (scene_type == "corridor")
  {
    if (crowd_density == "dense") {
      sm.attention_mult = 2.0;
      sm.terminal_mult = 1.0;
    } else if (crowd_density == "medium") {
      sm.attention_mult = 1.5;
      sm.terminal_mult = 0.7;
    } else {
      sm.attention_mult = 1.5;
      sm.terminal_mult = 0.7;
    }
  }
  else if (scene_type == "crossing")
  {
    if (crowd_density == "dense" || crowd_density == "medium") {
      sm.attention_mult = 2.0;
      sm.terminal_mult = 1.0;
    } else {
      sm.attention_mult = 1.0;
      sm.terminal_mult = 0.5;
    }
  }
  else if (scene_type == "queue")
  {
    sm.attention_mult = 1.0;
    sm.terminal_mult = 0.5;
  }
  else if (scene_type == "open_space" || scene_type == "lobby")
  {
    if (crowd_density == "empty" || crowd_density == "sparse") {
      sm.attention_mult = 0.5;
      sm.terminal_mult = 0.3;
    } else {
      sm.attention_mult = 1.0;
      sm.terminal_mult = 0.5;
    }
  }
  // else: unknown scene, use defaults

  return sm;
}

/**
 * @brief Compute attention-weighted social cost for a predicted robot state.
 *
 * Replaces the uniform 1/distance social cost with attention-modulated cost.
 * Each person's cost is scaled by (N * attention_i) to preserve total magnitude
 * while redistributing focus based on learned importance.
 *
 * @param pred_x Predicted robot x position
 * @param pred_y Predicted robot y position
 * @param people Current people states
 * @param sarl SARL output with attention weights
 * @param w_social Base social cost weight (from SocialContract)
 * @param scene_mult Scene-dependent multiplier for SARL attention
 * @param eps Epsilon to avoid division by zero
 * @return Social cost value
 */
inline double computeAttentionWeightedSocialCost(
  double pred_x, double pred_y,
  const social_mpc_nav::msg::People2D & people,
  const social_mpc_nav::msg::SARLOutput & sarl,
  double w_social,
  double scene_mult,
  double eps = 0.1)
{
  if (people.people.empty()) {
    return 0.0;
  }

  double cost = 0.0;
  const size_t n_people = people.people.size();
  const size_t n_weights = sarl.attention_weights.size();

  for (size_t i = 0; i < n_people; ++i)
  {
    const auto & person = people.people[i];
    double dx = pred_x - static_cast<double>(person.x);
    double dy = pred_y - static_cast<double>(person.y);
    double dist = std::hypot(dx, dy);
    double capped = std::max(dist, 0.1);

    // Attention weight: use SARL weight if available, else uniform
    double attn_weight = 1.0 / static_cast<double>(n_people);
    if (i < n_weights) {
      attn_weight = static_cast<double>(sarl.attention_weights[i]);
    }

    // Scale by N to preserve total cost magnitude, then apply scene multiplier
    double scaled_weight = w_social * static_cast<double>(n_people)
                         * attn_weight * scene_mult;
    cost += scaled_weight * (1.0 / (capped + eps));
  }

  return cost;
}

/**
 * @brief Compute uniform social cost (fallback when SARL data is stale).
 */
inline double computeUniformSocialCost(
  double pred_x, double pred_y,
  const social_mpc_nav::msg::People2D & people,
  double w_social,
  double eps = 0.1)
{
  double cost = 0.0;
  for (const auto & person : people.people)
  {
    double dx = pred_x - static_cast<double>(person.x);
    double dy = pred_y - static_cast<double>(person.y);
    double dist = std::hypot(dx, dy);
    double capped = std::max(dist, 0.1);
    cost += w_social * (1.0 / (capped + eps));
  }
  return cost;
}

/**
 * @brief Compute SARL terminal value cost for a single rollout.
 *
 * Higher V(s) = better social state = lower cost. Negated so MPC minimizes.
 *
 * @param terminal_value V(s) from SARL for this rollout's terminal state
 * @param w_sarl_terminal Base terminal value weight
 * @param scene_mult Scene-dependent multiplier
 * @return Terminal cost contribution
 */
inline double computeSARLTerminalCost(
  double terminal_value,
  double w_sarl_terminal,
  double scene_mult)
{
  return -w_sarl_terminal * scene_mult * terminal_value;
}

}  // namespace sarl_helpers
}  // namespace social_mpc_nav
