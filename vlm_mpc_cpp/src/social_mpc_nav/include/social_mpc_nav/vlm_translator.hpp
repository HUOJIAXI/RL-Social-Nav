#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include "social_mpc_nav/msg/vlm_parameters.hpp"
#include "social_mpc_nav/msg/translator_status.hpp"

namespace social_mpc_nav
{

/**
 * @brief Simple JSON parser for VLM responses
 *
 * Parses the specific JSON schema from VLM without requiring external libraries.
 * Handles the expected fields: scene_type, crowd_density, recommended_action,
 * speed_scale, min_personal_distance, side_preference, need_to_wait, explanation
 */
class SimpleJSONParser
{
public:
  /**
   * @brief Extract a string value from JSON
   * @param json The JSON string
   * @param key The key to extract
   * @return The extracted string value, or empty string if not found
   */
  static std::string extractString(const std::string & json, const std::string & key);

  /**
   * @brief Extract a float value from JSON
   * @param json The JSON string
   * @param key The key to extract
   * @return The extracted float value, or 0.0 if not found
   */
  static float extractFloat(const std::string & json, const std::string & key);

  /**
   * @brief Extract a boolean value from JSON
   * @param json The JSON string
   * @param key The key to extract
   * @return The extracted boolean value, or false if not found
   */
  static bool extractBool(const std::string & json, const std::string & key);

private:
  /**
   * @brief Find the value substring for a given key in JSON
   */
  static std::string findValue(const std::string & json, const std::string & key);

  /**
   * @brief Unescape JSON string (handle \n, \", etc.)
   */
  static std::string unescapeString(const std::string & str);
};

/**
 * @brief VLM Translator Node
 *
 * Subscribes to VLM responses (/vlm/response) and translates them into
 * MPC parameters (/vlm/mpc_parameters). Provides fallback mechanisms,
 * validation, monitoring, and visualization.
 */
class VLMTranslatorNode : public rclcpp::Node
{
public:
  VLMTranslatorNode();
  ~VLMTranslatorNode() = default;

private:
  // ========== Callbacks ==========

  /**
   * @brief Callback for VLM responses
   * Parses JSON, validates, caches, and publishes translated parameters
   */
  void onVLMResponse(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Update robot pose from TF tree
   */
  void updateRobotPose();

  /**
   * @brief Timer callback to check VLM timeout
   * Switches to fallback if no VLM response received recently
   */
  void checkVLMTimeout();

  /**
   * @brief Timer callback to publish status
   */
  void publishStatus();

  /**
   * @brief Timer callback to publish visualization markers
   */
  void publishVisualization();

  // ========== Translation Logic ==========

  /**
   * @brief Translate VLM JSON response to VLMParameters message
   * @param json_response The JSON string from VLM
   * @return Translated parameters
   * @throws std::runtime_error if JSON parsing fails
   */
  msg::VLMParameters translateResponse(const std::string & json_response);

  /**
   * @brief Validate and clamp parameters to safe ranges
   * @param params Parameters to validate (modified in place)
   */
  void validateAndClamp(msg::VLMParameters & params);

  /**
   * @brief Get fallback parameters (cached or default)
   * @return Fallback parameters
   */
  msg::VLMParameters getFallbackParameters();

  /**
   * @brief Get default safe fallback parameters
   * @return Default parameters (no VLM modulation)
   */
  msg::VLMParameters getDefaultFallbackParameters();

  // ========== Logging ==========

  /**
   * @brief Log translation to CSV file
   * @param params The translated parameters
   * @param parsing_success Whether JSON parsing succeeded
   * @param latency_ms Translation latency in milliseconds
   */
  void logTranslation(
    const msg::VLMParameters & params,
    bool parsing_success,
    double latency_ms);

  /**
   * @brief Ensure CSV log file is open and has header
   */
  void ensureLogFile();

  // ========== Visualization ==========

  /**
   * @brief Create visualization markers for current VLM state
   * @param params Current VLM parameters
   * @return MarkerArray for RViz
   */
  visualization_msgs::msg::MarkerArray createVisualizationMarkers(
    const msg::VLMParameters & params);

  // ========== ROS Communication ==========

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vlm_response_sub_;

  // Publishers
  rclcpp::Publisher<msg::VLMParameters>::SharedPtr vlm_params_pub_;
  rclcpp::Publisher<msg::TranslatorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  rclcpp::TimerBase::SharedPtr timeout_check_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========== Parameters ==========

  std::string vlm_response_topic_;
  std::string vlm_parameters_topic_;
  std::string translator_status_topic_;
  std::string visualization_topic_;
  std::string map_frame_;
  std::string robot_base_frame_;

  double vlm_timeout_threshold_sec_;
  double cache_expiry_sec_;
  bool use_cached_fallback_;

  float min_speed_scale_;
  float max_speed_scale_;
  float min_personal_distance_;
  float max_personal_distance_;

  double status_publish_rate_hz_;
  double visualization_rate_hz_;

  std::string log_directory_;
  bool log_translation_to_csv_;
  bool enable_debug_logging_;

  // ========== State Management ==========

  std::mutex vlm_state_mutex_;
  std::optional<msg::VLMParameters> last_valid_vlm_params_;
  rclcpp::Time last_vlm_response_time_;
  std::atomic<bool> vlm_active_{false};

  // Statistics
  uint32_t total_responses_{0};
  uint32_t parsing_errors_{0};
  std::deque<double> response_latencies_;  // Rolling window

  // Robot pose (from TF, for visualization)
  std::mutex pose_mutex_;
  double current_robot_x_{0.0};
  double current_robot_y_{0.0};
  double current_robot_yaw_{0.0};
  bool pose_received_{false};

  // CSV Logging
  std::mutex log_mutex_;
  std::optional<std::ofstream> translation_log_;
};

}  // namespace social_mpc_nav
