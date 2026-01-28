#include "social_mpc_nav/vlm_translator.hpp"
#include <algorithm>
#include <iomanip>

namespace social_mpc_nav
{

// ========== SimpleJSONParser Implementation ==========

std::string SimpleJSONParser::extractString(const std::string & json, const std::string & key)
{
  std::string value_str = findValue(json, key);
  if (value_str.empty()) {
    return "";
  }

  // Remove quotes
  if (value_str.front() == '"' && value_str.back() == '"') {
    value_str = value_str.substr(1, value_str.length() - 2);
  }

  return unescapeString(value_str);
}

float SimpleJSONParser::extractFloat(const std::string & json, const std::string & key)
{
  std::string value_str = findValue(json, key);
  if (value_str.empty()) {
    return 0.0f;
  }

  try {
    return std::stof(value_str);
  } catch (...) {
    return 0.0f;
  }
}

bool SimpleJSONParser::extractBool(const std::string & json, const std::string & key)
{
  std::string value_str = findValue(json, key);
  return (value_str == "true");
}

std::string SimpleJSONParser::findValue(const std::string & json, const std::string & key)
{
  // Find key with quotes: "key":
  std::string search_pattern = "\"" + key + "\"";
  size_t key_pos = json.find(search_pattern);

  if (key_pos == std::string::npos) {
    return "";
  }

  // Find the colon after the key
  size_t colon_pos = json.find(':', key_pos);
  if (colon_pos == std::string::npos) {
    return "";
  }

  // Skip whitespace after colon
  size_t value_start = colon_pos + 1;
  while (value_start < json.length() && std::isspace(json[value_start])) {
    value_start++;
  }

  if (value_start >= json.length()) {
    return "";
  }

  // Determine value type and extract
  char first_char = json[value_start];

  if (first_char == '"') {
    // String value
    size_t value_end = value_start + 1;
    while (value_end < json.length()) {
      if (json[value_end] == '"' && json[value_end - 1] != '\\') {
        break;
      }
      value_end++;
    }
    return json.substr(value_start, value_end - value_start + 1);
  } else if (first_char == 't' || first_char == 'f') {
    // Boolean value
    if (json.substr(value_start, 4) == "true") {
      return "true";
    } else if (json.substr(value_start, 5) == "false") {
      return "false";
    }
  } else if (std::isdigit(first_char) || first_char == '-' || first_char == '.') {
    // Number value
    size_t value_end = value_start;
    while (value_end < json.length() &&
           (std::isdigit(json[value_end]) || json[value_end] == '.' ||
            json[value_end] == '-' || json[value_end] == 'e' || json[value_end] == 'E')) {
      value_end++;
    }
    return json.substr(value_start, value_end - value_start);
  }

  return "";
}

std::string SimpleJSONParser::unescapeString(const std::string & str)
{
  std::string result;
  result.reserve(str.length());

  for (size_t i = 0; i < str.length(); ++i) {
    if (str[i] == '\\' && i + 1 < str.length()) {
      char next = str[i + 1];
      if (next == 'n') {
        result += '\n';
        i++;
      } else if (next == 'r') {
        result += '\r';
        i++;
      } else if (next == 't') {
        result += '\t';
        i++;
      } else if (next == '"') {
        result += '"';
        i++;
      } else if (next == '\\') {
        result += '\\';
        i++;
      } else {
        result += str[i];
      }
    } else {
      result += str[i];
    }
  }

  return result;
}

// ========== VLMTranslatorNode Implementation ==========

VLMTranslatorNode::VLMTranslatorNode()
: Node("vlm_translator_node")
{
  // Declare parameters
  vlm_response_topic_ = declare_parameter<std::string>("vlm_response_topic", "/vlm/response");
  vlm_parameters_topic_ = declare_parameter<std::string>("vlm_parameters_topic", "/vlm/mpc_parameters");
  translator_status_topic_ = declare_parameter<std::string>("translator_status_topic", "/vlm/translator_status");
  visualization_topic_ = declare_parameter<std::string>("visualization_topic", "/vlm/visualization_markers");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "tiago_base/base_footprint");

  vlm_timeout_threshold_sec_ = declare_parameter<double>("vlm_timeout_threshold_sec", 5.0);
  cache_expiry_sec_ = declare_parameter<double>("cache_expiry_sec", 30.0);
  use_cached_fallback_ = declare_parameter<bool>("use_cached_fallback", true);

  min_speed_scale_ = declare_parameter<float>("min_speed_scale", 0.0);
  max_speed_scale_ = declare_parameter<float>("max_speed_scale", 1.0);
  min_personal_distance_ = declare_parameter<float>("min_personal_distance", 0.5);
  max_personal_distance_ = declare_parameter<float>("max_personal_distance", 2.0);

  status_publish_rate_hz_ = declare_parameter<double>("status_publish_rate_hz", 1.0);
  visualization_rate_hz_ = declare_parameter<double>("visualization_rate_hz", 2.0);

  log_directory_ = declare_parameter<std::string>("log_directory", "~/ros2_logs/social_mpc_nav");
  log_translation_to_csv_ = declare_parameter<bool>("log_translation_to_csv", true);
  enable_debug_logging_ = declare_parameter<bool>("enable_debug_logging", false);

  // Expand home directory
  if (log_directory_.front() == '~') {
    const char* home = std::getenv("HOME");
    if (home) {
      log_directory_ = std::string(home) + log_directory_.substr(1);
    }
  }

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Robot pose update timer (high frequency for smooth updates)
  pose_update_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&VLMTranslatorNode::updateRobotPose, this));

  // Create subscriptions
  vlm_response_sub_ = create_subscription<std_msgs::msg::String>(
    vlm_response_topic_, rclcpp::QoS(10),
    std::bind(&VLMTranslatorNode::onVLMResponse, this, std::placeholders::_1));

  // Create publishers
  vlm_params_pub_ = create_publisher<msg::VLMParameters>(
    vlm_parameters_topic_, rclcpp::QoS(10));

  status_pub_ = create_publisher<msg::TranslatorStatus>(
    translator_status_topic_, rclcpp::QoS(10));

  viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    visualization_topic_, rclcpp::QoS(10));

  // Create timers
  timeout_check_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&VLMTranslatorNode::checkVLMTimeout, this));

  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / status_publish_rate_hz_),
    std::bind(&VLMTranslatorNode::publishStatus, this));

  viz_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / visualization_rate_hz_),
    std::bind(&VLMTranslatorNode::publishVisualization, this));

  // Initialize last VLM response time to zero
  last_vlm_response_time_ = rclcpp::Time(0);

  RCLCPP_INFO(get_logger(), "VLM Translator Node initialized");
  RCLCPP_INFO(get_logger(), "  Subscribing to: %s", vlm_response_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  Publishing to: %s", vlm_parameters_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  VLM timeout threshold: %.1f seconds", vlm_timeout_threshold_sec_);
  RCLCPP_INFO(get_logger(), "  Cache expiry: %.1f seconds", cache_expiry_sec_);
  RCLCPP_INFO(get_logger(), "  Log directory: %s", log_directory_.c_str());
}

void VLMTranslatorNode::onVLMResponse(const std_msgs::msg::String::SharedPtr msg)
{
  auto start_time = now();

  // Skip heartbeat responses - they're not navigation JSON
  if (msg->data.find("[HEARTBEAT]") == 0) {
    RCLCPP_DEBUG(get_logger(), "Skipping heartbeat response (warmup/keep-alive)");
    return;
  }

  try {
    // Parse and translate
    auto params = translateResponse(msg->data);

    // Validate and clamp
    validateAndClamp(params);

    // Cache as valid
    {
      std::lock_guard<std::mutex> lock(vlm_state_mutex_);
      last_valid_vlm_params_ = params;
      last_vlm_response_time_ = now();
      vlm_active_ = true;
      total_responses_++;
    }

    // Publish
    vlm_params_pub_->publish(params);

    // Calculate latency
    double latency_ms = (now() - start_time).seconds() * 1000.0;
    response_latencies_.push_back(latency_ms);
    if (response_latencies_.size() > 100) {
      response_latencies_.pop_front();
    }

    // Log
    logTranslation(params, true, latency_ms);

    RCLCPP_INFO(get_logger(),
                "VLM translation successful: scene=%s, action=%s, speed_scale=%.2f, latency=%.1fms",
                params.scene_type.c_str(), params.recommended_action.c_str(),
                params.speed_scale, latency_ms);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "VLM parsing failed: %s", e.what());

    {
      std::lock_guard<std::mutex> lock(vlm_state_mutex_);
      parsing_errors_++;
    }

    // Publish fallback
    auto fallback = getFallbackParameters();
    vlm_params_pub_->publish(fallback);

    double latency_ms = (now() - start_time).seconds() * 1000.0;
    logTranslation(fallback, false, latency_ms);
  }
}

void VLMTranslatorNode::updateRobotPose()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);

  // Directly query TF tree for robot position in map frame
  try
  {
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(
        map_frame_,
        robot_base_frame_,
        rclcpp::Time(0));

    current_robot_x_ = transform_stamped.transform.translation.x;
    current_robot_y_ = transform_stamped.transform.translation.y;
    current_robot_yaw_ = tf2::getYaw(transform_stamped.transform.rotation);
    pose_received_ = true;

    RCLCPP_DEBUG(get_logger(),
      "Robot pose in map frame: (%.2f, %.2f, %.2f)",
      current_robot_x_, current_robot_y_, current_robot_yaw_);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Could not get robot pose from TF (map→%s): %s. Waiting for TF tree...",
      robot_base_frame_.c_str(), ex.what());
    pose_received_ = false;
  }
}

void VLMTranslatorNode::checkVLMTimeout()
{
  std::lock_guard<std::mutex> lock(vlm_state_mutex_);

  if (last_vlm_response_time_.nanoseconds() == 0) {
    return;  // Never received VLM response
  }

  double age = (now() - last_vlm_response_time_).seconds();

  if (age > vlm_timeout_threshold_sec_) {
    if (vlm_active_) {
      RCLCPP_WARN(get_logger(),
                  "VLM timeout: No response for %.1fs - switching to fallback",
                  age);
      vlm_active_ = false;
    }

    // Publish fallback parameters
    auto fallback = getFallbackParameters();
    vlm_params_pub_->publish(fallback);
  }
}

void VLMTranslatorNode::publishStatus()
{
  std::lock_guard<std::mutex> lock(vlm_state_mutex_);

  msg::TranslatorStatus status;
  status.stamp = now();
  status.vlm_active = vlm_active_;

  if (last_vlm_response_time_.nanoseconds() != 0) {
    status.vlm_last_response_age = (now() - last_vlm_response_time_).seconds();
  } else {
    status.vlm_last_response_age = -1.0;
  }

  status.vlm_responses_received = total_responses_;
  status.vlm_parsing_errors = parsing_errors_;
  status.using_fallback = !vlm_active_;
  status.fallback_reason = vlm_active_ ? "none" : "VLM timeout";

  // Calculate average latency
  if (!response_latencies_.empty()) {
    double sum = 0.0;
    for (double lat : response_latencies_) {
      sum += lat;
    }
    status.translation_latency_ms = sum / response_latencies_.size();
  }

  status_pub_->publish(status);
}

void VLMTranslatorNode::publishVisualization()
{
  std::lock_guard<std::mutex> lock(vlm_state_mutex_);

  // If VLM is inactive (fallback mode), visualize the fallback parameters being sent to MPC
  // Otherwise, visualize the cached VLM parameters
  msg::VLMParameters params_to_visualize;

  if (!vlm_active_) {
    // In fallback mode - show what the MPC is actually receiving
    params_to_visualize = getFallbackParameters();
  } else {
    // VLM active - show cached VLM output
    if (!last_valid_vlm_params_.has_value()) {
      return;  // No VLM data yet
    }
    params_to_visualize = last_valid_vlm_params_.value();
  }

  auto markers = createVisualizationMarkers(params_to_visualize);
  viz_pub_->publish(markers);
}

msg::VLMParameters VLMTranslatorNode::translateResponse(const std::string & json_response)
{
  msg::VLMParameters params;

  // Parse JSON fields
  params.scene_type = SimpleJSONParser::extractString(json_response, "scene_type");
  params.crowd_density = SimpleJSONParser::extractString(json_response, "crowd_density");
  params.recommended_action = SimpleJSONParser::extractString(json_response, "recommended_action");
  params.speed_scale = SimpleJSONParser::extractFloat(json_response, "speed_scale");
  params.min_personal_distance = SimpleJSONParser::extractFloat(json_response, "min_personal_distance");
  params.side_preference = SimpleJSONParser::extractString(json_response, "side_preference");
  params.need_to_wait = SimpleJSONParser::extractBool(json_response, "need_to_wait");
  params.explanation = SimpleJSONParser::extractString(json_response, "explanation");

  // Metadata
  params.stamp = now();
  params.source = "vlm";
  params.confidence = 1.0;
  params.age_sec = 0.0;

  // Validate required fields exist
  if (params.scene_type.empty() || params.recommended_action.empty()) {
    throw std::runtime_error("Missing required JSON fields");
  }

  return params;
}

void VLMTranslatorNode::validateAndClamp(msg::VLMParameters & params)
{
  float original_speed = params.speed_scale;
  float original_distance = params.min_personal_distance;

  params.speed_scale = std::clamp(params.speed_scale, min_speed_scale_, max_speed_scale_);
  params.min_personal_distance = std::clamp(params.min_personal_distance,
                                            min_personal_distance_, max_personal_distance_);

  if (params.speed_scale != original_speed) {
    RCLCPP_WARN(get_logger(), "Speed scale clamped from %.2f to %.2f",
                original_speed, params.speed_scale);
  }

  if (params.min_personal_distance != original_distance) {
    RCLCPP_WARN(get_logger(), "Personal distance clamped from %.2f to %.2f",
                original_distance, params.min_personal_distance);
  }

  // Compute adaptive soft obstacle distance based on crowd density and scene
  // Dense environment → smaller threshold (allows tighter navigation)
  // Sparse environment → larger threshold (maintains comfortable distance)
  double density_factor = 0.5;  // Default: medium
  if (params.crowd_density == "dense" || params.crowd_density == "very_dense") {
    density_factor = 0.0;  // Tightest navigation
  } else if (params.crowd_density == "medium") {
    density_factor = 0.5;
  } else if (params.crowd_density == "sparse" || params.crowd_density == "empty") {
    density_factor = 1.0;  // Maximum comfortable distance
  }

  // Scene factor (scene overrides if more restrictive)
  double scene_factor = 0.5;  // Default
  if (params.scene_type == "doorway" || params.scene_type == "crossing") {
    scene_factor = 0.0;  // Tight spaces require close navigation
  } else if (params.scene_type == "corridor" || params.scene_type == "lobby") {
    scene_factor = 0.3;
  } else if (params.scene_type == "open_space") {
    scene_factor = 1.0;
  }

  // Use minimum of density and scene factors (more conservative)
  double adaptive_factor = std::min(density_factor, scene_factor);

  // Interpolate between min (0.25m) and max (0.5m) thresholds
  params.adaptive_min_obstacle_distance = 0.25 + adaptive_factor * (0.5 - 0.25);

  RCLCPP_DEBUG(get_logger(), "Adaptive obstacle distance: %.2fm (density=%s, scene=%s, factor=%.2f)",
               params.adaptive_min_obstacle_distance, params.crowd_density.c_str(),
               params.scene_type.c_str(), adaptive_factor);
}

msg::VLMParameters VLMTranslatorNode::getFallbackParameters()
{
  // Option 1: Use cached last valid VLM response
  if (use_cached_fallback_ && last_valid_vlm_params_.has_value()) {
    auto params = last_valid_vlm_params_.value();
    params.source = "fallback_cached";
    params.age_sec = (now() - last_vlm_response_time_).seconds();

    // Age out old cache
    if (params.age_sec > cache_expiry_sec_) {
      RCLCPP_WARN(get_logger(), "Cached VLM params expired (%.1fs old), using defaults",
                  params.age_sec);
      return getDefaultFallbackParameters();
    }

    return params;
  }

  // Option 2: Use safe defaults
  return getDefaultFallbackParameters();
}

msg::VLMParameters VLMTranslatorNode::getDefaultFallbackParameters()
{
  msg::VLMParameters params;

  params.stamp = now();
  params.source = "fallback_rules";
  params.confidence = 0.5;
  params.age_sec = 0.0;

  // Conservative defaults (don't modulate MPC)
  params.speed_scale = 1.0;            // Full speed (no reduction)
  params.min_personal_distance = 1.0;  // Moderate distance
  params.side_preference = "neutral";
  params.need_to_wait = false;
  params.adaptive_min_obstacle_distance = 0.3;  // Default medium threshold

  params.scene_type = "unknown";
  params.crowd_density = "unknown";
  params.recommended_action = "go_ahead";
  params.explanation = "VLM unavailable - using rule-based fallback";

  return params;
}

void VLMTranslatorNode::logTranslation(
  const msg::VLMParameters & params,
  bool parsing_success,
  double latency_ms)
{
  if (!log_translation_to_csv_) {
    return;
  }

  std::lock_guard<std::mutex> lock(log_mutex_);
  ensureLogFile();

  double stamp_sec = params.stamp.sec + params.stamp.nanosec * 1e-9;

  *translation_log_ << std::fixed << std::setprecision(3)
                    << stamp_sec << ","
                    << params.source << ","
                    << params.confidence << ","
                    << params.age_sec << ","
                    << params.scene_type << ","
                    << params.crowd_density << ","
                    << params.recommended_action << ","
                    << params.speed_scale << ","
                    << params.min_personal_distance << ","
                    << params.side_preference << ","
                    << (params.need_to_wait ? 1 : 0) << ","
                    << (parsing_success ? 1 : 0) << ","
                    << latency_ms << ","
                    << "\"" << params.explanation << "\"\n";

  translation_log_->flush();
}

void VLMTranslatorNode::ensureLogFile()
{
  if (translation_log_) {
    return;
  }

  // Create directory if it doesn't exist
  std::filesystem::create_directories(log_directory_);

  // Open log file
  std::string log_path = log_directory_ + "/vlm_translation.csv";
  translation_log_.emplace(log_path);

  // Write header
  *translation_log_ << "stamp_sec,source,confidence,age_sec,scene_type,crowd_density,"
                    << "recommended_action,speed_scale,min_personal_distance,side_preference,"
                    << "need_to_wait,parsing_success,latency_ms,explanation\n";

  RCLCPP_INFO(get_logger(), "Created translation log: %s", log_path.c_str());
}

visualization_msgs::msg::MarkerArray VLMTranslatorNode::createVisualizationMarkers(
  const msg::VLMParameters & params)
{
  visualization_msgs::msg::MarkerArray markers;

  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;

  // Get robot position and orientation from cached pose (already in map frame)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (pose_received_) {
      robot_x = current_robot_x_;
      robot_y = current_robot_y_;
      robot_yaw = current_robot_yaw_;
    }
  }

  // Get VLM status for color coding
  bool is_vlm_active = vlm_active_.load();

  // Marker 0: VLM Status Indicator (sphere above robot)
  visualization_msgs::msg::Marker status_sphere;
  status_sphere.header.frame_id = "map";
  status_sphere.header.stamp = now();
  status_sphere.ns = "vlm_status";
  status_sphere.id = 0;
  status_sphere.type = visualization_msgs::msg::Marker::SPHERE;
  status_sphere.action = visualization_msgs::msg::Marker::ADD;
  status_sphere.pose.position.x = robot_x;
  status_sphere.pose.position.y = robot_y;
  status_sphere.pose.position.z = 2.5;
  status_sphere.scale.x = 0.3;
  status_sphere.scale.y = 0.3;
  status_sphere.scale.z = 0.3;
  // Color: Green if VLM active, Yellow if fallback
  if (is_vlm_active) {
    status_sphere.color.r = 0.0;
    status_sphere.color.g = 1.0;
    status_sphere.color.b = 0.0;
  } else {
    status_sphere.color.r = 1.0;
    status_sphere.color.g = 1.0;
    status_sphere.color.b = 0.0;
  }
  status_sphere.color.a = 1.0;
  markers.markers.push_back(status_sphere);

  // Marker 1: Enhanced text with all parameters
  visualization_msgs::msg::Marker info_text;
  info_text.header.frame_id = "map";
  info_text.header.stamp = now();
  info_text.ns = "vlm_info";
  info_text.id = 1;
  info_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  info_text.action = visualization_msgs::msg::Marker::ADD;
  info_text.pose.position.x = robot_x;
  info_text.pose.position.y = robot_y;
  info_text.pose.position.z = 2.0;
  info_text.scale.z = 0.2;
  info_text.color.r = 1.0;
  info_text.color.g = 1.0;
  info_text.color.b = 1.0;
  info_text.color.a = 1.0;

  // Build detailed info text
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "VLM: " << (is_vlm_active ? "ACTIVE" : "FALLBACK") << " (" << params.source << ")\n";
  ss << "Scene: " << params.scene_type << " | Density: " << params.crowd_density << "\n";
  ss << "Action: " << params.recommended_action << "\n";
  ss << "Speed: " << (params.speed_scale * 100.0) << "% | Distance: " << params.min_personal_distance << "m\n";
  ss << "Side: " << params.side_preference;
  if (params.need_to_wait) {
    ss << " | WAIT";
  }
  info_text.text = ss.str();
  markers.markers.push_back(info_text);

  // Marker 2: Personal space zone (circle around robot)
  visualization_msgs::msg::Marker personal_zone;
  personal_zone.header.frame_id = "map";
  personal_zone.header.stamp = now();
  personal_zone.ns = "vlm_personal_space";
  personal_zone.id = 2;
  personal_zone.type = visualization_msgs::msg::Marker::CYLINDER;
  personal_zone.action = visualization_msgs::msg::Marker::ADD;
  personal_zone.pose.position.x = robot_x;
  personal_zone.pose.position.y = robot_y;
  personal_zone.pose.position.z = 0.01;
  personal_zone.scale.x = params.min_personal_distance * 2.0;
  personal_zone.scale.y = params.min_personal_distance * 2.0;
  personal_zone.scale.z = 0.02;
  // Color based on VLM status
  if (is_vlm_active) {
    personal_zone.color.r = 0.0;
    personal_zone.color.g = 1.0;
    personal_zone.color.b = 1.0;
  } else {
    personal_zone.color.r = 1.0;
    personal_zone.color.g = 0.8;
    personal_zone.color.b = 0.0;
  }
  personal_zone.color.a = 0.3;
  markers.markers.push_back(personal_zone);

  // Marker 3: Speed scale bar (vertical bar showing speed scale)
  visualization_msgs::msg::Marker speed_bar;
  speed_bar.header.frame_id = "map";
  speed_bar.header.stamp = now();
  speed_bar.ns = "vlm_speed_scale";
  speed_bar.id = 3;
  speed_bar.type = visualization_msgs::msg::Marker::CUBE;
  speed_bar.action = visualization_msgs::msg::Marker::ADD;
  speed_bar.pose.position.x = robot_x - 0.8;
  speed_bar.pose.position.y = robot_y;
  speed_bar.pose.position.z = params.speed_scale * 1.0;  // Height represents speed
  speed_bar.scale.x = 0.1;
  speed_bar.scale.y = 0.1;
  speed_bar.scale.z = params.speed_scale * 2.0;  // Max 2m height at full speed
  // Color gradient: red (slow) to green (fast)
  speed_bar.color.r = 1.0 - params.speed_scale;
  speed_bar.color.g = params.speed_scale;
  speed_bar.color.b = 0.0;
  speed_bar.color.a = 0.8;
  markers.markers.push_back(speed_bar);

  // Marker 4: Side preference arrow (always displayed)
  {
    visualization_msgs::msg::Marker side_arrow;
    side_arrow.header.frame_id = "map";
    side_arrow.header.stamp = now();
    side_arrow.ns = "vlm_side_preference";
    side_arrow.id = 4;
    side_arrow.type = visualization_msgs::msg::Marker::ARROW;
    side_arrow.action = visualization_msgs::msg::Marker::ADD;

    // Position arrow to the side
    side_arrow.pose.position.x = robot_x;
    side_arrow.pose.position.y = robot_y;
    side_arrow.pose.position.z = 0.5;

    // Orientation based on side preference (relative to robot's heading)
    tf2::Quaternion q;
    double arrow_yaw = robot_yaw;  // Default: point forward (for neutral/none)
    if (params.side_preference == "left") {
      arrow_yaw = robot_yaw + M_PI / 2;  // Point to robot's left
    } else if (params.side_preference == "right") {
      arrow_yaw = robot_yaw - M_PI / 2;  // Point to robot's right
    }
    // neutral, none, or any other value: arrow_yaw = robot_yaw (straight ahead)
    q.setRPY(0, 0, arrow_yaw);
    side_arrow.pose.orientation.x = q.x();
    side_arrow.pose.orientation.y = q.y();
    side_arrow.pose.orientation.z = q.z();
    side_arrow.pose.orientation.w = q.w();

    side_arrow.scale.x = 0.5;  // Arrow length
    side_arrow.scale.y = 0.1;  // Arrow width
    side_arrow.scale.z = 0.1;  // Arrow height
    side_arrow.color.r = 1.0;
    side_arrow.color.g = 0.5;
    side_arrow.color.b = 0.0;
    side_arrow.color.a = 0.8;
    markers.markers.push_back(side_arrow);
  }

  // Marker 5: Wait indicator (red octagon if need to wait)
  if (params.need_to_wait) {
    visualization_msgs::msg::Marker wait_sign;
    wait_sign.header.frame_id = "map";
    wait_sign.header.stamp = now();
    wait_sign.ns = "vlm_wait_indicator";
    wait_sign.id = 5;
    wait_sign.type = visualization_msgs::msg::Marker::CYLINDER;
    wait_sign.action = visualization_msgs::msg::Marker::ADD;
    wait_sign.pose.position.x = robot_x + 0.8;
    wait_sign.pose.position.y = robot_y;
    wait_sign.pose.position.z = 1.0;
    wait_sign.scale.x = 0.4;
    wait_sign.scale.y = 0.4;
    wait_sign.scale.z = 0.05;
    wait_sign.color.r = 1.0;
    wait_sign.color.g = 0.0;
    wait_sign.color.b = 0.0;
    wait_sign.color.a = 0.9;
    markers.markers.push_back(wait_sign);

    // Add "WAIT" text
    visualization_msgs::msg::Marker wait_text;
    wait_text.header.frame_id = "map";
    wait_text.header.stamp = now();
    wait_text.ns = "vlm_wait_text";
    wait_text.id = 6;
    wait_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    wait_text.action = visualization_msgs::msg::Marker::ADD;
    wait_text.pose.position.x = robot_x + 0.8;
    wait_text.pose.position.y = robot_y;
    wait_text.pose.position.z = 1.0;
    wait_text.scale.z = 0.2;
    wait_text.color.r = 1.0;
    wait_text.color.g = 1.0;
    wait_text.color.b = 1.0;
    wait_text.color.a = 1.0;
    wait_text.text = "WAIT";
    markers.markers.push_back(wait_text);
  }

  return markers;
}

}  // namespace social_mpc_nav

// ========== Main Function ==========

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<social_mpc_nav::VLMTranslatorNode>());
  rclcpp::shutdown();
  return 0;
}
