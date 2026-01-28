#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <set>
#include <optional>
#include <map>
#include <limits>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// OpenCV and cv_bridge for image processing
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// libcurl for HTTP requests
#include <curl/curl.h>

// Using gb_visual_detection_3d_msgs for 3D bounding boxes
#include "gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"

// Person tracking information with motion data
#include "person_tracker/msg/person_info_array.hpp"
#include "person_tracker/msg/person_info.hpp"
#include "person_tracker/msg/human_cluster_array.hpp"
#include "person_tracker/msg/human_cluster.hpp"

// SARL attention weights for enriched VLM prompts
#include "social_mpc_nav/msg/sarl_output.hpp"

/**
 * @brief VLM Integration Node for Social MPC Navigation
 *
 * This node subscribes to 3D bounding boxes from darknet_ros_3d and triggers
 * VLM (Vision Language Model) calls based on events. The VLM is used to
 * analyze the scene and provide semantic understanding for navigation.
 *
 * Event triggers:
 * - New bounding box detected (configurable threshold)
 * - Periodic trigger (configurable interval)
 * - Significant scene change (configurable threshold)
 */
class VLMIntegrationNode : public rclcpp::Node
{
public:
  VLMIntegrationNode()
  : Node("vlm_integration_node")
  {
    // Parameters
    bounding_boxes_topic_ = declare_parameter<std::string>(
      "bounding_boxes_topic", "/darknet_ros_3d/bounding_boxes");
    person_info_topic_ = declare_parameter<std::string>(
      "person_info_topic", "/person_speed_tracker/person_info");
    human_clusters_topic_ = declare_parameter<std::string>(
      "human_clusters_topic", "/person_tracker/human_clusters");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/task_generator_node/tiago_official/odom");
    rgb_image_topic_ = declare_parameter<std::string>(
      "rgb_image_topic", "/task_generator_node/tiago_official/top_camera/image");
    vlm_prompt_topic_ = declare_parameter<std::string>(
      "vlm_prompt_topic", "/vlm/prompt");
    vlm_response_topic_ = declare_parameter<std::string>(
      "vlm_response_topic", "/vlm/response");

    // Image encoding parameters
    image_resize_width_ = declare_parameter<int>("image_resize_width", 640);
    image_resize_height_ = declare_parameter<int>("image_resize_height", 480);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 85);

    // Event trigger parameters
    trigger_on_new_detection_ = declare_parameter<bool>(
      "trigger_on_new_detection", true);
    trigger_periodic_ = declare_parameter<bool>(
      "trigger_periodic", false);
    periodic_interval_sec_ = declare_parameter<double>(
      "periodic_interval_sec", 5.0);
    trigger_on_scene_change_ = declare_parameter<bool>(
      "trigger_on_scene_change", true);
    scene_change_threshold_ = declare_parameter<double>(
      "scene_change_threshold", 0.3);  // Distance threshold in meters

    // VLM API parameters
    vlm_api_url_ = declare_parameter<std::string>(
      "vlm_api_url", "http://localhost:8000/v1/chat/completions");
    vlm_model_ = declare_parameter<std::string>(
      "vlm_model", "gpt-4-vision-preview");
    vlm_api_key_ = declare_parameter<std::string>(
      "vlm_api_key", "");
    enable_vlm_call_ = declare_parameter<bool>(
      "enable_vlm_call", false);  // Set to true when VLM API is ready

    // VLM heartbeat parameters - keeps model warm when idle
    enable_vlm_heartbeat_ = declare_parameter<bool>(
      "enable_vlm_heartbeat", true);  // Enable heartbeat to keep model warm
    vlm_heartbeat_interval_sec_ = declare_parameter<double>(
      "vlm_heartbeat_interval_sec", 1.0);  // Heartbeat check interval in seconds
    vlm_idle_threshold_sec_ = declare_parameter<double>(
      "vlm_idle_threshold_sec", 5.0);  // Send heartbeat if idle longer than this

    // Frame parameters
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    robot_base_frame_ = declare_parameter<std::string>(
      "robot_base_frame", "tiago_official/base_footprint");

    // Navigation goal parameters
    goal_x_ = declare_parameter<double>("goal_x", 0.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);

    // SARL attention integration parameters
    sarl_output_topic_ = declare_parameter<std::string>(
      "sarl_output_topic", "/sarl/output");
    trigger_on_sarl_attention_shift_ = declare_parameter<bool>(
      "trigger_on_sarl_attention_shift", true);
    sarl_attention_shift_threshold_ = declare_parameter<double>(
      "sarl_attention_shift_threshold", 0.3);

    // Detection filtering
    min_confidence_ = declare_parameter<double>("min_confidence", 0.5);
    max_detection_distance_ = declare_parameter<double>(
      "max_detection_distance", 10.0);  // meters
    heartbeat_interval_sec_ = declare_parameter<double>(
      "heartbeat_interval_sec", 5.0);  // seconds - publish heartbeat to keep topic alive

    // Advanced trigger conditions for significant scene changes
    trigger_on_new_object_type_ = declare_parameter<bool>(
      "trigger_on_new_object_type", true);  // Trigger when new object class appears
    trigger_on_object_count_change_ = declare_parameter<bool>(
      "trigger_on_object_count_change", false);  // Trigger when object count changes
    object_count_change_threshold_ = declare_parameter<int>(
      "object_count_change_threshold", 2);  // Minimum change in object count to trigger
    
    trigger_on_object_position_change_ = declare_parameter<bool>(
      "trigger_on_object_position_change", true);  // Trigger when objects move significantly
    object_position_change_threshold_ = declare_parameter<double>(
      "object_position_change_threshold", 1.0);  // meters - minimum movement to trigger
    
    trigger_on_object_type_combination_change_ = declare_parameter<bool>(
      "trigger_on_object_type_combination_change", true);  // Trigger when object type mix changes

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    bbox_sub_ = create_subscription<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
      bounding_boxes_topic_, rclcpp::QoS(10),
      std::bind(&VLMIntegrationNode::onBoundingBoxes, this, std::placeholders::_1));

    person_info_sub_ = create_subscription<person_tracker::msg::PersonInfoArray>(
      person_info_topic_, rclcpp::QoS(10),
      std::bind(&VLMIntegrationNode::onPersonInfo, this, std::placeholders::_1));

    human_clusters_sub_ = create_subscription<person_tracker::msg::HumanClusterArray>(
      human_clusters_topic_, rclcpp::QoS(10),
      std::bind(&VLMIntegrationNode::onHumanClusters, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&VLMIntegrationNode::onOdom, this, std::placeholders::_1));

    // Subscribe to SARL attention weights for enriched prompts
    sarl_sub_ = create_subscription<social_mpc_nav::msg::SARLOutput>(
      sarl_output_topic_, rclcpp::QoS(10),
      std::bind(&VLMIntegrationNode::onSARLOutput, this, std::placeholders::_1));

    // Subscribe to global path to get next waypoint
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(10),
      std::bind(&VLMIntegrationNode::onGlobalPath, this, std::placeholders::_1));

    // Subscribe to RGB image for VLM
    // Use QoS(10) with RELIABLE to match the publisher's QoS
    auto image_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_image_topic_, image_qos,
      std::bind(&VLMIntegrationNode::onImage, this, std::placeholders::_1));

    // Initialize curl globally
    curl_global_init(CURL_GLOBAL_ALL);
    vlm_request_in_progress_ = false;

    // Publishers
    prompt_pub_ = create_publisher<std_msgs::msg::String>(
      vlm_prompt_topic_, rclcpp::QoS(10));
    response_pub_ = create_publisher<std_msgs::msg::String>(
      vlm_response_topic_, rclcpp::QoS(10));

    // Use a timer to publish initial message after a short delay
    // This ensures the publisher is fully registered before publishing
    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),  // 500ms delay
      [this]() {
        std_msgs::msg::String init_msg;
        init_msg.data = "[VLM Integration Node initialized - camera ready, warmup scheduled]";
        prompt_pub_->publish(init_msg);
        RCLCPP_INFO(get_logger(), "Published initial message to %s", vlm_prompt_topic_.c_str());
        // Stop the timer after first execution
        init_timer_->cancel();
      });

    // Also publish immediately (in case timer doesn't fire fast enough)
    // This helps ensure topic appears in ros2 topic list
    std_msgs::msg::String immediate_msg;
    immediate_msg.data = "[VLM Integration Node starting up...]";
    prompt_pub_->publish(immediate_msg);

    // Heartbeat timer to keep topic alive and visible
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration<double>(heartbeat_interval_sec_),
      [this]() {
        std_msgs::msg::String heartbeat_msg;
        heartbeat_msg.data = "[VLM Integration Node heartbeat - ready for scene analysis]";
        prompt_pub_->publish(heartbeat_msg);
        RCLCPP_DEBUG(get_logger(), "Published heartbeat to %s", vlm_prompt_topic_.c_str());
      });

    // Periodic trigger timer
    if (trigger_periodic_)
    {
      periodic_timer_ = create_wall_timer(
        std::chrono::duration<double>(periodic_interval_sec_),
        std::bind(&VLMIntegrationNode::onPeriodicTrigger, this));
    }

    // VLM heartbeat timer - keeps model warm when idle
    last_vlm_call_time_ = now();
    if (enable_vlm_heartbeat_ && enable_vlm_call_)
    {
      vlm_heartbeat_timer_ = create_wall_timer(
        std::chrono::duration<double>(vlm_heartbeat_interval_sec_),
        std::bind(&VLMIntegrationNode::onVLMHeartbeat, this));
      RCLCPP_INFO(get_logger(),
        "VLM heartbeat enabled: interval=%.1fs, idle_threshold=%.1fs",
        vlm_heartbeat_interval_sec_, vlm_idle_threshold_sec_);

      // Startup warmup timer - sends first VLM request after camera initializes
      // This warms up the VLM model before navigation begins
      warmup_timer_ = create_wall_timer(
        std::chrono::milliseconds(3000),  // 3 second delay to allow camera to start
        [this]() {
          RCLCPP_INFO(get_logger(), "⏳ Sending VLM startup warmup request...");
          sendVLMHeartbeat();  // Reuse heartbeat function for warmup
          warmup_timer_->cancel();  // One-time only
        });
      RCLCPP_INFO(get_logger(),
        "⏳ VLM startup warmup scheduled in 3 seconds (waiting for camera)...");

      // Initial navigation warmup - sends full navigation prompt to unlock MPC
      // Even with empty scene, this allows MPC to start
      navigation_warmup_timer_ = create_wall_timer(
        std::chrono::milliseconds(6000),  // 6 second delay (after heartbeat warmup)
        [this]() {
          RCLCPP_INFO(get_logger(), "🚀 Sending initial navigation warmup (empty scene allowed)...");
          // Force trigger even with empty bboxes
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (latest_bboxes_.empty()) {
            RCLCPP_INFO(get_logger(), "   No people detected - sending empty scene navigation prompt");
            // Create empty bbox list to trigger navigation prompt
            std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> empty_boxes;
            latest_bboxes_ = empty_boxes;
          }
          triggerVLMQueryImpl();  // Send navigation prompt
          navigation_warmup_timer_->cancel();  // One-time only
        });
      RCLCPP_INFO(get_logger(),
        "🚀 Initial navigation warmup scheduled in 6 seconds...");
    }

    // Initialize VLM inference time CSV logging
    const char* home_dir = std::getenv("HOME");
    if (home_dir == nullptr) {
      home_dir = "/tmp";
    }

    // Generate timestamp for log file name
    auto now_time = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now_time);
    std::stringstream timestamp_ss;
    timestamp_ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    std::string timestamp_str = timestamp_ss.str();

    vlm_inference_log_file_ = std::string(home_dir) + "/ros2_logs/social_mpc_nav/vlm_inference_times_" + timestamp_str + ".csv";

    // Create directory if it doesn't exist
    std::string log_dir = std::string(home_dir) + "/ros2_logs/social_mpc_nav";
    std::string mkdir_cmd = "mkdir -p " + log_dir;
    int ret = system(mkdir_cmd.c_str());
    (void)ret;  // Suppress unused variable warning

    vlm_inference_csv_.open(vlm_inference_log_file_, std::ios::out);
    if (vlm_inference_csv_.is_open()) {
      // Write CSV header
      vlm_inference_csv_ << "timestamp_sec,inference_time_ms,trigger_type\n";
      vlm_inference_csv_.flush();
      RCLCPP_INFO(get_logger(), "VLM inference time logging enabled: %s", vlm_inference_log_file_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open VLM inference time log file: %s", vlm_inference_log_file_.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "vlm_integration_node started. Subscribing to %s, %s, and %s",
      bounding_boxes_topic_.c_str(), person_info_topic_.c_str(), human_clusters_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "RGB image topic: %s", rgb_image_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Publishing prompts to: %s", vlm_prompt_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Publishing responses to: %s", vlm_response_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "VLM API URL: %s, Model: %s, Enabled: %s",
      vlm_api_url_.c_str(), vlm_model_.c_str(), enable_vlm_call_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "Event triggers: new_detection=%d, periodic=%d, scene_change=%d",
      trigger_on_new_detection_, trigger_periodic_, trigger_on_scene_change_);
    RCLCPP_INFO(
      get_logger(),
      "Advanced triggers: new_type=%d, count_change=%d, position_change=%d, "
      "type_combination=%d",
      trigger_on_new_object_type_, trigger_on_object_count_change_,
      trigger_on_object_position_change_, trigger_on_object_type_combination_change_);
  }

  ~VLMIntegrationNode()
  {
    // Close CSV file if open
    if (vlm_inference_csv_.is_open())
    {
      vlm_inference_csv_.close();
      RCLCPP_INFO(get_logger(), "VLM inference time log closed: %s", vlm_inference_log_file_.c_str());
    }

    // Cleanup curl
    curl_global_cleanup();
  }

private:
  // Image callback
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = msg;
    RCLCPP_DEBUG(get_logger(), "Received image: %dx%d, encoding: %s",
                 msg->width, msg->height, msg->encoding.c_str());
  }

  // Base64 encoding lookup table
  static const std::string base64_chars;

  // Base64 encode function
  std::string base64Encode(const std::vector<unsigned char>& data)
  {
    std::string encoded;
    int i = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    size_t data_len = data.size();
    const unsigned char* bytes_to_encode = data.data();

    while (data_len--)
    {
      char_array_3[i++] = *(bytes_to_encode++);
      if (i == 3)
      {
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (i = 0; i < 4; i++)
          encoded += base64_chars[char_array_4[i]];
        i = 0;
      }
    }

    if (i)
    {
      for (int j = i; j < 3; j++)
        char_array_3[j] = '\0';

      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

      for (int j = 0; j < i + 1; j++)
        encoded += base64_chars[char_array_4[j]];

      while (i++ < 3)
        encoded += '=';
    }

    return encoded;
  }

  // Get current image as base64-encoded JPEG
  std::string getImageBase64()
  {
    std::lock_guard<std::mutex> lock(image_mutex_);

    if (!latest_image_)
    {
      RCLCPP_WARN(get_logger(), "No image available for VLM request");
      return "";
    }

    try
    {
      // Convert ROS image to OpenCV
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(latest_image_, "bgr8");
      cv::Mat image = cv_ptr->image;

      // Resize if needed
      if (image_resize_width_ > 0 && image_resize_height_ > 0 &&
          (image.cols != image_resize_width_ || image.rows != image_resize_height_))
      {
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(image_resize_width_, image_resize_height_));
        image = resized;
      }

      // Encode to JPEG
      std::vector<unsigned char> jpeg_buffer;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      cv::imencode(".jpg", image, jpeg_buffer, params);

      // Convert to base64
      std::string base64_image = base64Encode(jpeg_buffer);

      RCLCPP_DEBUG(get_logger(), "Encoded image to base64: %zu bytes -> %zu chars",
                   jpeg_buffer.size(), base64_image.size());

      return base64_image;
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return "";
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Exception encoding image: %s", e.what());
      return "";
    }
  }

  // Curl write callback
  static size_t curlWriteCallback(void* contents, size_t size, size_t nmemb, std::string* output)
  {
    size_t total_size = size * nmemb;
    output->append(static_cast<char*>(contents), total_size);
    return total_size;
  }

  void onBoundingBoxes(
    const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d::SharedPtr msg)
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Received bounding boxes message with %zu boxes",
      msg->bounding_boxes.size());

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Filter bounding boxes by confidence and distance
    std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> filtered_boxes;
    for (const auto & bbox : msg->bounding_boxes)
    {
      if (bbox.probability < min_confidence_)
      {
        RCLCPP_DEBUG(
          get_logger(),
          "Filtered out %s (confidence %.2f < %.2f)",
          bbox.object_name.c_str(), bbox.probability, min_confidence_);
        continue;
      }

      // Calculate center and size from bounding box coordinates
      double center_x = (bbox.xmin + bbox.xmax) / 2.0;
      double center_y = (bbox.ymin + bbox.ymax) / 2.0;
      double center_z = (bbox.zmin + bbox.zmax) / 2.0;

      // Calculate distance from robot to bounding box center
      double distance = std::hypot(center_x, center_y, center_z);
      
      if (distance > max_detection_distance_)
      {
        RCLCPP_DEBUG(
          get_logger(),
          "Filtered out %s (distance %.2f > %.2f)",
          bbox.object_name.c_str(), distance, max_detection_distance_);
        continue;
      }

      filtered_boxes.push_back(bbox);
    }

    if (filtered_boxes.empty())
    {
      RCLCPP_DEBUG(
        get_logger(),
        "No bounding boxes passed filtering (received %zu, filtered: confidence < %.2f or distance > %.2f)",
        msg->bounding_boxes.size(), min_confidence_, max_detection_distance_);
      return;
    }

    // Count persons before and after filtering for debugging
    int total_persons = 0;
    int filtered_persons = 0;
    for (const auto & bbox : msg->bounding_boxes) {
      if (bbox.object_name == "person") total_persons++;
    }
    for (const auto & bbox : filtered_boxes) {
      if (bbox.object_name == "person") filtered_persons++;
    }

    RCLCPP_DEBUG(
      get_logger(),
      "👥 Person detection: %d total → %d after filtering (confidence≥%.2f, distance≤%.2fm)",
      total_persons, filtered_persons, min_confidence_, max_detection_distance_);

    RCLCPP_DEBUG(
      get_logger(),
      "Processing %zu filtered bounding boxes (from %zu total)",
      filtered_boxes.size(), msg->bounding_boxes.size());

    // Update latest bounding boxes FIRST before checking triggers
    // This ensures triggerVLMQuery() has access to the latest data
    latest_bboxes_ = filtered_boxes;
    latest_bbox_timestamp_ = msg->header.stamp;

    // Check for new detections (event trigger)
    bool has_new_detection = false;
    if (trigger_on_new_detection_)
    {
      has_new_detection = detectNewBoundingBoxes(filtered_boxes);
    }

    // Check for scene change (event trigger)
    bool scene_changed = false;
    if (trigger_on_scene_change_)
    {
      scene_changed = detectSceneChange(filtered_boxes);
    }

    // Trigger VLM call if event conditions are met
    if (has_new_detection || scene_changed)
    {
      RCLCPP_INFO(
        get_logger(),
        "Event triggered: new_detection=%d, scene_changed=%d | "
        "Filtered %zu bounding boxes",
        has_new_detection, scene_changed, filtered_boxes.size());

      // Call internal implementation directly since we already hold the lock
      triggerVLMQueryImpl();
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_ = msg;
  }

  void onPersonInfo(const person_tracker::msg::PersonInfoArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_person_info_ = msg;

    RCLCPP_DEBUG(
      get_logger(),
      "Received person info with %d persons",
      msg->num_persons);
  }

  void onSARLOutput(const social_mpc_nav::msg::SARLOutput::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Check for attention shift: if the most-attended person changed, trigger VLM
    if (trigger_on_sarl_attention_shift_ && msg->is_valid &&
        !msg->attention_weights.empty() && latest_sarl_output_ &&
        latest_sarl_output_->is_valid &&
        !latest_sarl_output_->attention_weights.empty())
    {
      // Find index of max attention in old and new
      auto old_max_it = std::max_element(
        latest_sarl_output_->attention_weights.begin(),
        latest_sarl_output_->attention_weights.end());
      auto new_max_it = std::max_element(
        msg->attention_weights.begin(),
        msg->attention_weights.end());

      size_t old_max_idx = std::distance(
        latest_sarl_output_->attention_weights.begin(), old_max_it);
      size_t new_max_idx = std::distance(
        msg->attention_weights.begin(), new_max_it);

      // Trigger if most-attended person changed or attention shifted significantly
      bool attention_shifted = (old_max_idx != new_max_idx);
      if (!attention_shifted && old_max_idx < msg->attention_weights.size())
      {
        double delta = std::abs(
          msg->attention_weights[old_max_idx] -
          latest_sarl_output_->attention_weights[old_max_idx]);
        attention_shifted = (delta > sarl_attention_shift_threshold_);
      }

      if (attention_shifted && !vlm_request_in_progress_.load() &&
          !latest_bboxes_.empty())
      {
        RCLCPP_INFO(get_logger(),
          "SARL attention shift detected (person %zu -> %zu), triggering VLM",
          old_max_idx, new_max_idx);
        triggerVLMQueryImpl();
      }
    }

    latest_sarl_output_ = msg;
  }

  void onHumanClusters(const person_tracker::msg::HumanClusterArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_human_clusters_ = msg;

    RCLCPP_DEBUG(
      get_logger(),
      "Received human clusters: %d clusters",
      msg->num_clusters);
  }

  void onGlobalPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_global_path_ = msg;

    RCLCPP_DEBUG(
      get_logger(),
      "Received global path with %zu waypoints",
      msg->poses.size());
  }

  void onPeriodicTrigger()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!latest_bboxes_.empty())
    {
      RCLCPP_INFO(
        get_logger(),
        "Periodic trigger: Processing %zu bounding boxes",
        latest_bboxes_.size());

      // Call internal implementation directly since we already hold the lock
      triggerVLMQueryImpl();
    }
  }

  // VLM heartbeat callback - sends simple prompt to keep model warm
  void onVLMHeartbeat()
  {
    // Skip if VLM call is disabled
    if (!enable_vlm_call_)
    {
      return;
    }

    // Skip if a request is already in progress
    if (vlm_request_in_progress_.load())
    {
      RCLCPP_DEBUG(get_logger(), "VLM heartbeat skipped - request in progress");
      return;
    }

    // Check if we've been idle long enough to need a heartbeat
    rclcpp::Time current_time = now();
    double idle_time = (current_time - last_vlm_call_time_).seconds();

    if (idle_time < vlm_idle_threshold_sec_)
    {
      RCLCPP_DEBUG(get_logger(),
        "VLM heartbeat not needed - idle time %.2fs < threshold %.2fs",
        idle_time, vlm_idle_threshold_sec_);
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Sending VLM heartbeat - model idle for %.2fs", idle_time);

    // Send a simple heartbeat prompt (no image to save bandwidth)
    sendVLMHeartbeat();
  }

  // Send a warmup/heartbeat request to keep VLM model warm
  // Uses current camera image + simple descriptive prompt
  void sendVLMHeartbeat()
  {
    // Mark as in progress to prevent overlapping heartbeats
    vlm_request_in_progress_ = true;

    // Capture VLM inference start time for heartbeat
    vlm_inference_start_time_ = now();

    // Get the base64-encoded image (same as regular VLM requests)
    std::string base64_image = getImageBase64();
    if (base64_image.empty())
    {
      RCLCPP_WARN(get_logger(), "No camera image available for VLM warmup/heartbeat, skipping");
      vlm_request_in_progress_ = false;  // Reset flag
      return;
    }

    // Simple descriptive prompt - verifies VLM is working and keeps it warm
    std::string heartbeat_prompt = "Describe the scene in 10 words.";

    // Build JSON payload with image + text (same format as regular VLM requests)
    std::ostringstream json_payload;
    json_payload << "{"
                 << "\"model\": \"" << vlm_model_ << "\","
                 << "\"messages\": [{"
                 << "\"role\": \"user\","
                 << "\"content\": ["
                 << "{"
                 << "\"type\": \"image_url\","
                 << "\"image_url\": {"
                 << "\"url\": \"data:image/jpeg;base64," << base64_image << "\""
                 << "}"
                 << "},"
                 << "{"
                 << "\"type\": \"text\","
                 << "\"text\": \"" << heartbeat_prompt << "\""
                 << "}"
                 << "]"
                 << "}]"
                 << "}";

    std::string payload = json_payload.str();

    RCLCPP_INFO(get_logger(),
      "VLM warmup/heartbeat: sending image + prompt (\"%s\") - payload: %zu bytes",
      heartbeat_prompt.c_str(), payload.length());

    // Launch async HTTP request in a separate thread
    std::thread([this, payload]()
    {
      std::string response_body;

      CURL* curl = curl_easy_init();
      if (curl)
      {
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        if (!vlm_api_key_.empty())
        {
          std::string auth_header = "Authorization: Bearer " + vlm_api_key_;
          headers = curl_slist_append(headers, auth_header.c_str());
        }

        curl_easy_setopt(curl, CURLOPT_URL, vlm_api_url_.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, payload.length());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);  // Shorter timeout for heartbeat
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);

        CURLcode res = curl_easy_perform(curl);

        if (res == CURLE_OK)
        {
          long http_code = 0;
          curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

          if (http_code == 200)
          {
            RCLCPP_INFO(rclcpp::get_logger("vlm_integration_node"),
              "✅ VLM heartbeat successful - model is warm (response: %zu bytes)",
              response_body.length());

            // Calculate and log VLM inference time for heartbeat
            auto end_time = this->now();
            double inference_time_ms = (end_time - vlm_inference_start_time_).seconds() * 1000.0;
            logVLMInferenceTime(inference_time_ms, "heartbeat");

            // Publish heartbeat response with special prefix for debugging/verification
            std_msgs::msg::String heartbeat_response;
            heartbeat_response.data = "[HEARTBEAT] " + response_body;
            response_pub_->publish(heartbeat_response);

            RCLCPP_DEBUG(rclcpp::get_logger("vlm_integration_node"),
              "Published heartbeat response: %s",
              response_body.substr(0, std::min(response_body.length(), size_t(100))).c_str());
          }
          else
          {
            RCLCPP_WARN(rclcpp::get_logger("vlm_integration_node"),
              "VLM heartbeat returned HTTP %ld: %s", http_code, response_body.c_str());
          }
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("vlm_integration_node"),
            "VLM heartbeat failed: %s", curl_easy_strerror(res));
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
      }

      // Mark request as complete
      vlm_request_in_progress_ = false;

    }).detach();

    // Update last call time immediately to prevent rapid consecutive heartbeats
    last_vlm_call_time_ = now();
  }

  bool detectNewBoundingBoxes(
    const std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> & current_boxes)
  {
    if (previous_bboxes_.empty())
    {
      previous_bboxes_ = current_boxes;
      return !current_boxes.empty();  // First detection counts as new
    }

    bool has_significant_change = false;
    std::string change_reason;

    // 1. Check for new object types (most significant)
    if (trigger_on_new_object_type_)
    {
      std::set<std::string> previous_classes;
      for (const auto & bbox : previous_bboxes_)
      {
        previous_classes.insert(bbox.object_name);
      }

      std::set<std::string> current_classes;
      for (const auto & bbox : current_boxes)
      {
        current_classes.insert(bbox.object_name);
        if (previous_classes.find(bbox.object_name) == previous_classes.end())
        {
          has_significant_change = true;
          change_reason = "new object type: " + bbox.object_name;
          break;
        }
      }

      // Also check if object types disappeared (scene simplification)
      for (const auto & prev_class : previous_classes)
      {
        if (current_classes.find(prev_class) == current_classes.end())
        {
          has_significant_change = true;
          change_reason = "object type disappeared: " + prev_class;
          break;
        }
      }
    }

    // 2. Check for significant object count change
    if (!has_significant_change && trigger_on_object_count_change_)
    {
      int count_diff = std::abs(
        static_cast<int>(current_boxes.size()) -
        static_cast<int>(previous_bboxes_.size()));
      
      if (count_diff >= object_count_change_threshold_)
      {
        has_significant_change = true;
        change_reason = "object count changed by " + std::to_string(count_diff);
      }
    }

    // 3. Check for object type combination change
    if (!has_significant_change && trigger_on_object_type_combination_change_)
    {
      std::set<std::string> previous_classes;
      std::set<std::string> current_classes;
      
      for (const auto & bbox : previous_bboxes_)
      {
        previous_classes.insert(bbox.object_name);
      }
      for (const auto & bbox : current_boxes)
      {
        current_classes.insert(bbox.object_name);
      }

      // Check if the combination of types changed (even if count is same)
      if (previous_classes != current_classes)
      {
        has_significant_change = true;
        change_reason = "object type combination changed";
      }
    }

    // 4. Check for significant object position changes
    if (!has_significant_change && trigger_on_object_position_change_)
    {
      // Match objects by type and check if they moved significantly
      std::map<std::string, std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d>> prev_by_type;
      std::map<std::string, std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d>> curr_by_type;

      for (const auto & bbox : previous_bboxes_)
      {
        prev_by_type[bbox.object_name].push_back(bbox);
      }
      for (const auto & bbox : current_boxes)
      {
        curr_by_type[bbox.object_name].push_back(bbox);
      }

      // For each object type, check if positions changed significantly
      for (const auto & [obj_type, prev_boxes] : prev_by_type)
      {
        if (curr_by_type.find(obj_type) == curr_by_type.end())
        {
          continue;  // Type disappeared, already handled above
        }

        const auto & curr_boxes = curr_by_type[obj_type];
        
        // Simple matching: find nearest previous box for each current box
        for (const auto & curr_bbox : curr_boxes)
        {
          double min_dist = std::numeric_limits<double>::infinity();
          
          for (const auto & prev_bbox : prev_boxes)
          {
            double prev_center_x = (prev_bbox.xmin + prev_bbox.xmax) / 2.0;
            double prev_center_y = (prev_bbox.ymin + prev_bbox.ymax) / 2.0;
            double prev_center_z = (prev_bbox.zmin + prev_bbox.zmax) / 2.0;
            
            double curr_center_x = (curr_bbox.xmin + curr_bbox.xmax) / 2.0;
            double curr_center_y = (curr_bbox.ymin + curr_bbox.ymax) / 2.0;
            double curr_center_z = (curr_bbox.zmin + curr_bbox.zmax) / 2.0;
            
            double dist = std::hypot(
              curr_center_x - prev_center_x,
              curr_center_y - prev_center_y,
              curr_center_z - prev_center_z);
            
            min_dist = std::min(min_dist, dist);
          }

          if (min_dist > object_position_change_threshold_)
          {
            has_significant_change = true;
            change_reason = "object '" + obj_type + "' moved " + 
                          std::to_string(min_dist) + " meters";
            break;
          }
        }

        if (has_significant_change)
        {
          break;
        }
      }
    }

    if (has_significant_change)
    {
      RCLCPP_INFO(
        get_logger(),
        "Significant scene change detected: %s",
        change_reason.c_str());
      previous_bboxes_ = current_boxes;
      return true;
    }

    return false;
  }

  bool detectSceneChange(
    const std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> & current_boxes)
  {
    if (previous_bboxes_.empty() || latest_odom_ == nullptr)
    {
      return false;
    }

    // Calculate robot movement since last detection
    double robot_x, robot_y;
    if (!getRobotPositionInMapFrame(robot_x, robot_y))
    {
      return false;
    }

    if (previous_robot_position_.has_value())
    {
      double dx = robot_x - previous_robot_position_->x;
      double dy = robot_y - previous_robot_position_->y;
      double distance_moved = std::hypot(dx, dy);

      if (distance_moved > scene_change_threshold_)
      {
        previous_robot_position_ = {robot_x, robot_y};
        return true;
      }
    }
    else
    {
      // Initialize robot position
      previous_robot_position_ = {robot_x, robot_y};
    }

    return false;
  }

  void triggerVLMQuery()
  {
    // Lock mutex to access latest_bboxes_
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Call internal implementation
    triggerVLMQueryImpl();
  }

private:
  // Internal implementation without locking (assumes caller holds data_mutex_)
  void triggerVLMQueryImpl()
  {
    // Allow VLM query even with empty bounding boxes (empty scene is valid)
    RCLCPP_INFO(
      get_logger(),
      "triggerVLMQuery() called with %zu bounding boxes%s",
      latest_bboxes_.size(),
      latest_bboxes_.empty() ? " (empty scene)" : "");

    // Assemble prompt from bounding boxes
    std::string prompt = assemblePrompt(latest_bboxes_);

    // Publish prompt
    std_msgs::msg::String prompt_msg;
    prompt_msg.data = prompt;
    prompt_pub_->publish(prompt_msg);

    RCLCPP_INFO(
      get_logger(),
      "✅ Published VLM prompt (%zu characters) to %s",
      prompt.length(), vlm_prompt_topic_.c_str());
    RCLCPP_DEBUG(
      get_logger(),
      "Prompt preview (first 200 chars):\n%.200s...",
      prompt.c_str());

    // Call VLM API if enabled
    if (enable_vlm_call_)
    {
      callVLMAPI(prompt, "scene_detection");
    }
  }

  std::string getGoalDirection(double robot_yaw, double goal_x, double goal_y,
                               double robot_x, double robot_y)
  {
    // Calculate angle to goal
    double dx = goal_x - robot_x;
    double dy = goal_y - robot_y;
    double angle_to_goal = std::atan2(dy, dx);

    // Calculate relative angle (goal direction relative to robot heading)
    double relative_angle = angle_to_goal - robot_yaw;

    // Normalize to [-pi, pi]
    while (relative_angle > M_PI) relative_angle -= 2.0 * M_PI;
    while (relative_angle < -M_PI) relative_angle += 2.0 * M_PI;

    // Convert to degrees for classification
    double angle_deg = relative_angle * 180.0 / M_PI;

    if (std::abs(angle_deg) < 15.0)
      return "straight ahead";
    else if (angle_deg >= 15.0 && angle_deg < 45.0)
      return "slightly left";
    else if (angle_deg >= 45.0 && angle_deg < 90.0)
      return "left";
    else if (angle_deg >= 90.0)
      return "sharp left";
    else if (angle_deg <= -15.0 && angle_deg > -45.0)
      return "slightly right";
    else if (angle_deg <= -45.0 && angle_deg > -90.0)
      return "right";
    else
      return "sharp right";
  }

  std::string assemblePrompt(
    const std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> & bboxes)
  {
    std::ostringstream prompt;

    // Build a map from person_id to PersonInfo for easy lookup
    std::map<int32_t, person_tracker::msg::PersonInfo> person_map;
    if (latest_person_info_)
    {
      for (const auto & person : latest_person_info_->persons)
      {
        person_map[person.person_id] = person;
      }
      RCLCPP_DEBUG(
        get_logger(),
        "Building prompt with %d tracked persons",
        latest_person_info_->num_persons);
    }

    // Extract robot state
    double v_linear = 0.0, v_angular = 0.0;
    double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
    std::string goal_direction = "unknown";
    double goal_distance = 0.0;
    double next_waypoint_x = goal_x_;  // Fallback to configured goal
    double next_waypoint_y = goal_y_;

    if (latest_odom_)
    {
      v_linear = latest_odom_->twist.twist.linear.x;
      v_angular = latest_odom_->twist.twist.angular.z;

      // Get robot position in map frame (with proper transformation)
      if (!getRobotPositionInMapFrame(robot_x, robot_y))
      {
        robot_x = 0.0;
        robot_y = 0.0;
      }

      // Extract yaw from quaternion
      double qx = latest_odom_->pose.pose.orientation.x;
      double qy = latest_odom_->pose.pose.orientation.y;
      double qz = latest_odom_->pose.pose.orientation.z;
      double qw = latest_odom_->pose.pose.orientation.w;
      robot_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

      // Get next waypoint from global path if available
      if (latest_global_path_ && !latest_global_path_->poses.empty())
      {
        // Find the closest waypoint ahead of the robot
        double min_distance_ahead = std::numeric_limits<double>::infinity();
        int best_waypoint_idx = 0;

        for (size_t i = 0; i < latest_global_path_->poses.size(); ++i)
        {
          const auto & pose = latest_global_path_->poses[i];
          double wp_x = pose.pose.position.x;
          double wp_y = pose.pose.position.y;

          // Calculate distance to waypoint
          double dist = std::hypot(wp_x - robot_x, wp_y - robot_y);

          // Check if waypoint is ahead of robot (dot product > 0)
          double dx = wp_x - robot_x;
          double dy = wp_y - robot_y;
          double dot = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);

          // Select closest waypoint that is ahead and at least 0.5m away
          if (dot > 0 && dist > 0.5 && dist < min_distance_ahead)
          {
            min_distance_ahead = dist;
            best_waypoint_idx = i;
          }
        }

        // Use the selected waypoint
        if (min_distance_ahead < std::numeric_limits<double>::infinity())
        {
          next_waypoint_x = latest_global_path_->poses[best_waypoint_idx].pose.position.x;
          next_waypoint_y = latest_global_path_->poses[best_waypoint_idx].pose.position.y;

          RCLCPP_DEBUG(
            get_logger(),
            "Using waypoint %d as next goal (distance: %.2f m)",
            best_waypoint_idx, min_distance_ahead);
        }
      }

      // Calculate goal direction and distance using next waypoint
      goal_direction = getGoalDirection(robot_yaw, next_waypoint_x, next_waypoint_y,
                                       robot_x, robot_y);
      goal_distance = std::hypot(next_waypoint_x - robot_x, next_waypoint_y - robot_y);
    }

    // Start building the prompt using the template
    prompt << "You are a social navigation advisor for a mobile robot.\n\n";
    prompt << "The robot has a forward-facing RGB-D camera. You are given:\n";
    prompt << "1) The current RGB image from the camera.\n";
    prompt << "2) A summary of humans detected in 3D using depth (already processed by a detector and tracker).\n";
    prompt << "3) The robot's navigation goal and motion state.\n\n";
    prompt << "Your job is to:\n";
    prompt << "- Understand the scene and the social context.\n";
    prompt << "- Predict the near-future motion of humans.\n";
    prompt << "- Decide a high-level navigation behavior that is socially appropriate and safe.\n";
    prompt << "- Provide a short explanation.\n\n";

    prompt << "### Robot state\n";
    prompt << "- Current linear velocity: " << std::fixed << std::setprecision(2)
           << v_linear << " m/s\n";
    prompt << "- Current angular velocity: " << std::fixed << std::setprecision(2)
           << v_angular << " rad/s\n";
    prompt << "- Goal direction: " << goal_direction << "\n";
    prompt << "- Goal distance: " << std::fixed << std::setprecision(2)
           << goal_distance << " m\n\n";

    prompt << "### Detected humans (from 3D bounding boxes)\n";

    // Count persons in bboxes
    int person_count = 0;
    for (const auto & bbox : bboxes)
    {
      if (bbox.object_name == "person")
      {
        person_count++;
      }
    }

    // Log the count for debugging
    RCLCPP_INFO(get_logger(),
                "📊 Counted %d person(s) in filtered bboxes for VLM prompt",
                person_count);

    if (person_count == 0)
    {
      prompt << "No humans currently detected.\n\n";
    }
    else
    {
      prompt << "**IMPORTANT: Total " << person_count << " human(s) detected**\n";
      prompt << "Based on this count, the crowd_density should be:\n";
      if (person_count < 3) {
        prompt << "  → 'sparse' (less than 3 people)\n";
      } else if (person_count <= 5) {
        prompt << "  → 'medium' (3-5 people)\n";
      } else {
        prompt << "  → 'dense' (more than 5 people)\n";
      }
      prompt << "\n";

      // Calculate distances first to provide critical warning
      std::vector<double> person_distances;
      for (const auto & bbox : bboxes)
      {
        if (bbox.object_name == "person")
        {
          double center_x = (bbox.xmin + bbox.xmax) / 2.0;
          double center_y = (bbox.ymin + bbox.ymax) / 2.0;
          double center_z = (bbox.zmin + bbox.zmax) / 2.0;
          double distance = std::hypot(center_x, center_y, center_z);
          person_distances.push_back(distance);
        }
      }

      // Add critical distance-based warning
      int close_count = 0;
      int far_count = 0;
      for (double d : person_distances)
      {
        if (d < 5.0) close_count++;
        if (d >= 10.0) far_count++;
      }

      if (close_count > 0)
      {
        prompt << "⚠️ ⚠️ ⚠️ CRITICAL WARNING - " << close_count << " CLOSE PEOPLE (< 5m) ⚠️ ⚠️ ⚠️\n";
        prompt << "Even though there are " << far_count << " far people, the " << close_count << " CLOSE people MUST dominate your decisions!\n";
        prompt << "Required action: speed_scale MUST be < 0.8 (NOT 0.9 or 1.0!)\n\n";
      }
      else if (far_count > 0)
      {
        prompt << "ℹ️ All " << far_count << " people are FAR AWAY (> 10m)\n";
        prompt << "Since NO people are close (< 5m), you can maintain higher speed (0.8-1.0)\n\n";
      }

      // Collect person data with distance for sorting
      struct PersonData {
        const gb_visual_detection_3d_msgs::msg::BoundingBox3d* bbox;
        double distance;
        double center_x, center_y, center_z;
        double angle_deg;
      };
      std::vector<PersonData> person_data_list;

      // First pass: collect all person data
      for (size_t i = 0; i < bboxes.size(); ++i)
      {
        const auto & bbox = bboxes[i];
        if (bbox.object_name != "person")
        {
          continue;
        }

        PersonData pdata;
        pdata.bbox = &bbox;
        pdata.center_x = (bbox.xmin + bbox.xmax) / 2.0;
        pdata.center_y = (bbox.ymin + bbox.ymax) / 2.0;
        pdata.center_z = (bbox.zmin + bbox.zmax) / 2.0;
        pdata.distance = std::hypot(pdata.center_x, pdata.center_y, pdata.center_z);

        // Calculate relative angle
        // Note: In ROS standard frames, x=forward, y=left(positive)/right(negative), z=up
        // If coordinate frame has y flipped (y positive = right), we need to negate center_y
        // Using -center_y to correct for potential coordinate frame flip
        double angle_rad = std::atan2(-pdata.center_y, pdata.center_x);
        pdata.angle_deg = angle_rad * 180.0 / M_PI;

        person_data_list.push_back(pdata);
      }

      // Sort by distance (closest first) - this emphasizes closer people
      std::sort(person_data_list.begin(), person_data_list.end(),
                [](const PersonData& a, const PersonData& b) {
                  return a.distance < b.distance;
                });

      // Second pass: describe each person, with distance-based priority labels
      int human_index = 1;
      for (const auto& pdata : person_data_list)
      {
        const auto & bbox = *pdata.bbox;
        double distance = pdata.distance;
        double angle_deg = pdata.angle_deg;

        // Debug logging for coordinate frame verification
        RCLCPP_DEBUG(get_logger(),
                     "Person %d: center=(%.2f, %.2f, %.2f), angle=%.1f deg, distance=%.2f m",
                     human_index, pdata.center_x, pdata.center_y, pdata.center_z, angle_deg, distance);

        // Add distance-based priority label, enriched with SARL attention if available
        std::string distance_label = "";
        if (distance < 3.0)
        {
          distance_label = " [CLOSE - HIGH PRIORITY]";
        }
        else if (distance < 5.0)
        {
          distance_label = " [NEAR - MEDIUM PRIORITY]";
        }
        else if (distance >= 10.0)
        {
          distance_label = " [FAR - LOW PRIORITY]";
        }

        // Inject SARL attention score if available (index-based matching,
        // person_data_list is sorted by distance, SARL uses People2D order)
        std::string sarl_label = "";
        if (latest_sarl_output_ && latest_sarl_output_->is_valid &&
            !latest_sarl_output_->attention_weights.empty())
        {
          float sarl_attn = 0.0f;
          bool found_match = false;

          // Try matching by person name from SARL output
          // pdata may have a tracker ID we can match against SARL person_names
          if (latest_person_info_)
          {
            // Find the tracker person closest to this bbox
            double min_tracker_dist = std::numeric_limits<double>::infinity();
            int matched_person_id = -1;
            for (const auto & person : latest_person_info_->persons)
            {
              double dx = pdata.center_x - person.position.x;
              double dy = pdata.center_y - person.position.y;
              double dist = std::hypot(dx, dy);
              if (dist < min_tracker_dist && dist < 1.0)
              {
                min_tracker_dist = dist;
                matched_person_id = person.person_id;
              }
            }

            if (matched_person_id >= 0)
            {
              std::string pid_str = std::to_string(matched_person_id);
              for (size_t si = 0; si < latest_sarl_output_->person_names.size(); ++si)
              {
                if (latest_sarl_output_->person_names[si] == pid_str &&
                    si < latest_sarl_output_->attention_weights.size())
                {
                  sarl_attn = latest_sarl_output_->attention_weights[si];
                  found_match = true;
                  break;
                }
              }
            }
          }

          // Fallback: use index if within bounds
          if (!found_match &&
              static_cast<size_t>(human_index - 1) < latest_sarl_output_->attention_weights.size())
          {
            sarl_attn = latest_sarl_output_->attention_weights[human_index - 1];
            found_match = true;
          }

          if (found_match)
          {
            std::string risk_level;
            if (sarl_attn > 0.5) { risk_level = "HIGH"; }
            else if (sarl_attn > 0.2) { risk_level = "MODERATE"; }
            else { risk_level = "LOW"; }

            std::ostringstream sarl_ss;
            sarl_ss << " [SARL_ATTENTION=" << std::fixed << std::setprecision(2)
                    << sarl_attn << ", RL_RISK=" << risk_level << "]";
            sarl_label = sarl_ss.str();
          }
        }

        prompt << "Human " << human_index << distance_label << sarl_label << ":\n";
        prompt << "  - Distance: " << std::fixed << std::setprecision(2)
               << distance << " m";

        // Describe relative position
        // Using ±30° front zone for center classification
        // After coordinate frame correction: positive angle = left, negative angle = right
        if (angle_deg > -30 && angle_deg < 30)
        {
          prompt << " (in front)";
        }
        else if (angle_deg >= 30 && angle_deg < 90)
        {
          prompt << " (front-left)";
        }
        else if (angle_deg <= -30 && angle_deg > -90)
        {
          prompt << " (front-right)";
        }
        else if (angle_deg >= 90)
        {
          prompt << " (left side)";
        }
        else
        {
          prompt << " (right side)";
        }
        prompt << "\n";

        // Try to find matching person tracking info
        const person_tracker::msg::PersonInfo* best_match = nullptr;
        double min_distance = std::numeric_limits<double>::infinity();

        if (latest_person_info_)
        {
          for (const auto & [person_id, person_info] : person_map)
          {
            double dx = pdata.center_x - person_info.position.x;
            double dy = pdata.center_y - person_info.position.y;
            double dz = pdata.center_z - person_info.position.z;
            double dist = std::hypot(dx, dy, dz);

            if (dist < min_distance && dist < 1.0)
            {
              min_distance = dist;
              best_match = &person_info;
            }
          }
        }

        // Add motion information if available
        if (best_match != nullptr)
        {
          prompt << "  - Speed: " << std::fixed << std::setprecision(2)
                 << best_match->speed << " m/s";

          // Motion state
          if (best_match->speed < 0.1)
          {
            prompt << " (standing still)";
          }
          else if (best_match->speed < 0.5)
          {
            prompt << " (walking slowly)";
          }
          else if (best_match->speed < 1.5)
          {
            prompt << " (walking)";
          }
          else
          {
            prompt << " (moving fast)";
          }
          prompt << "\n";

          // Movement direction
          if (best_match->speed > 0.1)
          {
            double heading_rad = std::atan2(best_match->velocity.y, best_match->velocity.x);
            double heading_deg = heading_rad * 180.0 / M_PI;

            // Determine if person is moving towards or away from robot
            double dot_product = (best_match->velocity.x * pdata.center_x +
                                 best_match->velocity.y * pdata.center_y);

            prompt << "  - Direction: ";
            if (dot_product < -0.05)
            {
              prompt << "approaching robot";
            }
            else if (dot_product > 0.05)
            {
              prompt << "moving away from robot";
            }
            else
            {
              prompt << "crossing path";
            }
            prompt << "\n";
          }
          else
          {
            prompt << "  - Direction: stationary\n";
          }

          prompt << "  - Tracking confidence: " << std::fixed << std::setprecision(2)
                 << best_match->confidence << "\n";
        }
        else
        {
          prompt << "  - Motion data: not available\n";
        }

        prompt << "\n";
        human_index++;
      }
    }

    // Add human cluster information
    prompt << "### Human cluster information\n";
    if (latest_human_clusters_ && latest_human_clusters_->num_clusters > 0)
    {
      prompt << "Detected " << latest_human_clusters_->num_clusters
             << " cluster(s) of pedestrians:\n\n";

      for (const auto & cluster : latest_human_clusters_->clusters)
      {
        prompt << "Cluster " << cluster.cluster_id << ":\n";
        prompt << "  - Number of people: " << cluster.cluster_size << "\n";
        prompt << "  - Centroid position: ("
               << std::fixed << std::setprecision(2)
               << cluster.centroid.x << ", "
               << cluster.centroid.y << ") m\n";

        // Calculate distance from robot to cluster centroid
        if (latest_odom_)
        {
          double cluster_dist = std::hypot(
            cluster.centroid.x - robot_x,
            cluster.centroid.y - robot_y);
          prompt << "  - Distance from robot: "
                 << std::fixed << std::setprecision(2)
                 << cluster_dist << " m\n";

          // Calculate relative angle to cluster
          double dx = cluster.centroid.x - robot_x;
          double dy = cluster.centroid.y - robot_y;
          double angle_to_cluster = std::atan2(dy, dx) - robot_yaw;
          // Normalize angle to [-pi, pi]
          while (angle_to_cluster > M_PI) angle_to_cluster -= 2.0 * M_PI;
          while (angle_to_cluster < -M_PI) angle_to_cluster += 2.0 * M_PI;
          double angle_deg = angle_to_cluster * 180.0 / M_PI;

          prompt << "  - Relative direction: ";
          // Narrowed front zone from ±30° to ±20° for better side classification
          if (angle_deg > -20 && angle_deg < 20)
          {
            prompt << "in front";
          }
          else if (angle_deg >= 20 && angle_deg < 90)
          {
            prompt << "front-left";
          }
          else if (angle_deg <= -20 && angle_deg > -90)
          {
            prompt << "front-right";
          }
          else if (angle_deg >= 90 && angle_deg < 150)
          {
            prompt << "left side";
          }
          else if (angle_deg <= -90 && angle_deg > -150)
          {
            prompt << "right side";
          }
          else
          {
            prompt << "behind";
          }
          prompt << "\n";
        }

        // List member IDs
        prompt << "  - Member person IDs: [";
        for (size_t i = 0; i < cluster.member_ids.size(); ++i)
        {
          prompt << cluster.member_ids[i];
          if (i < cluster.member_ids.size() - 1)
          {
            prompt << ", ";
          }
        }
        prompt << "]\n\n";
      }

      prompt << "Note: People in the same cluster are likely walking together as a group. "
             << "Consider treating clusters as single social units when planning navigation.\n\n";
    }
    else
    {
      prompt << "No clustering information available. Each person should be considered independently.\n\n";
    }

    // Add parameter guidelines to help VLM understand how to set control parameters
    prompt << "### Parameter Guidelines\n\n";

    prompt << "⚠️ CRITICAL - DISTANCE-BASED DECISION MAKING:\n";
    prompt << "ALL parameters must consider the DISTANCE to people, not just their presence!\n";
    prompt << "  • CLOSE people (< 5m): ALWAYS take priority - they determine speed/action\n";
    prompt << "  • MEDIUM distance (5-10m): Moderate adjustments based on density\n";
    prompt << "  • FAR people (> 10m): Should be IGNORED for speed decisions\n";
    prompt << "\n";
    prompt << "⚠️ PRIORITY RULE - NEVER VIOLATE:\n";
    prompt << "If EVEN ONE person is < 5m away → MUST reduce speed (< 0.8) regardless of how many far people exist\n";
    prompt << "  • Example: 2 people at 3m + 10 people at 20m\n";
    prompt << "    → Focus on the 2 CLOSE people, ignore the 10 far ones\n";
    prompt << "    → speed_scale: 0.7-0.8 (NOT 0.9! Close people require caution)\n";
    prompt << "  • Example: 0 people < 5m, 15 people at 20m\n";
    prompt << "    → crowd_density: 'dense', but speed_scale: 0.9 (all far away)\n";
    prompt << "\n";
    prompt << "⚠️ COMMON MISTAKE TO AVOID:\n";
    prompt << "✗ WRONG: \"3 people far, 2 people close → most are far, so speed=0.9\"\n";
    prompt << "✓ RIGHT: \"3 people far, 2 people close → 2 CLOSE people exist, so speed=0.7-0.8\"\n";
    prompt << "The presence of close people ALWAYS takes priority!\n\n";

    prompt << "**min_personal_distance** (0.5-2.0 meters):\n";
    prompt << "This sets how close the robot is allowed to pass near people.\n";
    prompt << "CRITICAL - Must consider BOTH crowd_density AND proximity:\n";
    prompt << "- CLOSE dense crowd (6+ people < 5m)  → min_personal_distance: 0.6-0.8m (tight spaces)\n";
    prompt << "- CLOSE medium crowd (3-5 people < 5m) → min_personal_distance: 0.9-1.1m (moderate)\n";
    prompt << "- CLOSE sparse (1-2 people < 5m)      → min_personal_distance: 1.2-1.5m (comfortable)\n";
    prompt << "- FAR crowds (> 10m) or empty         → min_personal_distance: 1.2-1.5m (default)\n";
    prompt << "Distance matters! Don't reduce personal space for far-away crowds.\n\n";

    prompt << "**speed_scale** (0.0-1.0):\n";
    prompt << "Multiplier for robot's maximum speed.\n";
    prompt << "⚠️ CRITICAL - DISTANCE MATTERS MORE THAN COUNT:\n";
    prompt << "- CLOSE dense crowd (6+ people < 5m)   → speed_scale: 0.4-0.6 (slow down significantly)\n";
    prompt << "- CLOSE medium crowd (3-5 people < 5m) → speed_scale: 0.6-0.8 (moderate caution)\n";
    prompt << "- CLOSE sparse (1-2 people < 5m)       → speed_scale: 0.7-0.9 (slight caution)\n";
    prompt << "- FAR crowds (> 10m), regardless of count → speed_scale: 0.8-1.0 (maintain speed)\n";
    prompt << "- Empty or no close people              → speed_scale: 0.9-1.0 (full speed)\n";
    prompt << "\n";
    prompt << "Example violations to AVOID:\n";
    prompt << "  ✗ WRONG: 8 people at 15m → speed_scale: 0.5 (too slow for far crowd!)\n";
    prompt << "  ✓ RIGHT: 8 people at 15m → speed_scale: 0.8-0.9 (they're far, maintain speed)\n";
    prompt << "  ✗ WRONG: 2 people at 2m → speed_scale: 0.9 (too fast for close people!)\n";
    prompt << "  ✓ RIGHT: 2 people at 2m → speed_scale: 0.7-0.8 (close people, slow down)\n\n";

    prompt << "**side_preference** (left, right, neutral) - CRITICAL PARAMETER:\n";
    prompt << "Which side of the path to goal the robot should bias toward (lateral offset from centerline).\n";
    prompt << "\n";
    prompt << "⚠️ CRITICAL LOGIC - READ CAREFULLY:\n";
    prompt << "side_preference tells the robot which EMPTY side to use for passing.\n";
    prompt << "  • If people are on the RIGHT → robot passes on the LEFT (side_preference='left')\n";
    prompt << "  • If people are on the LEFT → robot passes on the RIGHT (side_preference='right')\n";
    prompt << "  • Think: 'People on X side, so I go to the OPPOSITE side to avoid them'\n";
    prompt << "\n";
    prompt << "⚠️ IMPORTANT: Both 'left' and 'right' are EQUALLY VALID. There is NO inherent preference for either side!\n";
    prompt << "\n";
    prompt << "Decision rules (MUST FOLLOW EXACTLY):\n";
    prompt << "  1. People on LEFT side  → side_preference = 'right' (pass on RIGHT to avoid them)\n";
    prompt << "  2. People on RIGHT side → side_preference = 'left'  (pass on LEFT to avoid them)\n";
    prompt << "  3. People on BOTH sides → side_preference = 'neutral' (symmetric scene)\n";
    prompt << "  4. No people nearby     → side_preference = 'neutral'\n";
    prompt << "\n";
    prompt << "⚠️ DISTANCE WEIGHTING - CRITICAL (READ THIS CAREFULLY):\n";
    prompt << "The weight of each person in side_preference decision is INVERSE to their distance:\n";
    prompt << "  • CLOSE people (< 3m): Weight = 10x (HIGHEST PRIORITY - dominate decision)\n";
    prompt << "  • NEAR people (3-5m): Weight = 3x (MEDIUM PRIORITY - significant influence)\n";
    prompt << "  • MEDIUM people (5-10m): Weight = 1x (STANDARD - normal influence)\n";
    prompt << "  • FAR people (> 10m): Weight = 0.1x (MINIMAL - almost ignore for side_preference)\n";
    prompt << "\n";
    prompt << "Decision algorithm:\n";
    prompt << "  1. Count people on LEFT side, weighted by distance (close=10x, near=3x, medium=1x, far=0.1x)\n";
    prompt << "  2. Count people on RIGHT side, weighted by distance (same weights)\n";
    prompt << "  3. Choose side_preference = OPPOSITE of the side with HIGHER weighted count\n";
    prompt << "  4. If weighted counts are similar (< 20% difference) → use 'neutral'\n";
    prompt << "\n";
    prompt << "Examples:\n";
    prompt << "  • 2 CLOSE people (2m) on RIGHT + 5 FAR people (15m) on LEFT:\n";
    prompt << "    → RIGHT weighted count: 2 × 10 = 20\n";
    prompt << "    → LEFT weighted count: 5 × 0.1 = 0.5\n";
    prompt << "    → Correct: side_preference='left' (avoid the 2 CLOSE people on right, weight=20)\n";
    prompt << "    → WRONG: side_preference='right' (ignoring close threat!)\n";
    prompt << "  • 1 CLOSE person (2.5m) on LEFT + 3 NEAR people (4m) on RIGHT:\n";
    prompt << "    → LEFT weighted count: 1 × 10 = 10\n";
    prompt << "    → RIGHT weighted count: 3 × 3 = 9\n";
    prompt << "    → Correct: side_preference='right' (avoid the CLOSE person on left, weight=10)\n";
    prompt << "  • 2 NEAR people (4m) on LEFT + 1 FAR person (12m) on RIGHT:\n";
    prompt << "    → LEFT weighted count: 2 × 3 = 6\n";
    prompt << "    → RIGHT weighted count: 1 × 0.1 = 0.1\n";
    prompt << "    → Correct: side_preference='right' (avoid the 2 NEAR people on left)\n";
    prompt << "\n";
    prompt << "⚠️ REMEMBER: People are listed in order of distance (closest first).\n";
    prompt << "   The first few people in the list have MUCH MORE influence on your decision!\n";
    prompt << "\n";
    prompt << "Examples to learn from:\n";
    prompt << "  • Scene: 2 people close on RIGHT side (front-right)\n";
    prompt << "    → Correct: side_preference='left' (pass LEFT to avoid people on right)\n";
    prompt << "    → WRONG: side_preference='right' (this would drive toward the people!)\n";
    prompt << "  • Scene: People at front-LEFT and left side\n";
    prompt << "    → Correct: side_preference='right' (pass on right to avoid people on left)\n";
    prompt << "  • Scene: 3 people on LEFT, 1 person far on RIGHT\n";
    prompt << "    → Correct: side_preference='right' (avoid cluster on left, pass right)\n";
    prompt << "\n";
    prompt << "⚠️ BIAS CHECK: Both 'left' and 'right' are valid choices depending on people's positions.\n";
    prompt << "   Choose the side with MORE clearance, regardless of which side that is.\n";
    prompt << "\n";
    prompt << "The robot will try to stay on the specified side of its path while moving toward the goal.\n";
    prompt << "Consider: relative positions of people (front-left, front-right), which side has more clearance.\n";
    prompt << "Note: Static obstacles (walls, furniture) are handled separately by laser-based avoidance.\n\n";

    prompt << "**CRITICAL: How to use pedestrian velocity information**:\n";
    prompt << "The detected humans include speed and direction data. Use this to adjust parameters:\n\n";
    prompt << "Person APPROACHING robot (moving toward):\n";
    prompt << "  → INCREASE min_personal_distance (+0.2-0.4m buffer for collision avoidance)\n";
    prompt << "  → DECREASE speed_scale (-0.1 to -0.3, be more cautious)\n";
    prompt << "  → Choose side_preference away from their approach direction\n";
    prompt << "  Example: Person at 3m approaching → distance: 1.3-1.5m, speed: 0.6-0.7\n\n";
    prompt << "Person MOVING AWAY from robot:\n";
    prompt << "  → Use STANDARD or slightly lower min_personal_distance (0.8-1.0m, less risk)\n";
    prompt << "  → Can maintain higher speed_scale (0.8-1.0, safe to proceed)\n";
    prompt << "  Example: Person at 2m walking away → distance: 0.9m, speed: 0.9\n\n";
    prompt << "Person CROSSING path (perpendicular motion):\n";
    prompt << "  → MODERATE min_personal_distance (1.0-1.3m, account for trajectory uncertainty)\n";
    prompt << "  → REDUCE speed_scale (0.5-0.7, wait for them to cross)\n";
    prompt << "  → Use side_preference to pass behind them if possible\n";
    prompt << "  Example: Person at 2m crossing → distance: 1.1m, speed: 0.6\n\n";
    prompt << "Person STATIONARY (standing still):\n";
    prompt << "  → Use STANDARD min_personal_distance (0.9-1.2m, predictable obstacle)\n";
    prompt << "  → Can use higher speed_scale (0.7-0.9, known position)\n";
    prompt << "  Example: Person at 2m standing → distance: 1.0m, speed: 0.8\n\n";
    prompt << "Person moving FAST (>1.5 m/s, running/rushing):\n";
    prompt << "  → INCREASE min_personal_distance significantly (+0.3-0.5m, less predictable)\n";
    prompt << "  → STRONGLY reduce speed_scale (0.4-0.6, high caution)\n";
    prompt << "  Example: Person at 3m moving fast toward robot → distance: 1.6m, speed: 0.5\n\n";

    prompt << "**Example scenarios** (showing how velocity affects parameters):\n";
    prompt << "1. Person at 2m front-right, STATIONARY (0.0 m/s):\n";
    prompt << "   → min_personal_distance: 1.0, speed_scale: 0.8, side_preference: 'left'\n";
    prompt << "   Reasoning: Stationary = predictable, standard distance, can maintain speed, pass on left\n\n";
    prompt << "2. Person at 2.5m front-left, APPROACHING robot (0.8 m/s):\n";
    prompt << "   → min_personal_distance: 1.4, speed_scale: 0.6, side_preference: 'right'\n";
    prompt << "   Reasoning: Approaching = increase distance buffer, slow down, pass on right (away from them)\n\n";
    prompt << "3. Person at 1.5m front-right, MOVING AWAY (0.6 m/s):\n";
    prompt << "   → min_personal_distance: 0.9, speed_scale: 0.9, side_preference: 'left'\n";
    prompt << "   Reasoning: Moving away = less risk, can use smaller distance and higher speed\n\n";
    prompt << "4. Person at 2m front, CROSSING path (0.5 m/s):\n";
    prompt << "   → min_personal_distance: 1.1, speed_scale: 0.6, side_preference: 'neutral'\n";
    prompt << "   Reasoning: Crossing = moderate distance, slow down to let them pass\n\n";
    prompt << "5. Person at 3m front-right, MOVING FAST toward robot (2.1 m/s, running):\n";
    prompt << "   → min_personal_distance: 1.7, speed_scale: 0.5, side_preference: 'left'\n";
    prompt << "   Reasoning: Fast approaching = large buffer needed, significant slowdown, evade left\n\n";
    prompt << "6. DENSE CROWD: 7 people detected, some on both sides:\n";
    prompt << "   → crowd_density: 'dense', min_personal_distance: 0.7, speed_scale: 0.5, side_preference: 'neutral'\n";
    prompt << "   Reasoning: Dense (6+ people) = MUST use small distance (0.6-0.8m) and slow speed (0.4-0.6)\n\n";
    prompt << "7. Empty corridor, no people detected:\n";
    prompt << "   → crowd_density: 'empty', min_personal_distance: 1.3, speed_scale: 1.0, side_preference: 'neutral'\n";
    prompt << "   Reasoning: Empty (0 people) = can maintain comfort distance and full speed\n\n";

    // Add JSON output format requirements
    prompt << "### Required output format\n";
    prompt << "Respond ONLY with a JSON object using this schema:\n\n";
    prompt << "{\n";
    prompt << "  \"scene_type\": \"<one of: corridor, lobby, open_space, doorway, queue, crossing, unknown>\",\n";
    prompt << "  \"crowd_density\": \"<one of: empty, sparse, medium, dense>\",\n";
    prompt << "  \"recommended_action\": \"<one of: go_ahead, slow_down_and_go, stop_and_wait, keep_left, keep_right, yield_to_pedestrian>\",\n";
    prompt << "  \"speed_scale\": <float between 0.0 and 1.0>,\n";
    prompt << "  \"min_personal_distance\": <float in meters, between 0.5 and 2.0>,\n";
    prompt << "  \"side_preference\": \"<one of: left, right, neutral>\",\n";
    prompt << "  \"need_to_wait\": <true or false>,\n";
    prompt << "  \"explanation\": \"<1-2 short sentences explaining what the robot should do and why>\"\n";
    prompt << "}\n\n";
    prompt << "CRITICAL INSTRUCTIONS:\n";
    prompt << "1. Use the TOTAL COUNT from the '3D human summary' section above for crowd_density classification.\n";
    prompt << "2. The image may not show all detected people clearly, but the 3D detector count is accurate.\n";
    prompt << "3. Crowd density MUST match the total count:\n";
    prompt << "   • empty: 0 people\n";
    prompt << "   • sparse: less than 3 people (1-2)\n";
    prompt << "   • medium: 3-5 people\n";
    prompt << "   • dense: more than 5 people (6+)\n\n";
    prompt << "Additional guidelines:\n";
    prompt << "- Base your scene understanding on BOTH the image and 3D human summary.\n";
    prompt << "- Consider spatial distribution: distant people (>10m) or behind robot may be less relevant.\n";
    prompt << "- The JSON must be valid. Do not include any extra text outside the JSON.\n\n";

    prompt << "⚠️ FINAL CRITICAL REMINDER FOR side_preference:\n";
    prompt << "  • People on RIGHT → side_preference='left' (go to the empty LEFT side)\n";
    prompt << "  • People on LEFT → side_preference='right' (go to the empty RIGHT side)\n";
    prompt << "  • Your explanation MUST match your side_preference choice!\n";
    prompt << "  • Example: If you see people on the right and choose side_preference='left',\n";
    prompt << "    explain: 'people on right, so pass on left to avoid them' ✓\n";
    prompt << "  • DO NOT say: 'people on right, so pass on right' ✗ (this is wrong!)\n\n";

    return prompt.str();
  }

  // Helper to transform robot position from odom to map frame
  bool getRobotPositionInMapFrame(double& x, double& y)
  {
    if (!latest_odom_)
    {
      return false;
    }

    try
    {
      // Lookup transform from odom to map
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(
          map_frame_,                    // target frame: map
          odom_frame_,                   // source frame: odom
          rclcpp::Time(0));              // get latest available transform

      // Create pose in odom frame
      geometry_msgs::msg::PoseStamped pose_odom;
      pose_odom.header.frame_id = odom_frame_;
      pose_odom.header.stamp = latest_odom_->header.stamp;
      pose_odom.pose = latest_odom_->pose.pose;

      // Transform to map frame
      geometry_msgs::msg::PoseStamped pose_map;
      tf2::doTransform(pose_odom, pose_map, transform_stamped);

      // Return transformed position
      x = pose_map.pose.position.x;
      y = pose_map.pose.position.y;
      return true;

    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Could not transform robot position to map frame: %s. Using odom coordinates as fallback.", ex.what());
      // Fallback: use odom coordinates directly
      x = latest_odom_->pose.pose.position.x;
      y = latest_odom_->pose.pose.position.y;
      return true;  // Still return true since we have a position (even if not transformed)
    }
  }

  // Helper function to log VLM inference time to CSV
  void logVLMInferenceTime(double inference_time_ms, const std::string& trigger_type)
  {
    std::lock_guard<std::mutex> lock(csv_mutex_);
    if (vlm_inference_csv_.is_open())
    {
      double timestamp_sec = now().seconds();
      vlm_inference_csv_ << std::fixed << std::setprecision(3)
                         << timestamp_sec << ","
                         << inference_time_ms << ","
                         << trigger_type << "\n";
      vlm_inference_csv_.flush();
      RCLCPP_INFO(get_logger(), "VLM inference time logged: %.2f ms (%s)",
                  inference_time_ms, trigger_type.c_str());
    }
  }

  // Helper to escape JSON strings
  std::string escapeJsonString(const std::string& input)
  {
    std::string output;
    output.reserve(input.size() * 1.1);
    for (char c : input)
    {
      switch (c)
      {
        case '"':  output += "\\\""; break;
        case '\\': output += "\\\\"; break;
        case '\n': output += "\\n";  break;
        case '\r': output += "\\r";  break;
        case '\t': output += "\\t";  break;
        case '\b': output += "\\b";  break;
        case '\f': output += "\\f";  break;
        default:
          if (static_cast<unsigned char>(c) < 0x20)
          {
            // Control characters
            char buf[8];
            snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned int>(c));
            output += buf;
          }
          else
          {
            output += c;
          }
      }
    }
    return output;
  }

  void callVLMAPI(const std::string & prompt, const std::string & trigger_type = "normal")
  {
    if (!enable_vlm_call_)
    {
      RCLCPP_DEBUG(
        get_logger(),
        "VLM API call disabled. Enable via enable_vlm_call parameter.");
      return;
    }

    // Check if a request is already in progress
    if (vlm_request_in_progress_.load())
    {
      RCLCPP_WARN(get_logger(),
        "VLM request already in progress, skipping new request. "
        "Waiting for previous response before sending new prompt.");
      return;
    }

    // Get the base64-encoded image
    std::string base64_image = getImageBase64();
    if (base64_image.empty())
    {
      RCLCPP_WARN(get_logger(), "No image available, skipping VLM call");
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Calling VLM API:\n"
      "  URL: %s\n"
      "  Model: %s\n"
      "  Prompt length: %zu chars\n"
      "  Image base64 length: %zu chars",
      vlm_api_url_.c_str(), vlm_model_.c_str(),
      prompt.length(), base64_image.size());

    // Mark request as in progress and update last call time
    vlm_request_in_progress_ = true;
    last_vlm_call_time_ = now();  // Reset idle timer for heartbeat

    // Capture VLM inference start time
    vlm_inference_start_time_ = now();

    // Build JSON payload for OpenAI Vision API format
    // The format includes both text and image in the content array
    std::ostringstream json_payload;
    json_payload << "{"
                 << "\"model\": \"" << vlm_model_ << "\","
                 << "\"messages\": [{"
                 << "\"role\": \"user\","
                 << "\"content\": ["
                 << "{"
                 << "\"type\": \"image_url\","
                 << "\"image_url\": {"
                 << "\"url\": \"data:image/jpeg;base64," << base64_image << "\""
                 << "}"
                 << "},"
                 << "{"
                 << "\"type\": \"text\","
                 << "\"text\": \"" << escapeJsonString(prompt) << "\""
                 << "}"
                 << "]"
                 << "}]"
                 << "}";

    std::string payload = json_payload.str();

    RCLCPP_INFO(get_logger(),
      "VLM API request payload size: %zu bytes", payload.length());

    // Launch async HTTP request in a separate thread
    std::thread([this, payload, prompt, trigger_type]()
    {
      std::string response_body;
      bool success = false;

      CURL* curl = curl_easy_init();
      if (curl)
      {
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        if (!vlm_api_key_.empty())
        {
          std::string auth_header = "Authorization: Bearer " + vlm_api_key_;
          headers = curl_slist_append(headers, auth_header.c_str());
        }

        curl_easy_setopt(curl, CURLOPT_URL, vlm_api_url_.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, payload.length());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 120L);  // 120 second timeout
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);

        CURLcode res = curl_easy_perform(curl);

        if (res == CURLE_OK)
        {
          long http_code = 0;
          curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

          if (http_code == 200)
          {
            success = true;
            RCLCPP_INFO(rclcpp::get_logger("vlm_integration_node"),
              "VLM API call successful. Response size: %zu bytes",
              response_body.length());
          }
          else
          {
            RCLCPP_ERROR(rclcpp::get_logger("vlm_integration_node"),
              "VLM API returned HTTP %ld: %s", http_code, response_body.c_str());
          }
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("vlm_integration_node"),
            "VLM API call failed: %s", curl_easy_strerror(res));
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("vlm_integration_node"),
          "Failed to initialize CURL");
      }

      // Publish response on the main thread context
      // Since we're in a separate thread, we need to be careful
      std_msgs::msg::String response_msg;

      if (success)
      {
        // Extract the content from OpenAI response format
        // Response format: {"choices":[{"message":{"content":"..."}}]}
        std::string content = response_body;

        // Try to extract just the content field for cleaner output
        size_t content_start = response_body.find("\"content\":");
        if (content_start != std::string::npos)
        {
          content_start = response_body.find("\"", content_start + 10);
          if (content_start != std::string::npos)
          {
            size_t content_end = content_start + 1;
            int escape_count = 0;
            while (content_end < response_body.length())
            {
              if (response_body[content_end] == '\\')
              {
                escape_count++;
                content_end++;
              }
              else if (response_body[content_end] == '"' && (escape_count % 2 == 0))
              {
                break;
              }
              else
              {
                escape_count = 0;
              }
              content_end++;
            }
            content = response_body.substr(content_start + 1, content_end - content_start - 1);

            // Unescape the content
            std::string unescaped;
            for (size_t i = 0; i < content.length(); i++)
            {
              if (content[i] == '\\' && i + 1 < content.length())
              {
                char next = content[i + 1];
                switch (next)
                {
                  case 'n': unescaped += '\n'; i++; break;
                  case 'r': unescaped += '\r'; i++; break;
                  case 't': unescaped += '\t'; i++; break;
                  case '"': unescaped += '"'; i++; break;
                  case '\\': unescaped += '\\'; i++; break;
                  default: unescaped += content[i];
                }
              }
              else
              {
                unescaped += content[i];
              }
            }
            content = unescaped;
          }
        }

        response_msg.data = content;
        RCLCPP_INFO(rclcpp::get_logger("vlm_integration_node"),
          "Publishing VLM response: %s",
          content.substr(0, std::min(content.length(), size_t(200))).c_str());
      }
      else
      {
        response_msg.data = "{\"error\": \"VLM API call failed\", \"details\": \"" +
                           escapeJsonString(response_body) + "\"}";
      }

      response_pub_->publish(response_msg);

      // Calculate and log VLM inference time
      if (success)
      {
        auto end_time = this->now();
        double inference_time_ms = (end_time - vlm_inference_start_time_).seconds() * 1000.0;
        logVLMInferenceTime(inference_time_ms, trigger_type);
      }

      // Mark request as complete - ready for next request
      vlm_request_in_progress_ = false;

      RCLCPP_INFO(rclcpp::get_logger("vlm_integration_node"),
        "VLM request completed. Ready for next prompt.");

    }).detach();  // Detach thread to run independently
  }

  // Parameters
  std::string bounding_boxes_topic_;
  std::string person_info_topic_;
  std::string human_clusters_topic_;
  std::string odom_topic_;
  std::string rgb_image_topic_;
  std::string vlm_prompt_topic_;
  std::string vlm_response_topic_;
  std::string vlm_api_url_;
  std::string vlm_model_;
  std::string vlm_api_key_;
  bool enable_vlm_call_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_base_frame_;
  double goal_x_;
  double goal_y_;
  int image_resize_width_;
  int image_resize_height_;
  int jpeg_quality_;

  // Event trigger parameters
  bool trigger_on_new_detection_;
  bool trigger_periodic_;
  double periodic_interval_sec_;
  bool trigger_on_scene_change_;
  double scene_change_threshold_;
  double min_confidence_;
  double max_detection_distance_;
  double heartbeat_interval_sec_;

  // VLM heartbeat parameters
  bool enable_vlm_heartbeat_;
  double vlm_heartbeat_interval_sec_;
  double vlm_idle_threshold_sec_;
  rclcpp::Time last_vlm_call_time_;

  // Advanced trigger conditions
  bool trigger_on_new_object_type_;
  bool trigger_on_object_count_change_;
  int object_count_change_threshold_;
  bool trigger_on_object_position_change_;
  double object_position_change_threshold_;
  bool trigger_on_object_type_combination_change_;

  // Subscribers
  rclcpp::Subscription<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr bbox_sub_;
  rclcpp::Subscription<person_tracker::msg::PersonInfoArray>::SharedPtr person_info_sub_;
  rclcpp::Subscription<person_tracker::msg::HumanClusterArray>::SharedPtr human_clusters_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<social_mpc_nav::msg::SARLOutput>::SharedPtr sarl_sub_;

  // SARL parameters
  std::string sarl_output_topic_;
  bool trigger_on_sarl_attention_shift_;
  double sarl_attention_shift_threshold_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr prompt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr periodic_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr vlm_heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr warmup_timer_;  // One-time startup warmup timer (heartbeat)
  rclcpp::TimerBase::SharedPtr navigation_warmup_timer_;  // One-time navigation warmup (unlock MPC)

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Data storage
  std::mutex data_mutex_;
  std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> latest_bboxes_;
  std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> previous_bboxes_;
  builtin_interfaces::msg::Time latest_bbox_timestamp_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  person_tracker::msg::PersonInfoArray::SharedPtr latest_person_info_;
  person_tracker::msg::HumanClusterArray::SharedPtr latest_human_clusters_;
  nav_msgs::msg::Path::SharedPtr latest_global_path_;
  social_mpc_nav::msg::SARLOutput::SharedPtr latest_sarl_output_;

  struct RobotPosition
  {
    double x;
    double y;
  };
  std::optional<RobotPosition> previous_robot_position_;

  // Image storage
  std::mutex image_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_image_;

  // VLM request state - atomic to allow thread-safe access
  std::atomic<bool> vlm_request_in_progress_;

  // VLM inference timing and CSV logging
  std::string vlm_inference_log_file_;
  std::ofstream vlm_inference_csv_;
  std::mutex csv_mutex_;
  rclcpp::Time vlm_inference_start_time_;
};

// Static member definition for base64 encoding
const std::string VLMIntegrationNode::base64_chars =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  "abcdefghijklmnopqrstuvwxyz"
  "0123456789+/";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VLMIntegrationNode>());
  rclcpp::shutdown();
  return 0;
}