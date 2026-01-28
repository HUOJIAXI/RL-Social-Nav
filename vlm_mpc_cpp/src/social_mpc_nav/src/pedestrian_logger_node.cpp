#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hunav_msgs/msg/agents.hpp"
#include "hunav_msgs/msg/agent.hpp"

/**
 * @brief Node that logs ground truth pedestrian movements from hunav (arena4)
 *
 * Subscribes to /human_states (hunav ground truth) and logs pedestrian positions,
 * velocities, and headings to a CSV file for social compliance analysis.
 */
class PedestrianLoggerNode : public rclcpp::Node
{
public:
  PedestrianLoggerNode()
  : Node("pedestrian_logger_node")
  {
    // Declare parameters
    log_directory_ = declare_parameter<std::string>(
      "log_directory",
      "~/ros2_logs/social_mpc_nav");

    human_states_topic_ = declare_parameter<std::string>(
      "human_states_topic", "/task_generator_node/human_states");

    log_rate_hz_ = declare_parameter<double>("log_rate_hz", 10.0);

    enable_logging_ = declare_parameter<bool>("enable_logging", true);

    if (!enable_logging_) {
      RCLCPP_INFO(get_logger(), "Pedestrian logging is disabled");
      return;
    }

    // Setup CSV logging
    setupLogging();

    // Subscribe to hunav ground truth topic
    human_states_sub_ = create_subscription<hunav_msgs::msg::Agents>(
      human_states_topic_,
      rclcpp::QoS(10),
      std::bind(&PedestrianLoggerNode::onHumanStates, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Pedestrian logger started. Subscribed to: %s | Logging to: %s",
      human_states_topic_.c_str(),
      log_file_path_.c_str());
  }

  ~PedestrianLoggerNode()
  {
    if (log_stream_.is_open()) {
      log_stream_.close();
      RCLCPP_INFO(get_logger(), "Pedestrian log file closed");
    }
  }

private:
  void setupLogging()
  {
    namespace fs = std::filesystem;

    // Expand tilde in path
    std::string expanded_dir = log_directory_;
    if (!expanded_dir.empty() && expanded_dir[0] == '~') {
      const char* home = std::getenv("HOME");
      if (home) {
        expanded_dir = std::string(home) + expanded_dir.substr(1);
      }
    }

    fs::path log_dir(expanded_dir);

    try {
      // Create directory if it doesn't exist
      if (!fs::exists(log_dir)) {
        fs::create_directories(log_dir);
      }

      // Generate timestamp for filename
      auto timestamp = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(timestamp);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
      std::string time_str = ss.str();

      // Create log file
      log_file_path_ = expanded_dir + "/pedestrian_ground_truth_" + time_str + ".csv";
      log_stream_.open(log_file_path_);

      if (log_stream_.is_open()) {
        // Write CSV header
        log_stream_ << "timestamp_sec,pedestrian_id,x,y,vx,vy,speed,yaw\n";
        log_stream_.flush();
        RCLCPP_INFO(get_logger(), "Pedestrian ground truth log: %s", log_file_path_.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to setup logging: %s", e.what());
    }
  }

  // Callback for hunav_msgs/Agents messages (ground truth)
  void onHumanStates(const hunav_msgs::msg::Agents::SharedPtr msg)
  {
    if (!log_stream_.is_open()) {
      return;
    }

    // Get timestamp from message header
    double timestamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // Process all agents in the message
    for (const auto &agent : msg->agents) {
      // Filter for persons only (exclude robots and other types)
      if (agent.type != hunav_msgs::msg::Agent::PERSON) {
        continue;
      }

      // Extract position from geometry_msgs/Pose
      double x = agent.position.position.x;
      double y = agent.position.position.y;

      // Extract velocity from geometry_msgs/Twist
      double vx = agent.velocity.linear.x;
      double vy = agent.velocity.linear.y;
      double speed = std::hypot(vx, vy);

      // Extract yaw (already provided in Agent message)
      double yaw = agent.yaw;

      // Write to CSV
      log_stream_ << std::fixed << std::setprecision(6)
                  << timestamp_sec << ","
                  << agent.id << ","
                  << x << ","
                  << y << ","
                  << vx << ","
                  << vy << ","
                  << speed << ","
                  << yaw << "\n";
    }

    // Flush to ensure data is written (prevents data loss on crashes)
    log_stream_.flush();

    // Log statistics periodically
    static int callback_count = 0;
    callback_count++;
    if (callback_count % 100 == 0) {
      size_t person_count = 0;
      for (const auto &agent : msg->agents) {
        if (agent.type == hunav_msgs::msg::Agent::PERSON) {
          person_count++;
        }
      }
      RCLCPP_DEBUG(
        get_logger(),
        "Logged %zu pedestrians (total callbacks: %d)",
        person_count,
        callback_count);
    }
  }

  // Member variables
  std::string log_directory_;
  std::string human_states_topic_;
  double log_rate_hz_;
  bool enable_logging_;

  std::string log_file_path_;
  std::ofstream log_stream_;

  rclcpp::Subscription<hunav_msgs::msg::Agents>::SharedPtr human_states_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PedestrianLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
