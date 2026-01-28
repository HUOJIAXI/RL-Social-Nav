#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/people.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/msg/person2_d.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

/**
 * @brief Adapter node that converts people_msgs/msg/People to custom People2D messages.
 *
 * This node bridges the gap between the system's people detection (people_msgs)
 * and the MPC controller's expected format (People2D). It subscribes to the
 * people topic from task_generator and republishes in the format expected by
 * the social MPC navigation stack.
 *
 * FUTURE VLM INTEGRATION POINT:
 * This is an ideal location to add VLM-based enhancements:
 * - Replace/augment people_msgs input with VLM-detected pedestrians
 * - Add semantic attributes (groups, intentions, restricted zones)
 * - Compute VLM-derived constraints before publishing
 */
class PeopleAdapterNode : public rclcpp::Node
{
public:
  PeopleAdapterNode()
  : Node("people_adapter_node")
  {
    // Parameters
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/task_generator_node/people");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/crowd/people2d");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    use_tf_for_positions_ = declare_parameter<bool>("use_tf_for_positions", true);

    // Dynamic actor discovery from TF tree
    // We'll discover actors matching pattern D_gazebo_actor_* automatically
    actor_frame_prefix_ = declare_parameter<std::string>(
      "actor_frame_prefix", "D_gazebo_actor_");
    max_actors_to_check_ = declare_parameter<int>("max_actors_to_check", 50);

    const char * home = std::getenv("HOME");
    const std::string default_log = home ?
      std::string(home) + "/ros2_logs/social_mpc_nav" : "/tmp/social_mpc_nav";
    log_directory_ = declare_parameter<std::string>("log_directory", default_log);
    log_to_csv_ = declare_parameter<bool>("log_people_to_csv", true);

    // Create publisher
    people_pub_ = create_publisher<social_mpc_nav::msg::People2D>(
      output_topic_, rclcpp::QoS(10));

    if (use_tf_for_positions_)
    {
      // Initialize TF2 for getting actor positions from TF tree
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Discover actors from TF tree immediately
      discoverActorFrames();

      // Create timer to periodically discover new actors (every 5 seconds)
      discovery_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&PeopleAdapterNode::discoverActorFrames, this));

      // Create timer to periodically publish actor positions from TF
      tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&PeopleAdapterNode::publishPeopleFromTF, this));

      RCLCPP_INFO(
        get_logger(),
        "people_adapter_node started: Using TF tree for dynamic actor discovery -> %s",
        output_topic_.c_str());
    }
    else
    {
      // Fallback: Subscribe to people topic
      people_sub_ = create_subscription<people_msgs::msg::People>(
        input_topic_, rclcpp::QoS(10),
        std::bind(&PeopleAdapterNode::onPeople, this, std::placeholders::_1));

      RCLCPP_INFO(
        get_logger(),
        "people_adapter_node started: %s -> %s",
        input_topic_.c_str(), output_topic_.c_str());
    }

    if (log_to_csv_)
    {
      createLogFile();
    }
  }

private:
  void onPeople(const people_msgs::msg::People::SharedPtr msg)
  {
    // Convert people_msgs::msg::People to social_mpc_nav::msg::People2D
    auto people2d_msg = std::make_shared<social_mpc_nav::msg::People2D>();
    people2d_msg->stamp = msg->header.stamp;

    for (const auto & person : msg->people)
    {
      social_mpc_nav::msg::Person2D person2d;
      person2d.name = person.name;
      person2d.x = static_cast<float>(person.position.x);
      person2d.y = static_cast<float>(person.position.y);
      person2d.vx = static_cast<float>(person.velocity.x);
      person2d.vy = static_cast<float>(person.velocity.y);

      people2d_msg->people.push_back(person2d);

      if (log_to_csv_)
      {
        logCsvRow(msg->header.stamp, person2d);
      }
    }

    people_pub_->publish(*people2d_msg);

    // Throttled logging for debugging
    if (people2d_msg->people.size() > 0)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Converted %zu people from %s to %s",
        people2d_msg->people.size(),
        input_topic_.c_str(),
        output_topic_.c_str());
    }
  }

  void createLogFile()
  {
    std::error_code ec;
    std::filesystem::create_directories(log_directory_, ec);
    if (ec)
    {
      RCLCPP_WARN(
        get_logger(), "Failed to create log directory '%s': %s",
        log_directory_.c_str(), ec.message().c_str());
      log_to_csv_ = false;
      return;
    }

    std::filesystem::path file_path(log_directory_);
    file_path /= "people_adapter.csv";

    csv_stream_.open(file_path, std::ios::out | std::ios::app);
    if (!csv_stream_.good())
    {
      RCLCPP_WARN(
        get_logger(), "Failed to open people_adapter.csv for writing at '%s'.",
        file_path.string().c_str());
      log_to_csv_ = false;
      return;
    }

    if (csv_stream_.tellp() == 0)
    {
      csv_stream_ << "stamp_sec,person_name,x,y,vx,vy\n";
    }

    RCLCPP_INFO(get_logger(), "People adapter logging enabled: %s",
                file_path.string().c_str());
  }

  void logCsvRow(
    const builtin_interfaces::msg::Time & stamp,
    const social_mpc_nav::msg::Person2D & person)
  {
    if (!log_to_csv_ || !csv_stream_.good())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(log_mutex_);
    const double stamp_sec = stamp.sec + stamp.nanosec * 1e-9;

    csv_stream_ << std::fixed << std::setprecision(3)
                << stamp_sec << ","
                << person.name << ","
                << person.x << ","
                << person.y << ","
                << person.vx << ","
                << person.vy << "\n";
    csv_stream_.flush();
  }

  void discoverActorFrames()
  {
    if (!tf_buffer_)
    {
      return;
    }

    std::vector<std::string> discovered_actors;

    // Try to find actor frames by checking transforms
    for (int i = 1; i <= max_actors_to_check_; ++i)
    {
      std::string actor_frame = actor_frame_prefix_ + std::to_string(i);

      try
      {
        // Try to look up transform to see if this actor exists
        tf_buffer_->lookupTransform(
          map_frame_,
          actor_frame,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(0.1));

        // If we get here, the transform exists
        discovered_actors.push_back(actor_frame);
      }
      catch (const tf2::TransformException &)
      {
        // This actor frame doesn't exist, continue checking
      }
    }

    // Update actor_frames_ if the list changed
    std::lock_guard<std::mutex> lock(actor_mutex_);
    if (discovered_actors != actor_frames_)
    {
      size_t old_count = actor_frames_.size();
      actor_frames_ = discovered_actors;

      RCLCPP_INFO(
        get_logger(),
        "Actor discovery: Found %zu actors (was %zu) matching pattern '%s*'",
        actor_frames_.size(), old_count, actor_frame_prefix_.c_str());

      // Log the discovered actors
      std::string actor_list;
      for (const auto & actor : actor_frames_)
      {
        if (!actor_list.empty()) actor_list += ", ";
        actor_list += actor;
      }
      RCLCPP_INFO(get_logger(), "Active actors: %s", actor_list.c_str());
    }
  }

  void publishPeopleFromTF()
  {
    auto people2d_msg = std::make_shared<social_mpc_nav::msg::People2D>();
    people2d_msg->stamp = now();

    // Copy actor frames under lock to avoid data races
    std::vector<std::string> current_actors;
    {
      std::lock_guard<std::mutex> lock(actor_mutex_);
      current_actors = actor_frames_;
    }

    for (const auto & actor_frame : current_actors)
    {
      try
      {
        // Look up transform from map to actor
        geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(
            map_frame_,
            actor_frame,
            rclcpp::Time(0));  // Get latest

        social_mpc_nav::msg::Person2D person;
        person.name = actor_frame;
        person.x = static_cast<float>(transform_stamped.transform.translation.x);
        person.y = static_cast<float>(transform_stamped.transform.translation.y);

        // Calculate velocity from position changes (simple numerical derivative)
        // TODO: Could get this from twist if available
        person.vx = 0.0f;
        person.vy = 0.0f;

        people2d_msg->people.push_back(person);

        if (log_to_csv_)
        {
          logCsvRow(people2d_msg->stamp, person);
        }
      }
      catch (const tf2::TransformException & ex)
      {
        RCLCPP_DEBUG(get_logger(),
          "Could not get transform for %s: %s", actor_frame.c_str(), ex.what());
        // Skip this actor if transform not available
      }
    }

    // Only publish if we found at least one actor
    if (!people2d_msg->people.empty())
    {
      people_pub_->publish(*people2d_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Published %zu people from TF tree to %s",
        people2d_msg->people.size(), output_topic_.c_str());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string map_frame_;
  std::string log_directory_;
  std::string actor_frame_prefix_;
  int max_actors_to_check_{50};
  bool log_to_csv_{true};
  bool use_tf_for_positions_{true};
  std::vector<std::string> actor_frames_;

  rclcpp::Publisher<social_mpc_nav::msg::People2D>::SharedPtr people_pub_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex log_mutex_;
  std::mutex actor_mutex_;
  std::ofstream csv_stream_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeopleAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
