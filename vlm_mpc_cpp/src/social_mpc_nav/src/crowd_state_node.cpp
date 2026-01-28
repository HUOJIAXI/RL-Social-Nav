#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "gazebo_msgs/msg/model_states.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/msg/person2_d.hpp"

/**
 * @brief Node that extracts people states from Gazebo model ground-truth.
 *
 * Velocities are estimated via finite differences per-person to keep the prototype simple.
 */
class CrowdStateNode : public rclcpp::Node
{
public:
  CrowdStateNode()
  : Node("crowd_state_node")
  {
    person_prefix_ = declare_parameter<std::string>("person_name_prefix", "person");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_topic_ = declare_parameter<std::string>("publish_topic", "/crowd/people2d");
    const char * home = std::getenv("HOME");
    const std::string default_log = home ? std::string(home) + "/ros2_logs/social_mpc_nav" : "/tmp/social_mpc_nav";
    log_directory_ = declare_parameter<std::string>("log_directory", default_log);
    log_to_csv_ = declare_parameter<bool>("log_crowd_to_csv", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    if (publish_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be > 0. Resetting to 10 Hz.");
      publish_rate_hz_ = 10.0;
    }
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/gazebo/model_states");

    // Initialize TF2 for coordinate frame transformations
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    people_pub_ = create_publisher<social_mpc_nav::msg::People2D>(publish_topic_, rclcpp::QoS(10));
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, rclcpp::QoS(10),
      std::bind(&CrowdStateNode::onModelStates, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      timer_period,
      std::bind(&CrowdStateNode::publishPeople, this));

    if (log_to_csv_)
    {
      createLogFile();
    }

    RCLCPP_INFO(
      get_logger(),
      "crowd_state_node publishing to '%s' at %.1f Hz using prefix '%s'. "
      "Transforming from '%s' to '%s' frame.",
      publish_topic_.c_str(), publish_rate_hz_, person_prefix_.c_str(),
      world_frame_.c_str(), map_frame_.c_str());
  }

private:
  struct PersonHistory
  {
    double x{0.0};
    double y{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool initialized{false};
  };

  void onModelStates(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_msg_ = msg;
  }

  void publishPeople()
  {
    gazebo_msgs::msg::ModelStates::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      msg = latest_msg_;
    }

    if (!msg)
    {
      return;
    }

    // Get transform from world frame to map frame
    // Try multiple paths: world -> map (direct), world -> odom -> map, world -> base_link -> odom -> map
    geometry_msgs::msg::TransformStamped world_to_map;
    bool transform_available = false;
    
    // Try direct transform first
    try
    {
      world_to_map = tf_buffer_->lookupTransform(
        map_frame_,                    // target frame: map
        world_frame_,                  // source frame: world (Gazebo)
        rclcpp::Time(0));             // get latest available transform
      transform_available = true;
    }
    catch (const tf2::TransformException & ex1)
    {
      // Try via odom frame: world -> odom -> map
      try
      {
        // First get world -> odom
        geometry_msgs::msg::TransformStamped world_to_odom = tf_buffer_->lookupTransform(
          odom_frame_,                 // target frame: odom
          world_frame_,                 // source frame: world
          rclcpp::Time(0));
        
        // Then get odom -> map
        geometry_msgs::msg::TransformStamped odom_to_map = tf_buffer_->lookupTransform(
          map_frame_,                  // target frame: map
          odom_frame_,                 // source frame: odom
          rclcpp::Time(0));
        
        // Compose the transforms: world -> odom -> map
        geometry_msgs::msg::PoseStamped pose_world_temp;
        pose_world_temp.header.frame_id = world_frame_;
        pose_world_temp.pose.position.x = 0.0;
        pose_world_temp.pose.position.y = 0.0;
        pose_world_temp.pose.position.z = 0.0;
        pose_world_temp.pose.orientation.w = 1.0;
        
        geometry_msgs::msg::PoseStamped pose_odom_temp;
        tf2::doTransform(pose_world_temp, pose_odom_temp, world_to_odom);
        
        geometry_msgs::msg::PoseStamped pose_map_temp;
        tf2::doTransform(pose_odom_temp, pose_map_temp, odom_to_map);
        
        // Create composed transform
        world_to_map.header.frame_id = world_frame_;
        world_to_map.child_frame_id = map_frame_;
        world_to_map.transform.translation.x = pose_map_temp.pose.position.x;
        world_to_map.transform.translation.y = pose_map_temp.pose.position.y;
        world_to_map.transform.translation.z = pose_map_temp.pose.position.z;
        world_to_map.transform.rotation = pose_map_temp.pose.orientation;
        transform_available = true;
        
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Found world -> map transform via odom frame");
      }
      catch (const tf2::TransformException & ex2)
      {
        // Try via base_link: world -> base_link -> odom -> map
        try
        {
          // Try to find any robot base frame (common names)
          std::vector<std::string> possible_base_frames = {
            "base_link", "base_footprint", "tiago_official/base_link", "tiago_official/base_footprint"
          };
          
          std::string found_base_frame;
          geometry_msgs::msg::TransformStamped world_to_base;
          
          for (const auto & base_frame : possible_base_frames)
          {
            try
            {
              world_to_base = tf_buffer_->lookupTransform(
                base_frame,            // target frame: base_link
                world_frame_,          // source frame: world
                rclcpp::Time(0));
              found_base_frame = base_frame;
              break;
            }
            catch (const tf2::TransformException &)
            {
              continue;
            }
          }
          
          if (!found_base_frame.empty())
          {
            // Get base -> odom
            geometry_msgs::msg::TransformStamped base_to_odom = tf_buffer_->lookupTransform(
              odom_frame_,             // target frame: odom
              found_base_frame,        // source frame: base_link
              rclcpp::Time(0));
            
            // Get odom -> map
            geometry_msgs::msg::TransformStamped odom_to_map = tf_buffer_->lookupTransform(
              map_frame_,              // target frame: map
              odom_frame_,             // source frame: odom
              rclcpp::Time(0));
            
            // Compose: world -> base -> odom -> map
            geometry_msgs::msg::PoseStamped pose_world_temp;
            pose_world_temp.header.frame_id = world_frame_;
            pose_world_temp.pose.position.x = 0.0;
            pose_world_temp.pose.position.y = 0.0;
            pose_world_temp.pose.position.z = 0.0;
            pose_world_temp.pose.orientation.w = 1.0;
            
            geometry_msgs::msg::PoseStamped pose_base_temp;
            tf2::doTransform(pose_world_temp, pose_base_temp, world_to_base);
            
            geometry_msgs::msg::PoseStamped pose_odom_temp;
            tf2::doTransform(pose_base_temp, pose_odom_temp, base_to_odom);
            
            geometry_msgs::msg::PoseStamped pose_map_temp;
            tf2::doTransform(pose_odom_temp, pose_map_temp, odom_to_map);
            
            // Create composed transform
            world_to_map.header.frame_id = world_frame_;
            world_to_map.child_frame_id = map_frame_;
            world_to_map.transform.translation.x = pose_map_temp.pose.position.x;
            world_to_map.transform.translation.y = pose_map_temp.pose.position.y;
            world_to_map.transform.translation.z = pose_map_temp.pose.position.z;
            world_to_map.transform.rotation = pose_map_temp.pose.orientation;
            transform_available = true;
            
            RCLCPP_DEBUG_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Found world -> map transform via %s -> odom -> map", found_base_frame.c_str());
          }
          else
          {
            throw tf2::TransformException("Could not find base_link frame");
          }
        }
        catch (const tf2::TransformException & ex3)
        {
          // Last resort: try with parameterized odom frame name
          try
          {
            // Try with parameterized odom frame name
            geometry_msgs::msg::TransformStamped world_to_odom = tf_buffer_->lookupTransform(
              odom_frame_,             // target frame: odom (parameterized)
              world_frame_,            // source frame: world
              rclcpp::Time(0));
            
            geometry_msgs::msg::TransformStamped odom_to_map = tf_buffer_->lookupTransform(
              map_frame_,              // target frame: map
              odom_frame_,             // source frame: odom (parameterized)
              rclcpp::Time(0));
            
            // Compose transforms
            geometry_msgs::msg::PoseStamped pose_world_temp;
            pose_world_temp.header.frame_id = world_frame_;
            pose_world_temp.pose.position.x = 0.0;
            pose_world_temp.pose.position.y = 0.0;
            pose_world_temp.pose.position.z = 0.0;
            pose_world_temp.pose.orientation.w = 1.0;
            
            geometry_msgs::msg::PoseStamped pose_odom_temp;
            tf2::doTransform(pose_world_temp, pose_odom_temp, world_to_odom);
            
            geometry_msgs::msg::PoseStamped pose_map_temp;
            tf2::doTransform(pose_odom_temp, pose_map_temp, odom_to_map);
            
            world_to_map.header.frame_id = world_frame_;
            world_to_map.child_frame_id = map_frame_;
            world_to_map.transform.translation.x = pose_map_temp.pose.position.x;
            world_to_map.transform.translation.y = pose_map_temp.pose.position.y;
            world_to_map.transform.translation.z = pose_map_temp.pose.position.z;
            world_to_map.transform.rotation = pose_map_temp.pose.orientation;
            transform_available = true;
          }
          catch (const tf2::TransformException & ex4)
          {
            // All transform attempts failed - use identity as fallback
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Could not find transform from '%s' to '%s' via any path. "
              "Tried: direct, via odom, via base_link. Error: %s. "
              "Using identity transform (may be incorrect if frames differ!).",
              world_frame_.c_str(), map_frame_.c_str(), ex4.what());
            
            // Set identity transform as fallback
            world_to_map.header.frame_id = world_frame_;
            world_to_map.child_frame_id = map_frame_;
            world_to_map.transform.translation.x = 0.0;
            world_to_map.transform.translation.y = 0.0;
            world_to_map.transform.translation.z = 0.0;
            world_to_map.transform.rotation.w = 1.0;
            world_to_map.transform.rotation.x = 0.0;
            world_to_map.transform.rotation.y = 0.0;
            world_to_map.transform.rotation.z = 0.0;
            transform_available = true;  // Use identity transform (may be wrong!)
          }
        }
      }
    }

    auto people_msg = std::make_shared<social_mpc_nav::msg::People2D>();
    const rclcpp::Time stamp = now();
    people_msg->stamp = stamp;

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      const auto & name = msg->name[i];
      if (name.find(person_prefix_) == std::string::npos)
      {
        continue;
      }

      // Get person position in world frame (Gazebo coordinates)
      const double x_world = msg->pose[i].position.x;
      const double y_world = msg->pose[i].position.y;

      // Transform position from world frame to map frame
      geometry_msgs::msg::PoseStamped pose_world;
      pose_world.header.frame_id = world_frame_;
      pose_world.header.stamp = stamp;
      pose_world.pose.position.x = x_world;
      pose_world.pose.position.y = y_world;
      pose_world.pose.position.z = msg->pose[i].position.z;
      pose_world.pose.orientation = msg->pose[i].orientation;

      geometry_msgs::msg::PoseStamped pose_map;
      if (transform_available)
      {
        tf2::doTransform(pose_world, pose_map, world_to_map);
      }
      else
      {
        // Fallback: use world coordinates directly (may be incorrect!)
        pose_map = pose_world;
      }

      const double x_map = pose_map.pose.position.x;
      const double y_map = pose_map.pose.position.y;

      // Calculate velocity in map frame
      // Note: If there's rotation between frames, velocity should also be rotated
      // For simplicity, we calculate velocity from transformed positions
      double vx = 0.0;
      double vy = 0.0;
      auto & history = histories_[name];
      if (history.initialized)
      {
        const double dt = (stamp - history.stamp).seconds();
        if (dt > 1e-3)
        {
          // Velocity is calculated from transformed positions, so it's already in map frame
          vx = (x_map - history.x) / dt;
          vy = (y_map - history.y) / dt;
        }
      }

      history = PersonHistory{x_map, y_map, stamp, true};

      social_mpc_nav::msg::Person2D person;
      person.name = name;
      person.x = static_cast<float>(x_map);
      person.y = static_cast<float>(y_map);
      person.vx = static_cast<float>(vx);
      person.vy = static_cast<float>(vy);
      people_msg->people.push_back(person);

      if (log_to_csv_)
      {
        logCsvRow(people_msg->stamp, person);
      }
    }

    people_pub_->publish(*people_msg);
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
    file_path /= "crowd_state.csv";

    csv_stream_.open(file_path, std::ios::out | std::ios::app);
    if (!csv_stream_.good())
    {
      RCLCPP_WARN(
        get_logger(), "Failed to open crowd_state.csv for writing at '%s'.",
        file_path.string().c_str());
      log_to_csv_ = false;
      return;
    }

    if (csv_stream_.tellp() == 0)
    {
      csv_stream_ << "stamp_sec,person,x,y,vx,vy\n";
    }
  }

  void logCsvRow(const rclcpp::Time & stamp, const social_mpc_nav::msg::Person2D & person)
  {
    if (!log_to_csv_ || !csv_stream_.good())
    {
      return;
    }
    csv_stream_ << std::fixed << std::setprecision(3)
                << stamp.seconds() << ","
                << person.name << ","
                << person.x << ","
                << person.y << ","
                << person.vx << ","
                << person.vy << "\n";
    csv_stream_.flush();
  }

  std::string person_prefix_;
  std::string map_frame_;
  std::string world_frame_;
  std::string odom_frame_;
  std::string publish_topic_;
  std::string log_directory_;
  std::string model_states_topic_;
  bool log_to_csv_{true};
  double publish_rate_hz_{10.0};

  // TF2 for coordinate frame transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<social_mpc_nav::msg::People2D>::SharedPtr people_pub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unordered_map<std::string, PersonHistory> histories_;
  gazebo_msgs::msg::ModelStates::SharedPtr latest_msg_;
  std::mutex data_mutex_;

  std::ofstream csv_stream_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrowdStateNode>());
  rclcpp::shutdown();
  return 0;
}

