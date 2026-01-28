#include <algorithm>
#include <functional>
#include <string>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"

#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/msg/person2_d.hpp"

// Gazebo Transport for direct physics-based pose subscription
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

/**
 * @brief Node that extracts robot and pedestrian positions from Gazebo.
 *
 * New approach:
 * - Uses Gazebo Transport to directly subscribe to /world/default/pose/info
 * - Gets physics-based robot pose that accounts for collisions
 * - Publishes ONLY map -> odom transform (not map -> base_footprint)
 * - Respects existing odom -> base_footprint from wheel odometry
 * - Final chain: map -> odom -> base_footprint (accurate position)
 */
class GTLocalizationNode : public rclcpp::Node
{
public:
  GTLocalizationNode()
  : Node("gt_localization_node")
  {
    robot_model_name_ = declare_parameter<std::string>("robot_model_name", "tiago");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    gazebo_world_name_ = declare_parameter<std::string>("gazebo_world_name", "default");

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    people_pub_ = create_publisher<social_mpc_nav::msg::People2D>("/crowd/people2d", rclcpp::QoS(10));

    // Subscribe to Gazebo pose topic for physics-based positions
    std::string gz_pose_topic = "/world/" + gazebo_world_name_ + "/pose/info";
    if (!gz_node_.Subscribe(gz_pose_topic, &GTLocalizationNode::onGazeboPoses, this))
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to subscribe to Gazebo topic '%s'. Will fall back to TF tree.",
        gz_pose_topic.c_str());

      // Only use timer fallback if Gazebo subscription fails
      timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&GTLocalizationNode::publishTFFallback, this));
    }
    else
    {
      RCLCPP_INFO(
        get_logger(),
        "Subscribed to Gazebo topic '%s' for physics-based poses.",
        gz_pose_topic.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "gt_localization_node started. Using Gazebo physics-based pose with TF fallback. "
      "Publishing map->%s TF.",
      odom_frame_.c_str());
  }

private:
  // Gazebo Transport callback for physics-based poses
  // Publishes TF immediately when fresh data arrives
  void onGazeboPoses(const gz::msgs::Pose_V &msg)
  {
    // Get timestamp from Gazebo message
    rclcpp::Time stamp;
    if (msg.has_header() && msg.header().has_stamp())
    {
      stamp = rclcpp::Time(msg.header().stamp().sec(), msg.header().stamp().nsec());
    }
    else
    {
      // Fallback to current time if no timestamp in message
      stamp = get_clock()->now();
    }
    
    // Extract robot pose from Gazebo
    bool robot_found = false;
    geometry_msgs::msg::Pose robot_world_pose;

    for (int i = 0; i < msg.pose_size(); ++i)
    {
      const auto &pose = msg.pose(i);
      if (pose.name() == robot_model_name_)
      {
        robot_world_pose.position.x = pose.position().x();
        robot_world_pose.position.y = pose.position().y();
        robot_world_pose.position.z = pose.position().z();
        robot_world_pose.orientation.x = pose.orientation().x();
        robot_world_pose.orientation.y = pose.orientation().y();
        robot_world_pose.orientation.z = pose.orientation().z();
        robot_world_pose.orientation.w = pose.orientation().w();
        robot_found = true;
        break;
      }
    }

    if (!robot_found)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Robot '%s' not found in Gazebo poses.",
        robot_model_name_.c_str());
      return;
    }

    // Compute map -> odom correction based on physics pose
    // The robot publishes odom -> base_footprint (wheel odometry)
    // We publish map -> odom (correction to match Gazebo physics)
    // Final chain: map -> odom -> base_footprint (accurate!)
    //
    // Note: Even if the physics pose jumps (e.g., robot reset/teleport), the TF chain remains stable:
    // - odom→base_footprint (wheel odometry) continues smoothly
    // - map→odom will jump to reflect the new physics position
    // - This is the expected and correct behavior

    try
    {
      // Get current odom -> base_footprint transform (wheel odometry)
      // lookupTransform(target, source) - we want base in odom frame
      geometry_msgs::msg::TransformStamped odom_to_base = tf_buffer_->lookupTransform(
        odom_frame_,           // target: express result in odom frame
        base_link_frame_,      // source: position of base_footprint
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.1));

      // Compute map -> odom = physics_pose * inverse(odom_to_base)
      // physics_pose is where robot actually is (in map frame)
      // odom_to_base is where wheel odometry thinks robot is (relative to odom)
      // We need to find where odom should be in map frame

      tf2::Transform tf_physics;
      tf2::fromMsg(robot_world_pose, tf_physics);

      tf2::Transform tf_odom_to_base;
      tf2::fromMsg(odom_to_base.transform, tf_odom_to_base);

      // map_to_base = map_to_odom * odom_to_base
      // Therefore: map_to_odom = map_to_base * inverse(odom_to_base)
      tf2::Transform tf_map_to_odom = tf_physics * tf_odom_to_base.inverse();

      // Publish map -> odom transform
      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = stamp;
      map_to_odom.header.frame_id = map_frame_;
      map_to_odom.child_frame_id = odom_frame_;
      map_to_odom.transform = tf2::toMsg(tf_map_to_odom);

      tf_broadcaster_->sendTransform(map_to_odom);

      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Using Gazebo physics-based pose at (%.2f, %.2f, %.2f)",
        robot_world_pose.position.x, robot_world_pose.position.y, robot_world_pose.position.z);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not compute map->odom from physics: %s. Using identity.",
        ex.what());

      // Fallback: publish identity map -> odom
      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = stamp;
      map_to_odom.header.frame_id = map_frame_;
      map_to_odom.child_frame_id = odom_frame_;
      map_to_odom.transform.translation.x = 0.0;
      map_to_odom.transform.translation.y = 0.0;
      map_to_odom.transform.translation.z = 0.0;
      map_to_odom.transform.rotation.w = 1.0;
      map_to_odom.transform.rotation.x = 0.0;
      map_to_odom.transform.rotation.y = 0.0;
      map_to_odom.transform.rotation.z = 0.0;
      tf_broadcaster_->sendTransform(map_to_odom);
    }

    // Extract actors (pedestrians) for people detection
    social_mpc_nav::msg::People2D people_msg;
    for (int i = 0; i < msg.pose_size(); ++i)
    {
      const auto &pose = msg.pose(i);
      const std::string &name = pose.name();

      // Check if this is an actor (pedestrian)
      if (name.find("actor") != std::string::npos ||
          name.find("Patient") != std::string::npos ||
          name.find("Visitor") != std::string::npos ||
          name.find("Lady") != std::string::npos ||
          name.find("Male") != std::string::npos)
      {
        if (name == robot_model_name_) continue;

        social_mpc_nav::msg::Person2D person;
        person.name = name;
        person.x = static_cast<float>(pose.position().x());
        person.y = static_cast<float>(pose.position().y());
        person.vx = 0.0f;  // Pose_V doesn't include velocity
        person.vy = 0.0f;
        people_msg.people.push_back(person);
      }
    }

    // Publish people if available
    if (!people_msg.people.empty())
    {
      auto people_pub_msg = std::make_shared<social_mpc_nav::msg::People2D>();
      people_pub_msg->stamp.sec = static_cast<int32_t>(stamp.seconds());
      people_pub_msg->stamp.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000);
      people_pub_msg->people = people_msg.people;
      people_pub_->publish(*people_pub_msg);

      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Published %zu actors from Gazebo | First: (%s at %.2f, %.2f)",
        people_pub_msg->people.size(),
        people_pub_msg->people[0].name.c_str(),
        people_pub_msg->people[0].x,
        people_pub_msg->people[0].y);
    }
  }

  // Fallback TF publishing (only used when Gazebo subscription fails)
  // Uses TF tree to get robot pose from wheel odometry
  void publishTFFallback()
  {
    auto stamp = get_clock()->now();

    // FALLBACK: Use TF tree (wheel odometry - may drift when stuck)
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Using TF tree fallback (wheel odometry). Gazebo physics pose not available.");

    // Publish identity map -> odom as basic fallback
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;
    map_to_odom.transform.rotation.x = 0.0;
    map_to_odom.transform.rotation.y = 0.0;
    map_to_odom.transform.rotation.z = 0.0;
    tf_broadcaster_->sendTransform(map_to_odom);
  }

  std::string robot_model_name_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::string world_frame_;
  std::string gazebo_world_name_;

  // Gazebo Transport
  gz::transport::Node gz_node_;

  // ROS 2 publishers and TF
  rclcpp::Publisher<social_mpc_nav::msg::People2D>::SharedPtr people_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;  // Only used for fallback
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GTLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}

