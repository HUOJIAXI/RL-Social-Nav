#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "person_tracker/msg/person_info_array.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/social_contract.hpp"

using social_mpc_nav::SocialContract;
using social_mpc_nav::SocialContractHelper;

namespace
{
struct RobotState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};
}  // namespace

/**
 * @brief Minimal social MPC controller for TIAGo differential drive.
 *
 * The controller implements a random-shooting MPC:
 *  - Sample candidate control sequences (v, omega) over the planning horizon.
 *  - Roll out the unicycle model and evaluate the cost.
 *  - Apply the first control of the best sequence.
 *
 * This keeps the implementation dependency-free while leaving clear seams for future solvers.
 */
class MPCControllerNode : public rclcpp::Node
{
public:
  MPCControllerNode()
  : Node("mpc_controller_node"),
    contract_helper_(get_logger(),
      declare_parameter<std::string>(
        "log_directory",
        []() {
          const char * home = std::getenv("HOME");
          return home ? std::string(home) + "/ros2_logs/social_mpc_nav" : "/tmp/social_mpc_nav";
        }()),
      declare_parameter<bool>("log_social_contract_to_csv", true))
  {
    goal_x_ = declare_parameter<double>("goal_x", 2.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.3);  // 0.3m = 30cm tolerance
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);
    if (control_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "control_rate_hz must be > 0. Resetting to 10 Hz.");
      control_rate_hz_ = 10.0;
    }
    dt_ = declare_parameter<double>("dt", 0.2);
    horizon_steps_ = declare_parameter<int>("N", 15);
    default_v_max_ = declare_parameter<double>("default_v_max", 0.6);
    omega_max_ = declare_parameter<double>("omega_max", 0.9);
    smooth_weight_ = declare_parameter<double>("w_smooth", 0.1);
    num_rollouts_ = declare_parameter<int>("num_rollouts", 60);
    log_directory_ = get_parameter("log_directory").as_string();
    log_mpc_to_csv_ = declare_parameter<bool>("log_mpc_to_csv", true);
    crowd_topic_ = declare_parameter<std::string>("crowd_topic", "/person_tracker/person_info");
    obstacle_weight_ = declare_parameter<double>("w_obstacle", 3.0);
    min_obstacle_distance_ = declare_parameter<double>("min_obstacle_distance", 0.3);
    min_valid_laser_range_ = declare_parameter<double>("min_valid_laser_range", 0.15);
    safety_check_enabled_ = declare_parameter<bool>("safety_check_enabled", true);
    enable_debug_logging_ = declare_parameter<bool>("enable_debug_logging", false);

    // Declare topic parameters for robot-specific configuration
    std::string odom_topic = declare_parameter<std::string>(
      "odom_topic", "/task_generator_node/tiago_official/odom");
    std::string cmd_vel_topic = declare_parameter<std::string>(
      "cmd_vel_topic", "/task_generator_node/tiago_official/cmd_vel");
    std::string scan_topic = declare_parameter<std::string>(
      "scan_topic", "/scan_2d");
    std::string global_path_topic = declare_parameter<std::string>(
      "global_path_topic", "/global_path");

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10),
      std::bind(&MPCControllerNode::onOdom, this, std::placeholders::_1));
    crowd_sub_ = create_subscription<person_tracker::msg::PersonInfoArray>(
      crowd_topic_, rclcpp::QoS(10),
      std::bind(&MPCControllerNode::onPersonInfo, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),  // Use sensor QoS (BEST_EFFORT) for converted Velodyne data
      std::bind(&MPCControllerNode::onScan, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic, rclcpp::QoS(10).transient_local(),
      std::bind(&MPCControllerNode::onPath, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      std::bind(&MPCControllerNode::onGoalPose, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      period,
      std::bind(&MPCControllerNode::controlLoop, this));

    createMpcLog();
    createObstacleLog();
    createPedestrianLog();

    int seed = declare_parameter<int>("random_seed", 42);
    rng_.seed(seed);

    // Initialize TF2 for coordinate frame transformations
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare coordinate frame parameters
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

    RCLCPP_INFO(
      get_logger(),
      "mpc_controller_node started. Goal: (%.2f, %.2f) dt=%.2f N=%d rollouts=%d",
      goal_x_, goal_y_, dt_, horizon_steps_, num_rollouts_);
  }

  /**
   * @brief Stops the robot by publishing zero velocity
   */
  void stopRobot()
  {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.linear.z = 0.0;
    stop_cmd.angular.x = 0.0;
    stop_cmd.angular.y = 0.0;
    stop_cmd.angular.z = 0.0;

    cmd_pub_->publish(stop_cmd);
    RCLCPP_INFO(get_logger(), "Robot stopped - zero velocity published");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_ = msg;
  }

  void onPersonInfo(const person_tracker::msg::PersonInfoArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Convert PersonInfoArray to People2D for internal use
    auto people_msg = std::make_shared<social_mpc_nav::msg::People2D>();
    people_msg->stamp = msg->header.stamp;

    for (const auto & person : msg->persons)
    {
      social_mpc_nav::msg::Person2D person_2d;
      person_2d.name = std::to_string(person.person_id);
      person_2d.x = static_cast<float>(person.position.x);
      person_2d.y = static_cast<float>(person.position.y);
      person_2d.vx = static_cast<float>(person.velocity.x);
      person_2d.vy = static_cast<float>(person.velocity.y);
      people_msg->people.push_back(person_2d);
    }

    latest_people_ = people_msg;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = msg;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    global_path_ = msg;
    path_received_ = true;
    current_waypoint_index_ = 0;  // Reset to start of new path

    if (msg && !msg->poses.empty())
    {
      RCLCPP_INFO(
        get_logger(),
        "Received global path with %zu waypoints. Final goal: (%.2f, %.2f)",
        msg->poses.size(),
        msg->poses.back().pose.position.x,
        msg->poses.back().pose.position.y);
    }
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Transform goal pose to map frame if needed
    geometry_msgs::msg::PoseStamped goal_in_map;
    try
    {
      if (msg->header.frame_id.empty() || msg->header.frame_id == map_frame_)
      {
        // Goal is already in map frame or frame_id is not specified (assume map frame)
        goal_in_map = *msg;
      }
      else
      {
        // Transform to map frame
        tf_buffer_->transform(*msg, goal_in_map, map_frame_, tf2::durationFromSec(0.1));
      }

      // Update goal position
      goal_x_ = goal_in_map.pose.position.x;
      goal_y_ = goal_in_map.pose.position.y;

      // Extract goal orientation (yaw) from quaternion
      goal_yaw_ = tf2::getYaw(goal_in_map.pose.orientation);

      // Reset goal reached flag to allow navigation to new goal
      goal_reached_ = false;

      // Enable navigation now that we have a goal from RViz2
      goal_received_ = true;

      RCLCPP_INFO(
        get_logger(),
        "🎯 New goal received from RViz2: (%.2f, %.2f, yaw: %.2f°) in %s frame - Starting navigation",
        goal_x_, goal_y_, goal_yaw_ * 180.0 / M_PI, map_frame_.c_str());
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to transform goal pose from %s to %s: %s",
        msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    }
  }

  bool getLatestData(
    RobotState & state,
    social_mpc_nav::msg::People2D::SharedPtr & people,
    sensor_msgs::msg::LaserScan::SharedPtr & scan)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_odom_)
    {
      return false;
    }

    // Get robot position in odom frame
    double x_odom = latest_odom_->pose.pose.position.x;
    double y_odom = latest_odom_->pose.pose.position.y;
    double yaw_odom = tf2::getYaw(latest_odom_->pose.pose.orientation);

    // Transform robot position from odom frame to map frame
    try
    {
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

      // Use transformed position
      state.x = pose_map.pose.position.x;
      state.y = pose_map.pose.position.y;
      state.yaw = tf2::getYaw(pose_map.pose.orientation);

      RCLCPP_DEBUG(get_logger(),
        "Robot in map frame: (%.2f, %.2f) [transformed from odom: (%.2f, %.2f)]",
        state.x, state.y, x_odom, y_odom);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Could not transform from %s to %s: %s. Using odom position (incorrect!)",
        odom_frame_.c_str(), map_frame_.c_str(), ex.what());

      // Fallback to odom position (will be incorrect if frames diverge!)
      state.x = x_odom;
      state.y = y_odom;
      state.yaw = yaw_odom;
    }

    people = latest_people_;
    scan = latest_scan_;
    return true;
  }

  void controlLoop()
  {
    RobotState robot;
    social_mpc_nav::msg::People2D::SharedPtr people;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    if (!getLatestData(robot, people, scan))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odom...");
      return;
    }

    // Wait for goal from RViz2 before starting navigation
    if (!goal_received_)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "⏸️  Waiting for goal from RViz2 (use 2D Goal Pose tool)...");
      return;
    }

    // Get current target (either waypoint from path or final goal)
    double target_x = goal_x_;
    double target_y = goal_y_;
    bool tracking_waypoint = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (path_received_ && global_path_ && !global_path_->poses.empty())
      {
        // Find nearest waypoint ahead of robot
        updateCurrentWaypoint(robot.x, robot.y);

        if (current_waypoint_index_ < global_path_->poses.size())
        {
          const auto& wp = global_path_->poses[current_waypoint_index_];
          target_x = wp.pose.position.x;
          target_y = wp.pose.position.y;
          tracking_waypoint = true;

          // Log waypoint tracking info
          const double dist_to_wp = std::hypot(robot.x - target_x, robot.y - target_y);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "📍 Tracking waypoint %zu/%zu at (%.2f, %.2f), distance: %.2fm",
            current_waypoint_index_, global_path_->poses.size()-1,
            target_x, target_y, dist_to_wp);
        }
      }
    }

    // Check if goal/waypoint is reached
    const double dist_to_target = std::hypot(robot.x - target_x, robot.y - target_y);
    const double dist_to_final_goal = std::hypot(robot.x - goal_x_, robot.y - goal_y_);

    // Calculate orientation error (goal - current) to turn TOWARD goal
    double yaw_error = goal_yaw_ - robot.yaw;
    // Normalize to [-π, π] using atan2 for numerical stability
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

    // Increased tolerance to avoid getting stuck at ±180° singularity
    const double orientation_tolerance = 0.175;  // ~10 degrees (was 5.7°)

    // Check if position reached
    if (dist_to_final_goal < goal_tolerance_)
    {
      // Position reached, now align orientation

      // Check for ±180° singularity ONLY when very close (within 5° of exactly opposite)
      // This catches 175-180° where oscillation actually occurs
      const double singularity_threshold = 0.087;  // ~5° margin (very tight!)
      if (std::abs(yaw_error) > (M_PI - singularity_threshold))  // Error > 175° or < -175°
      {
        if (!goal_reached_)
        {
          RCLCPP_WARN(
            get_logger(),
            "⚠️  Orientation %.1f° apart (exactly opposite) - skipping alignment to avoid oscillation",
            std::abs(yaw_error) * 180.0 / M_PI);
          RCLCPP_INFO(
            get_logger(),
            "✅ GOAL POSITION REACHED! Orientation alignment skipped (±180° singularity)");
          goal_reached_ = true;
        }

        // Keep publishing stop command
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_pub_->publish(stop_cmd);
        return;
      }

      if (std::abs(yaw_error) > orientation_tolerance)
      {
        // Perform in-place rotation to match goal orientation
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;  // No forward motion

        // Simple saturated proportional control with very low gains
        // This prevents oscillations by being gentle and using speed limits
        double k_rot = 0.5;  // Very gentle proportional gain
        double max_rotation_speed = 0.2;  // Very slow rotation (0.2 rad/s ≈ 11.5°/s)

        // Proportional control
        double desired_omega = k_rot * yaw_error;

        // Smooth saturation using tanh for gentle speed limiting
        // tanh provides smooth transitions without hard clipping
        double omega_normalized = desired_omega / max_rotation_speed;
        cmd.angular.z = max_rotation_speed * std::tanh(omega_normalized);

        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "🔄 Position reached, aligning orientation: %.1f° remaining (ω=%.2f rad/s)",
          std::abs(yaw_error) * 180.0 / M_PI, cmd.angular.z);
        return;
      }

      // Both position and orientation reached
      if (!goal_reached_)
      {
        RCLCPP_INFO(
          get_logger(),
          "✅ GOAL FULLY REACHED! Position: %.3fm, Orientation: %.1f° - Stopping robot",
          dist_to_final_goal, std::abs(yaw_error) * 180.0 / M_PI);
        goal_reached_ = true;
      }

      // Keep publishing stop command when at goal
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_pub_->publish(stop_cmd);
      return;
    }

    // Reset goal_reached flag if we've moved away from goal (e.g., new goal set)
    if (goal_reached_ && dist_to_final_goal > goal_tolerance_ * 2.0)
    {
      RCLCPP_INFO(get_logger(), "Resuming navigation - distance to goal: %.2fm", dist_to_final_goal);
      goal_reached_ = false;
    }

    // Log detected pedestrians
    if (people && !people->people.empty())
    {
      // Find nearest pedestrian for terminal logging
      double min_person_dist = std::numeric_limits<double>::infinity();
      double nearest_ped_x = 0.0;
      double nearest_ped_y = 0.0;
      for (const auto & person : people->people)
      {
        const double dist = std::hypot(robot.x - person.x, robot.y - person.y);
        if (dist < min_person_dist)
        {
          min_person_dist = dist;
          nearest_ped_x = person.x;
          nearest_ped_y = person.y;
        }
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "🚶 PEDESTRIAN AVOIDANCE: %zu pedestrian(s) detected | "
        "Robot at (%.2f, %.2f) | Nearest at %.2fm (%.2f, %.2f)",
        people->people.size(), robot.x, robot.y, min_person_dist, nearest_ped_x, nearest_ped_y);
    }

    // Log detected obstacles
    if (scan)
    {
      const double min_obstacle_dist = getMinObstacleDistanceFromCurrentPose(robot, scan);
      if (min_obstacle_dist < 2.0)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "🚧 OBSTACLE AVOIDANCE: Obstacle at %.2fm - planning avoidance maneuver", min_obstacle_dist);
      }
    }

    auto contract = contract_helper_.compute(
      robot.x, robot.y, robot.yaw, default_v_max_, people);

    // Reduce velocity when approaching final goal for smooth, precise arrival
    // Uses quadratic slowdown profile for smoother deceleration
    const double slowdown_distance = 2.5;  // Start slowing down at 2.5m from goal
    const double min_approach_velocity = 0.1;  // Minimum velocity when very close (0.1 m/s)
    if (dist_to_final_goal < slowdown_distance)
    {
      // Quadratic velocity reduction for smoother deceleration
      // v = v_min + (v_max - v_min) * (dist / slowdown_dist)^2
      // This creates gentle slowdown far away, aggressive slowdown near goal
      const double normalized_dist = dist_to_final_goal / slowdown_distance;
      const double velocity_scale = normalized_dist * normalized_dist;  // Quadratic profile

      contract.v_max = min_approach_velocity +
                       (contract.v_max - min_approach_velocity) * velocity_scale;

      // Extra slowdown in final 0.5m for very precise approach
      if (dist_to_final_goal < 0.5)
      {
        const double final_scale = dist_to_final_goal / 0.5;
        contract.v_max = std::min(contract.v_max, 0.2 * final_scale + 0.05);
      }

      RCLCPP_DEBUG(get_logger(),
        "Approaching goal (%.2fm) - reducing v_max from %.2f to %.2f (scale: %.2f)",
        dist_to_final_goal, default_v_max_, contract.v_max, velocity_scale);
    }

    // Detect circular motion (local minima)
    detectCircularMotion(robot, dist_to_final_goal);

    // Temporarily override goal for MPC to track waypoint
    const double original_goal_x = goal_x_;
    const double original_goal_y = goal_y_;
    goal_x_ = target_x;
    goal_y_ = target_y;

    const auto best_control = solveMpc(robot, people, scan, contract);
    if (!best_control.has_value())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC failed to find valid control");
      return;
    }

    // Restore original goal after MPC
    goal_x_ = original_goal_x;
    goal_y_ = original_goal_y;

    // Log MPC debug info more frequently
    if (enable_debug_logging_)
    {
      if (tracking_waypoint)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        RCLCPP_INFO(
          get_logger(),
          "[MPC Debug] Waypoint %zu/%zu: (%.2f, %.2f) | Robot: (%.2f, %.2f) | Dist: %.2fm | "
          "v=%.2f ω=%.2f | Final goal: (%.2f, %.2f) %.2fm",
          current_waypoint_index_ + 1, global_path_ ? global_path_->poses.size() : 0,
          target_x, target_y, robot.x, robot.y, dist_to_target,
          best_control->first, best_control->second,
          original_goal_x, original_goal_y, dist_to_final_goal);
      }
      else
      {
        RCLCPP_INFO(
          get_logger(),
          "[MPC Debug] Direct goal: (%.2f, %.2f) | Robot: (%.2f, %.2f) | Dist: %.2fm | "
          "v=%.2f ω=%.2f | Contract: v_max=%.2f w_goal=%.2f w_social=%.2f",
          target_x, target_y, robot.x, robot.y, dist_to_target,
          best_control->first, best_control->second,
          contract.v_max, contract.w_goal, contract.w_social);
      }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = best_control->first;
    cmd.angular.z = best_control->second;

    // Safety check: prevent collision with immediate obstacles
    if (safety_check_enabled_ && scan && isCollisionImminent(scan, cmd.linear.x))
    {
      const double front_dist = getMinFrontDistance(scan);
      RCLCPP_WARN(
        get_logger(),
        "⚠️  COLLISION IMMINENT! Obstacle at %.2fm ahead - EMERGENCY STOP", front_dist);
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
    else
    {
      // Log navigation status with context
      if (dist_to_final_goal < 0.5)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "🎯 Approaching goal: %.2fm remaining | v=%.2f m/s, ω=%.2f rad/s",
          dist_to_final_goal, cmd.linear.x, cmd.angular.z);
      }
      else
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Navigating to goal: %.2fm remaining | v=%.2f m/s, ω=%.2f rad/s",
          dist_to_final_goal, cmd.linear.x, cmd.angular.z);
      }
    }

    cmd_pub_->publish(cmd);
    last_cmd_ = {cmd.linear.x, cmd.angular.z};

    if (log_mpc_to_csv_)
    {
      logMpcStep(robot, people, contract, cmd);

      // Log obstacle avoidance if obstacles are present
      if (scan)
      {
        logObstacleAvoidance(robot, scan, cmd);
      }

      // Log pedestrian avoidance if pedestrians are present
      if (people && !people->people.empty())
      {
        logPedestrianAvoidance(robot, people, cmd);
      }
    }
  }

  std::optional<std::pair<double, double>> solveMpc(
    const RobotState & robot,
    const social_mpc_nav::msg::People2D::SharedPtr & people,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    const SocialContract & contract)
  {
    if (horizon_steps_ <= 0 || num_rollouts_ <= 0)
    {
      return std::nullopt;
    }

    std::uniform_real_distribution<double> vel_dist(0.0, contract.v_max);
    std::uniform_real_distribution<double> omega_dist(-omega_max_, omega_max_);

    double best_cost = std::numeric_limits<double>::infinity();
    std::pair<double, double> best_u{0.0, 0.0};

    for (int rollout = 0; rollout < num_rollouts_; ++rollout)
    {
      std::vector<double> v_seq(horizon_steps_);
      std::vector<double> w_seq(horizon_steps_);
      for (int k = 0; k < horizon_steps_; ++k)
      {
        v_seq[k] = vel_dist(rng_);
        w_seq[k] = omega_dist(rng_);
      }

      const double cost = evaluateSequence(robot, people, scan, contract, v_seq, w_seq);

      if (cost < best_cost)
      {
        best_cost = cost;
        best_u = {v_seq.front(), w_seq.front()};
      }
    }

    return best_u;
  }

  double evaluateSequence(
    const RobotState & robot,
    const social_mpc_nav::msg::People2D::SharedPtr & people,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    const SocialContract & contract,
    const std::vector<double> & v_seq,
    const std::vector<double> & w_seq)
  {
    RobotState pred = robot;
    double cost = 0.0;
    double prev_v = last_cmd_.first;
    double prev_w = last_cmd_.second;

    const double eps = 0.1;

    for (size_t k = 0; k < v_seq.size(); ++k)
    {
      const double v = v_seq[k];
      const double w = w_seq[k];

      pred.x += v * std::cos(pred.yaw) * dt_;
      pred.y += v * std::sin(pred.yaw) * dt_;
      pred.yaw += w * dt_;

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

      // Obstacle cost - repel from static obstacles detected by laser scan
      if (scan)
      {
        const double obstacle_dist = getMinObstacleDistance(robot, pred, scan);
        if (obstacle_dist < min_obstacle_distance_)
        {
          // Heavy penalty for collision
          cost += 1000.0;
        }
        else
        {
          // Inverse distance penalty for nearby obstacles
          const double capped_dist = std::max(obstacle_dist, 0.1);
          cost += obstacle_weight_ * (1.0 / (capped_dist + eps));
        }
      }

      // Smoothness
      cost += smooth_weight_ * (std::pow(v - prev_v, 2) + std::pow(w - prev_w, 2));
      prev_v = v;
      prev_w = w;
    }

    // Terminal goal cost - distance to goal
    const double dx_goal = pred.x - goal_x_;
    const double dy_goal = pred.y - goal_y_;
    const double dist_to_goal_sq = dx_goal * dx_goal + dy_goal * dy_goal;
    cost += contract.w_goal * dist_to_goal_sq;

    // Heading alignment cost - encourage robot to face toward the goal
    // This prevents the robot from turning around when approaching the goal
    const double dist_to_goal = std::sqrt(dist_to_goal_sq);
    if (dist_to_goal > 0.1)  // Only apply when not too close to avoid singularity
    {
      // Desired heading toward goal
      const double desired_yaw = std::atan2(-dy_goal, -dx_goal);

      // Heading error (normalized to [-π, π])
      double heading_error = desired_yaw - pred.yaw;
      heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

      // Penalize heading misalignment (weight increases as we get closer to goal)
      // This encourages smooth forward approach rather than turning around
      const double heading_weight = 0.5 * std::min(1.0, 2.0 / dist_to_goal);
      cost += heading_weight * (heading_error * heading_error);
    }

    // NOTE: Final orientation alignment (matching goal_yaw_) is handled
    // separately via in-place rotation after reaching goal position

    return cost;
  }

  /**
   * @brief Get minimum distance to obstacles from current robot pose
   * @param robot Current robot state
   * @param scan LaserScan data
   * @return Minimum distance to obstacles
   */
  double getMinObstacleDistanceFromCurrentPose(
    const RobotState & robot,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) const
  {
    if (!scan || scan->ranges.empty())
    {
      return std::numeric_limits<double>::infinity();
    }

    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto & range : scan->ranges)
    {
      if (!std::isnan(range) && !std::isinf(range) &&
          range >= min_valid_laser_range_ && range <= scan->range_max)
      {
        min_dist = std::min(min_dist, static_cast<double>(range));
      }
    }
    return min_dist;
  }

  /**
   * @brief Get minimum distance in front of robot (forward cone)
   * @param scan LaserScan data
   * @return Minimum distance in forward direction
   */
  double getMinFrontDistance(const sensor_msgs::msg::LaserScan::SharedPtr & scan) const
  {
    if (!scan || scan->ranges.empty())
    {
      return std::numeric_limits<double>::infinity();
    }

    const size_t num_rays = scan->ranges.size();
    const double angle_min = scan->angle_min;
    const double angle_increment = scan->angle_increment;
    const double cone_angle = M_PI / 4.0;  // 45 degrees

    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < num_rays; ++i)
    {
      const double ray_angle = angle_min + i * angle_increment;
      if (std::abs(ray_angle) > cone_angle)
      {
        continue;
      }

      const float range = scan->ranges[i];
      if (!std::isnan(range) && !std::isinf(range) &&
          range >= min_valid_laser_range_ && range <= scan->range_max)
      {
        min_dist = std::min(min_dist, static_cast<double>(range));
      }
    }
    return min_dist;
  }

  /**
   * @brief Compute minimum distance from predicted robot position to obstacles
   * @param current_robot Current robot state (where scan was taken)
   * @param pred_robot Predicted robot state (where we want to check collision)
   * @param scan LaserScan data (in current robot frame)
   * @return Minimum distance from predicted position to obstacles
   */
  double getMinObstacleDistance(
    const RobotState & current_robot,
    const RobotState & pred_robot,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) const
  {
    if (!scan || scan->ranges.empty())
    {
      return std::numeric_limits<double>::infinity();
    }

    double min_dist = std::numeric_limits<double>::infinity();

    // Sample laser scan rays to find closest obstacle
    const size_t num_rays = scan->ranges.size();
    const double angle_min = scan->angle_min;
    const double angle_increment = scan->angle_increment;

    // Define forward cone angle (90 degrees = ±45 degrees from heading)
    // This ensures we only check obstacles ahead of the robot's moving direction
    const double forward_cone_angle = M_PI / 2.0;  // 90 degrees

    for (size_t i = 0; i < num_rays; ++i)
    {
      const float range = scan->ranges[i];

      // Skip invalid readings and self-detections (robot's own body)
      if (std::isnan(range) || std::isinf(range) ||
          range < min_valid_laser_range_ || range > scan->range_max)
      {
        continue;
      }

      // Calculate the angle of this ray in the current robot's frame
      const double ray_angle = angle_min + i * angle_increment;

      // Step 1: Convert scan point to current robot's local coordinates
      const double local_x = range * std::cos(ray_angle);
      const double local_y = range * std::sin(ray_angle);

      // Step 2: Transform to global coordinates using CURRENT robot pose
      const double global_x = current_robot.x +
                              local_x * std::cos(current_robot.yaw) -
                              local_y * std::sin(current_robot.yaw);
      const double global_y = current_robot.y +
                              local_x * std::sin(current_robot.yaw) +
                              local_y * std::cos(current_robot.yaw);

      // Step 3: Check if obstacle is in the forward cone relative to PREDICTED heading
      // Calculate angle from predicted robot to obstacle
      const double dx = global_x - pred_robot.x;
      const double dy = global_y - pred_robot.y;
      const double angle_to_obstacle = std::atan2(dy, dx);

      // Calculate relative angle (obstacle angle - robot heading)
      double relative_angle = angle_to_obstacle - pred_robot.yaw;

      // Normalize to [-pi, pi]
      while (relative_angle > M_PI) relative_angle -= 2.0 * M_PI;
      while (relative_angle < -M_PI) relative_angle += 2.0 * M_PI;

      // Only consider obstacles in the forward cone (±45 degrees from heading)
      if (std::abs(relative_angle) > forward_cone_angle / 2.0)
      {
        continue;  // Skip obstacles behind or to the side
      }

      // Step 4: Calculate distance from PREDICTED robot position to obstacle
      const double dist = std::hypot(dx, dy);
      min_dist = std::min(min_dist, dist);
    }

    return min_dist;
  }

  /**
   * @brief Check if collision is imminent based on forward laser scan readings
   * @param scan LaserScan data
   * @param linear_vel Current linear velocity command
   * @return true if collision is imminent
   */
  bool isCollisionImminent(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    double linear_vel) const
  {
    if (!scan || scan->ranges.empty() || linear_vel <= 0.0)
    {
      return false;
    }

    // Check a frontal cone (±45 degrees)
    const size_t num_rays = scan->ranges.size();
    const double angle_min = scan->angle_min;
    const double angle_increment = scan->angle_increment;
    const double cone_angle = M_PI / 4.0;  // 45 degrees

    for (size_t i = 0; i < num_rays; ++i)
    {
      const float range = scan->ranges[i];
      const double ray_angle = angle_min + i * angle_increment;

      // Only check rays in the forward cone
      if (std::abs(ray_angle) > cone_angle)
      {
        continue;
      }

      // Skip invalid readings and self-detections (robot's own body)
      if (std::isnan(range) || std::isinf(range) ||
          range < min_valid_laser_range_ || range > scan->range_max)
      {
        continue;
      }

      // If any obstacle is within safety distance, collision is imminent
      if (range < min_obstacle_distance_)
      {
        return true;
      }
    }

    return false;
  }

  void createMpcLog()
  {
    if (!log_mpc_to_csv_)
    {
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_directory_, ec);
    if (ec)
    {
      RCLCPP_WARN(
        get_logger(), "Failed to create log directory '%s': %s",
        log_directory_.c_str(), ec.message().c_str());
      log_mpc_to_csv_ = false;
      return;
    }
    std::filesystem::path file_path(log_directory_);
    file_path /= "mpc_log.csv";
    mpc_log_stream_.open(file_path, std::ios::out | std::ios::app);
    if (!mpc_log_stream_.good())
    {
      RCLCPP_WARN(get_logger(), "Failed to open MPC log file '%s'.", file_path.string().c_str());
      log_mpc_to_csv_ = false;
      return;
    }
    if (mpc_log_stream_.tellp() == 0)
    {
      mpc_log_stream_ << "stamp_sec,px,py,yaw,goal_x,goal_y,v_cmd,w_cmd,v_max,w_goal,w_social,min_dist,num_people\n";
    }
  }

  void createObstacleLog()
  {
    if (!log_mpc_to_csv_)
    {
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_directory_, ec);
    if (ec)
    {
      return;
    }
    std::filesystem::path file_path(log_directory_);
    file_path /= "obstacle_avoidance_log.csv";
    obstacle_log_stream_.open(file_path, std::ios::out | std::ios::app);
    if (!obstacle_log_stream_.good())
    {
      RCLCPP_WARN(get_logger(), "Failed to open obstacle log file '%s'.", file_path.string().c_str());
      return;
    }
    if (obstacle_log_stream_.tellp() == 0)
    {
      obstacle_log_stream_ << "stamp_sec,robot_x,robot_y,robot_yaw,min_obstacle_dist,v_cmd,w_cmd,avoidance_active\n";
    }
    RCLCPP_INFO(get_logger(), "Obstacle avoidance logging enabled: %s", file_path.string().c_str());
  }

  void createPedestrianLog()
  {
    if (!log_mpc_to_csv_)
    {
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_directory_, ec);
    if (ec)
    {
      return;
    }
    std::filesystem::path file_path(log_directory_);
    file_path /= "pedestrian_avoidance_log.csv";
    pedestrian_log_stream_.open(file_path, std::ios::out | std::ios::app);
    if (!pedestrian_log_stream_.good())
    {
      RCLCPP_WARN(get_logger(), "Failed to open pedestrian log file '%s'.", file_path.string().c_str());
      return;
    }
    if (pedestrian_log_stream_.tellp() == 0)
    {
      pedestrian_log_stream_ << "stamp_sec,robot_x,robot_y,robot_yaw,num_pedestrians,min_pedestrian_dist,nearest_ped_x,nearest_ped_y,v_cmd,w_cmd\n";
    }
    RCLCPP_INFO(get_logger(), "Pedestrian avoidance logging enabled: %s", file_path.string().c_str());
  }

  void logMpcStep(
    const RobotState & robot,
    const social_mpc_nav::msg::People2D::SharedPtr & people,
    const SocialContract & contract,
    const geometry_msgs::msg::Twist & cmd)
  {
    if (!log_mpc_to_csv_ || !mpc_log_stream_.good())
    {
      return;
    }

    const size_t num_people = people ? people->people.size() : 0;
    const double min_dist = contract.min_person_distance;
    const double stamp = now().seconds();

    mpc_log_stream_ << std::fixed << std::setprecision(3)
                    << stamp << ","
                    << robot.x << ","
                    << robot.y << ","
                    << robot.yaw << ","
                    << goal_x_ << ","
                    << goal_y_ << ","
                    << cmd.linear.x << ","
                    << cmd.angular.z << ","
                    << contract.v_max << ","
                    << contract.w_goal << ","
                    << contract.w_social << ","
                    << min_dist << ","
                    << num_people
                    << "\n";
    mpc_log_stream_.flush();
  }

  void logObstacleAvoidance(
    const RobotState & robot,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    const geometry_msgs::msg::Twist & cmd)
  {
    if (!log_mpc_to_csv_ || !obstacle_log_stream_.good() || !scan)
    {
      return;
    }

    const double min_obstacle_dist = getMinObstacleDistanceFromCurrentPose(robot, scan);
    const double stamp = now().seconds();

    // Log if obstacle is close enough to trigger avoidance (within 2 meters)
    const bool avoidance_active = min_obstacle_dist < 2.0;

    obstacle_log_stream_ << std::fixed << std::setprecision(3)
                         << stamp << ","
                         << robot.x << ","
                         << robot.y << ","
                         << robot.yaw << ","
                         << min_obstacle_dist << ","
                         << cmd.linear.x << ","
                         << cmd.angular.z << ","
                         << (avoidance_active ? 1 : 0)
                         << "\n";
    obstacle_log_stream_.flush();
  }

  void logPedestrianAvoidance(
    const RobotState & robot,
    const social_mpc_nav::msg::People2D::SharedPtr & people,
    const geometry_msgs::msg::Twist & cmd)
  {
    if (!log_mpc_to_csv_ || !pedestrian_log_stream_.good() || !people || people->people.empty())
    {
      return;
    }

    const double stamp = now().seconds();
    const size_t num_people = people->people.size();

    // Find nearest pedestrian
    double min_person_dist = std::numeric_limits<double>::infinity();
    double nearest_ped_x = 0.0;
    double nearest_ped_y = 0.0;

    for (const auto & person : people->people)
    {
      const double dist = std::hypot(robot.x - person.x, robot.y - person.y);
      if (dist < min_person_dist)
      {
        min_person_dist = dist;
        nearest_ped_x = person.x;
        nearest_ped_y = person.y;
      }
    }

    pedestrian_log_stream_ << std::fixed << std::setprecision(3)
                           << stamp << ","
                           << robot.x << ","
                           << robot.y << ","
                           << robot.yaw << ","
                           << num_people << ","
                           << min_person_dist << ","
                           << nearest_ped_x << ","
                           << nearest_ped_y << ","
                           << cmd.linear.x << ","
                           << cmd.angular.z
                           << "\n";
    pedestrian_log_stream_.flush();
  }

  /**
   * @brief Update current waypoint index based on robot position
   * Must be called with data_mutex_ locked
   */
  void updateCurrentWaypoint(double robot_x, double robot_y)
  {
    if (!global_path_ || global_path_->poses.empty())
    {
      return;
    }

    // Use smaller lookahead to avoid advancing past goal too early
    // This prevents turning behavior when approaching goal
    const double lookahead_dist = 0.3;  // Advance waypoint when within 0.3m

    // Check if current waypoint is reached
    if (current_waypoint_index_ < global_path_->poses.size())
    {
      const auto& current_wp = global_path_->poses[current_waypoint_index_];
      const double dist = std::hypot(
        robot_x - current_wp.pose.position.x,
        robot_y - current_wp.pose.position.y);

      if (dist < lookahead_dist)
      {
        // Don't advance past the last waypoint until we're within goal tolerance
        bool is_last_waypoint = (current_waypoint_index_ == global_path_->poses.size() - 1);
        const double dist_to_goal = std::hypot(robot_x - goal_x_, robot_y - goal_y_);

        if (is_last_waypoint && dist_to_goal > goal_tolerance_)
        {
          // Keep tracking last waypoint until within goal tolerance
          return;
        }

        // Advance to next waypoint
        current_waypoint_index_++;

        if (current_waypoint_index_ < global_path_->poses.size())
        {
          RCLCPP_INFO(
            get_logger(),
            "✅ Waypoint %zu reached! Advancing to waypoint %zu/%zu",
            current_waypoint_index_,
            current_waypoint_index_ + 1,
            global_path_->poses.size());
        }
        else
        {
          RCLCPP_INFO(get_logger(), "✅ All waypoints reached! Approaching final goal.");
        }
      }
    }
  }

  /**
   * @brief Detect if robot is stuck in circular motion (local minima)
   * @param robot Current robot state
   * @param dist_to_goal Current distance to goal
   */
  void detectCircularMotion(const RobotState & robot, double dist_to_goal)
  {
    const auto now_time = now();
    const double current_time = now_time.seconds();

    // Update position history
    position_history_.push_back({robot.x, robot.y, current_time});
    goal_distance_history_.push_back({dist_to_goal, current_time});

    // Keep only recent history (last 10 seconds)
    const double history_window = 10.0;
    while (!position_history_.empty() &&
           current_time - position_history_.front().time > history_window)
    {
      position_history_.pop_front();
    }
    while (!goal_distance_history_.empty() &&
           current_time - goal_distance_history_.front().time > history_window)
    {
      goal_distance_history_.pop_front();
    }

    // Need at least 5 seconds of history
    if (position_history_.size() < 25 || goal_distance_history_.empty())
    {
      return;
    }

    // Check 1: Has the robot made progress towards goal?
    const double initial_dist = goal_distance_history_.front().distance;
    const double current_dist = dist_to_goal;
    const double progress = initial_dist - current_dist;
    const double time_elapsed = current_time - goal_distance_history_.front().time;

    // Only check if far from goal (> 1m) to avoid false alarms near goal
    if (time_elapsed > 10.0 && progress < 0.5 && dist_to_goal > 1.0)
    {
      stuck_counter_++;
      // Require 100 cycles (10 seconds at 10Hz) before warning
      if (stuck_counter_ > 100)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "⚠️  LOCAL MINIMA DETECTED! No progress in %.1fs (progress: %.2fm). "
          "Consider increasing num_rollouts or adjusting cost weights.",
          time_elapsed, progress);

        // Log detailed state for debugging
        RCLCPP_INFO(
          get_logger(),
          "  Position: (%.2f, %.2f) → Goal: (%.2f, %.2f) | Distance: %.2fm",
          robot.x, robot.y, goal_x_, goal_y_, dist_to_goal);
      }
    }
    else
    {
      stuck_counter_ = 0;
    }

    // Check 2: Is the robot moving in a small area? (circling)
    if (position_history_.size() >= 50)  // Require more history (5 seconds at 10Hz)
    {
      double mean_x = 0.0, mean_y = 0.0;
      for (const auto & pos : position_history_)
      {
        mean_x += pos.x;
        mean_y += pos.y;
      }
      mean_x /= position_history_.size();
      mean_y /= position_history_.size();

      double max_radius = 0.0;
      for (const auto & pos : position_history_)
      {
        const double radius = std::hypot(pos.x - mean_x, pos.y - mean_y);
        max_radius = std::max(max_radius, radius);
      }

      // Only warn if circling in very tight radius AND goal is far AND getting worse
      if (max_radius < 0.5 && dist_to_goal > 3.0 && progress < -0.5)
      {
        circle_counter_++;
        // Require 100 cycles (10 seconds at 10Hz) before warning
        if (circle_counter_ > 100)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "⚠️  CIRCULAR MOTION DETECTED! Moving in %.2fm radius, goal is %.2fm away. "
            "MPC stuck in local minimum.",
            max_radius, dist_to_goal);
        }
      }
      else
      {
        circle_counter_ = 0;
      }
    }
  }

  SocialContractHelper contract_helper_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  double goal_tolerance_{0.3};  // 30cm position tolerance for goal reaching
  bool goal_reached_{false};
  bool goal_received_{false};  // Flag to prevent navigation until goal is set via RViz2
  double control_rate_hz_{10.0};
  double dt_{0.2};
  int horizon_steps_{15};
  double default_v_max_{0.6};
  double omega_max_{0.9};
  double smooth_weight_{0.1};
  double obstacle_weight_{3.0};
  double min_obstacle_distance_{0.3};
  double min_valid_laser_range_{0.15};
  bool safety_check_enabled_{true};
  bool enable_debug_logging_{false};
  int num_rollouts_{60};
  std::string log_directory_;
  bool log_mpc_to_csv_{true};
  std::string crowd_topic_;

  // Local minima detection
  struct PositionRecord {
    double x;
    double y;
    double time;
  };
  struct DistanceRecord {
    double distance;
    double time;
  };
  std::deque<PositionRecord> position_history_;
  std::deque<DistanceRecord> goal_distance_history_;
  int stuck_counter_{0};
  int circle_counter_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<person_tracker::msg::PersonInfoArray>::SharedPtr crowd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF2 for coordinate frame transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_;
  std::string odom_frame_;

  std::mutex data_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  social_mpc_nav::msg::People2D::SharedPtr latest_people_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Path::SharedPtr global_path_;
  bool path_received_{false};
  size_t current_waypoint_index_{0};

  std::pair<double, double> last_cmd_{0.0, 0.0};

  std::mt19937 rng_;
  std::ofstream mpc_log_stream_;
  std::ofstream obstacle_log_stream_;
  std::ofstream pedestrian_log_stream_;
};

// Global pointer for signal handler
std::shared_ptr<MPCControllerNode> g_node = nullptr;

void signalHandler(int signum)
{
  (void)signum;  // Unused parameter
  if (g_node)
  {
    RCLCPP_INFO(g_node->get_logger(), "Ctrl+C detected - stopping robot...");
    g_node->stopRobot();
    // Give time for the stop command to be published
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create node instance
  g_node = std::make_shared<MPCControllerNode>();

  // Register signal handler for Ctrl+C
  std::signal(SIGINT, signalHandler);

  RCLCPP_INFO(g_node->get_logger(), "Press Ctrl+C to stop the robot and exit");

  // Spin the node
  rclcpp::spin(g_node);

  // Clean shutdown
  g_node.reset();
  rclcpp::shutdown();
  return 0;
}

