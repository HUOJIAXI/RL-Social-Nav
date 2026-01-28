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

#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/social_contract.hpp"
#include "social_mpc_nav/msg/vlm_parameters.hpp"
#include "social_mpc_nav/mpc_vlm_helpers.hpp"

using social_mpc_nav::SocialContract;
using social_mpc_nav::SocialContractHelper;
using namespace social_mpc_nav::vlm_helpers;
using social_mpc_nav::vlm_helpers::RobotState;

namespace
{
struct DWAWindow
{
  double min_v;
  double max_v;
  double min_w;
  double max_w;
};
}  // namespace

/**
 * @brief DWA Local Planner with VLM Integration for comparison with VLM-MPC
 *
 * The controller implements a Dynamic Window Approach (DWA):
 *  - Sample candidate velocities (v, omega) within dynamic window.
 *  - Roll out the unicycle model and evaluate the multi-objective cost.
 *  - Apply the best velocity command.
 *  - Matches VLM-MPC's cost function and logging for fair comparison.
 */
class DWAControllerNode : public rclcpp::Node
{
public:
  DWAControllerNode()
  : Node("dwa_controller_node"),
    contract_helper_(get_logger(),
      declare_parameter<std::string>(
        "log_directory",
        []() {
          const char * home = std::getenv("HOME");
          return home ? std::string(home) + "/ros2_logs/dwa_local_planner" : "/tmp/dwa_local_planner";
        }()),
      declare_parameter<bool>("log_social_contract_to_csv", true))
  {
    // Goal and control parameters
    goal_x_ = declare_parameter<double>("goal_x", 2.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.3);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);
    if (control_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "control_rate_hz must be > 0. Resetting to 10 Hz.");
      control_rate_hz_ = 10.0;
    }

    // DWA parameters
    dt_ = declare_parameter<double>("dt", 0.2);
    predict_time_ = declare_parameter<double>("predict_time", 3.0);  // 3 seconds ahead
    default_v_max_ = declare_parameter<double>("default_v_max", 0.6);
    omega_max_ = declare_parameter<double>("omega_max", 0.9);
    v_resolution_ = declare_parameter<double>("v_resolution", 0.05);  // 0.05 m/s steps
    w_resolution_ = declare_parameter<double>("w_resolution", 0.1);   // 0.1 rad/s steps

    // Acceleration limits for dynamic window
    acc_lim_v_ = declare_parameter<double>("acc_lim_v", 0.5);      // m/s^2
    acc_lim_w_ = declare_parameter<double>("acc_lim_w", 1.0);      // rad/s^2

    // Cost weights
    smooth_weight_ = declare_parameter<double>("w_smooth", 0.5);
    obstacle_weight_ = declare_parameter<double>("w_obstacle", 3.0);
    heading_weight_ = declare_parameter<double>("w_heading", 2.0);

    // VLM cost weights (matching VLM-MPC)
    w_vlm_directional_ = declare_parameter<double>("w_vlm_directional", 1.0);
    w_vlm_action_ = declare_parameter<double>("w_vlm_action", 2.0);
    w_vlm_scene_ = declare_parameter<double>("w_vlm_scene", 1.5);
    w_vlm_personal_ = declare_parameter<double>("w_vlm_personal", 5.0);

    // Obstacle parameters
    min_obstacle_distance_ = declare_parameter<double>("min_obstacle_distance", 0.3);
    hard_min_obstacle_distance_ = declare_parameter<double>("hard_min_obstacle_distance", 0.05);
    min_valid_laser_range_ = declare_parameter<double>("min_valid_laser_range", 0.15);
    safety_check_enabled_ = declare_parameter<bool>("safety_check_enabled", true);

    // Logging
    log_directory_ = get_parameter("log_directory").as_string();
    log_to_csv_ = declare_parameter<bool>("log_to_csv", true);
    enable_debug_logging_ = declare_parameter<bool>("enable_debug_logging", false);
    crowd_topic_ = declare_parameter<std::string>("crowd_topic", "/person_tracker/person_info");

    // Initialize previous velocities
    prev_v_ = 0.0;
    prev_w_ = 0.0;

    // Subscribers
    std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&DWAControllerNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to odometry: %s", odom_topic.c_str());

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar/scan", 10,
      std::bind(&DWAControllerNode::laserCallback, this, std::placeholders::_1));

    people_sub_ = create_subscription<social_mpc_nav::msg::People2D>(
      "/people_2d", 10,
      std::bind(&DWAControllerNode::peopleCallback, this, std::placeholders::_1));

    vlm_params_sub_ = create_subscription<social_mpc_nav::msg::VLMParameters>(
      "/vlm/mpc_parameters", 10,
      std::bind(&DWAControllerNode::vlmParametersCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&DWAControllerNode::goalCallback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/task_generator_node/tiago_base/cmd_vel", 10);

    // Control loop timer
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DWAControllerNode::controlLoop, this));

    // Setup logging
    if (log_to_csv_)
    {
      setupLogging();
    }

    RCLCPP_INFO(get_logger(), "DWA Controller Node initialized");
    RCLCPP_INFO(get_logger(), "  Goal: (%.2f, %.2f)", goal_x_, goal_y_);
    RCLCPP_INFO(get_logger(), "  Max velocity: %.2f m/s, %.2f rad/s", default_v_max_, omega_max_);
    RCLCPP_INFO(get_logger(), "  DWA resolution: v=%.3f, w=%.3f", v_resolution_, w_resolution_);
    RCLCPP_INFO(get_logger(), "  Prediction time: %.1f s", predict_time_);
  }

  ~DWAControllerNode()
  {
    // Stop the robot
    auto twist = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist);

    // Close log files
    if (mpc_log_.is_open()) mpc_log_.close();
    if (obstacle_log_.is_open()) obstacle_log_.close();
    if (pedestrian_log_.is_open()) pedestrian_log_.close();
  }

private:
  void setupLogging()
  {
    namespace fs = std::filesystem;
    fs::path log_dir(log_directory_);

    try {
      if (!fs::exists(log_dir)) {
        fs::create_directories(log_dir);
      }

      auto timestamp = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(timestamp);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
      std::string time_str = ss.str();

      // DWA log (equivalent to mpc_log)
      std::string mpc_log_path = log_directory_ + "/dwa_log_" + time_str + ".csv";
      mpc_log_.open(mpc_log_path);
      if (mpc_log_.is_open()) {
        mpc_log_ << "stamp_sec,px,py,yaw,goal_x,goal_y,v_cmd,w_cmd,v_max,w_goal,w_social,min_dist,num_people\n";
        RCLCPP_INFO(get_logger(), "DWA log: %s", mpc_log_path.c_str());
      }

      // Obstacle avoidance log
      std::string obstacle_log_path = log_directory_ + "/obstacle_avoidance_log_" + time_str + ".csv";
      obstacle_log_.open(obstacle_log_path);
      if (obstacle_log_.is_open()) {
        obstacle_log_ << "stamp_sec,robot_x,robot_y,robot_yaw,min_obstacle_dist,v_cmd,w_cmd,avoidance_active\n";
        RCLCPP_INFO(get_logger(), "Obstacle log: %s", obstacle_log_path.c_str());
      }

      // Pedestrian avoidance log
      std::string pedestrian_log_path = log_directory_ + "/pedestrian_avoidance_log_" + time_str + ".csv";
      pedestrian_log_.open(pedestrian_log_path);
      if (pedestrian_log_.is_open()) {
        pedestrian_log_ << "stamp_sec,robot_x,robot_y,robot_yaw,num_pedestrians,min_pedestrian_dist,nearest_ped_x,nearest_ped_y,v_cmd,w_cmd\n";
        RCLCPP_INFO(get_logger(), "Pedestrian log: %s", pedestrian_log_path.c_str());
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to setup logging: %s", e.what());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_odom_ = msg;
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_laser_ = msg;
  }

  void peopleCallback(const social_mpc_nav::msg::People2D::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_people_ = msg;
  }

  void vlmParametersCallback(const social_mpc_nav::msg::VLMParameters::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_vlm_params_ = msg;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_reached_ = false;  // Reset goal reached flag
    RCLCPP_INFO(get_logger(), "New goal received: (%.2f, %.2f)", goal_x_, goal_y_);
  }

  void controlLoop()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!last_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No odometry received");
      return;
    }

    // Extract robot state
    RobotState robot;
    robot.x = last_odom_->pose.pose.position.x;
    robot.y = last_odom_->pose.pose.position.y;
    robot.yaw = tf2::getYaw(last_odom_->pose.pose.orientation);

    // Check if goal reached
    double dx = goal_x_ - robot.x;
    double dy = goal_y_ - robot.y;
    double dist_to_goal = std::hypot(dx, dy);

    if (dist_to_goal < goal_tolerance_) {
      if (!goal_reached_) {
        RCLCPP_INFO(get_logger(), "Goal reached! Distance: %.2f m", dist_to_goal);
        goal_reached_ = true;
      }
      auto twist = geometry_msgs::msg::Twist();
      cmd_vel_pub_->publish(twist);
      return;
    }

    // Compute social contract (same as VLM-MPC)
    SocialContract contract = contract_helper_.compute(
      robot.x, robot.y, robot.yaw,
      default_v_max_,
      last_people_);

    // Apply VLM modulation if available
    if (last_vlm_params_) {
      contract.v_max *= last_vlm_params_->speed_scale;

      // Emergency stop if VLM says wait
      if (last_vlm_params_->need_to_wait) {
        if (enable_debug_logging_) {
          RCLCPP_INFO(get_logger(), "VLM emergency stop: need_to_wait=true");
        }
        auto twist = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(twist);
        logStep(robot, contract, 0.0, 0.0);
        return;
      }

      // Adaptive social weight based on personal distance
      if (last_people_ && !last_people_->people.empty()) {
        double min_person_dist = computeMinPersonDistance(robot, last_people_);
        double personal_dist = last_vlm_params_->min_personal_distance;

        if (min_person_dist < personal_dist * 2.0) {
          double distance_ratio = min_person_dist / (personal_dist * 2.0);
          contract.w_social *= (2.0 - distance_ratio);
        }
      }
    }

    // Run DWA optimization
    auto [best_v, best_w] = solveDWA(robot, contract);

    // Safety check for imminent collisions
    if (safety_check_enabled_ && last_laser_) {
      if (checkImmediateCollision()) {
        if (enable_debug_logging_) {
          RCLCPP_WARN(get_logger(), "Safety check: immediate collision detected, stopping");
        }
        best_v = 0.0;
        best_w = 0.0;
      }
    }

    // Publish velocity command
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = best_v;
    twist.angular.z = best_w;
    cmd_vel_pub_->publish(twist);

    // Update previous velocities for next iteration
    prev_v_ = best_v;
    prev_w_ = best_w;

    // Logging
    logStep(robot, contract, best_v, best_w);

    if (enable_debug_logging_) {
      RCLCPP_INFO(get_logger(), "DWA: v=%.2f, w=%.2f | Goal dist=%.2f",
                  best_v, best_w, dist_to_goal);
    }
  }

  std::pair<double, double> solveDWA(const RobotState& robot, const SocialContract& contract)
  {
    // Compute dynamic window based on current velocity and acceleration limits
    DWAWindow window = computeDynamicWindow(prev_v_, prev_w_, contract.v_max);

    double best_v = 0.0;
    double best_w = 0.0;
    double best_cost = std::numeric_limits<double>::max();

    // Sample velocities within dynamic window
    for (double v = window.min_v; v <= window.max_v; v += v_resolution_) {
      for (double w = window.min_w; w <= window.max_w; w += w_resolution_) {
        // Evaluate trajectory
        double cost = evaluateTrajectory(v, w, robot, contract);

        if (cost < best_cost) {
          best_cost = cost;
          best_v = v;
          best_w = w;
        }
      }
    }

    return {best_v, best_w};
  }

  DWAWindow computeDynamicWindow(double curr_v, double curr_w, double v_max_contract)
  {
    DWAWindow window;

    // Velocity limits from robot kinematics
    double v_max = std::min(default_v_max_, v_max_contract);
    double w_max = omega_max_;

    // Dynamic window from acceleration limits
    double v_min_acc = curr_v - acc_lim_v_ * dt_;
    double v_max_acc = curr_v + acc_lim_v_ * dt_;
    double w_min_acc = curr_w - acc_lim_w_ * dt_;
    double w_max_acc = curr_w + acc_lim_w_ * dt_;

    // Intersection of velocity and acceleration constraints
    window.min_v = std::max(0.0, v_min_acc);
    window.max_v = std::min(v_max, v_max_acc);
    window.min_w = std::max(-w_max, w_min_acc);
    window.max_w = std::min(w_max, w_max_acc);

    return window;
  }

  double evaluateTrajectory(double v, double w, const RobotState& robot, const SocialContract& contract)
  {
    double total_cost = 0.0;
    const double eps = 1e-6;
    const double infinity = 1e9;

    // Predict trajectory
    RobotState pred = robot;
    int num_steps = static_cast<int>(predict_time_ / dt_);

    for (int i = 0; i < num_steps; ++i) {
      // Simulate unicycle dynamics
      pred.x += v * std::cos(pred.yaw) * dt_;
      pred.y += v * std::sin(pred.yaw) * dt_;
      pred.yaw += w * dt_;

      // Normalize yaw
      while (pred.yaw > M_PI) pred.yaw -= 2.0 * M_PI;
      while (pred.yaw < -M_PI) pred.yaw += 2.0 * M_PI;

      // === Social cost (same as VLM-MPC) ===
      if (last_people_ && !last_people_->people.empty()) {
        for (const auto& person : last_people_->people) {
          double dx_person = person.x - pred.x;
          double dy_person = person.y - pred.y;
          double dist = std::hypot(dx_person, dy_person);
          double capped_dist = std::min(dist, 3.0);

          // Inverse distance penalty
          total_cost += contract.w_social * (1.0 / (capped_dist + eps));

          // VLM personal distance violation cost
          if (last_vlm_params_) {
            total_cost += computeVLMPersonalDistanceCost(dist, *last_vlm_params_, w_vlm_personal_);
          }
        }
      }

      // === Obstacle cost (same as VLM-MPC) ===
      if (last_laser_) {
        double min_obstacle_dist = getMinObstacleDistance(pred);

        // Hard constraint: reject trajectory if too close
        if (min_obstacle_dist < hard_min_obstacle_distance_) {
          return infinity;
        }

        // Soft penalty for nearby obstacles
        if (min_obstacle_dist < 2.0) {
          double capped_dist = std::min(min_obstacle_dist, 2.0);
          total_cost += obstacle_weight_ * (1.0 / (capped_dist + eps));
        }
      }
    }

    // === Smoothness cost (same as VLM-MPC) ===
    double dv = v - prev_v_;
    double dw = w - prev_w_;
    total_cost += smooth_weight_ * (dv * dv + dw * dw);

    // === VLM costs (same as VLM-MPC) ===
    if (last_vlm_params_) {
      RobotState mid_pred = robot;
      // Evaluate VLM costs at middle of trajectory
      for (int i = 0; i < num_steps / 2; ++i) {
        mid_pred.x += v * std::cos(mid_pred.yaw) * dt_;
        mid_pred.y += v * std::sin(mid_pred.yaw) * dt_;
        mid_pred.yaw += w * dt_;
      }

      total_cost += computeVLMDirectionalCost(mid_pred, robot.x, robot.y, goal_x_, goal_y_,
                                               *last_vlm_params_, w_vlm_directional_);
      total_cost += computeVLMActionCost(v, mid_pred, last_people_, *last_vlm_params_, w_vlm_action_);
      total_cost += computeVLMSceneCost(v, mid_pred, *last_vlm_params_, w_vlm_scene_);
    }

    // === Terminal goal cost (same as VLM-MPC) ===
    RobotState final_pred = robot;
    for (int i = 0; i < num_steps; ++i) {
      final_pred.x += v * std::cos(final_pred.yaw) * dt_;
      final_pred.y += v * std::sin(final_pred.yaw) * dt_;
      final_pred.yaw += w * dt_;
    }

    double dx_goal = goal_x_ - final_pred.x;
    double dy_goal = goal_y_ - final_pred.y;
    double dist_to_goal = std::hypot(dx_goal, dy_goal);
    total_cost += contract.w_goal * (dx_goal * dx_goal + dy_goal * dy_goal);

    // Heading alignment to goal
    double desired_yaw = std::atan2(dy_goal, dx_goal);
    double yaw_error = desired_yaw - final_pred.yaw;
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    total_cost += heading_weight_ * (yaw_error * yaw_error);

    return total_cost;
  }

  double getMinObstacleDistance(const RobotState& state)
  {
    if (!last_laser_) {
      return std::numeric_limits<double>::max();
    }

    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < last_laser_->ranges.size(); ++i) {
      float range = last_laser_->ranges[i];

      if (std::isnan(range) || std::isinf(range)) continue;
      if (range < min_valid_laser_range_ || range > last_laser_->range_max) continue;

      double angle = last_laser_->angle_min + i * last_laser_->angle_increment;
      double obstacle_x = state.x + range * std::cos(state.yaw + angle);
      double obstacle_y = state.y + range * std::sin(state.yaw + angle);

      double dx = obstacle_x - state.x;
      double dy = obstacle_y - state.y;
      double dist = std::hypot(dx, dy);

      min_dist = std::min(min_dist, dist);
    }

    return min_dist;
  }

  double computeMinPersonDistance(const RobotState& robot,
                                   const social_mpc_nav::msg::People2D::SharedPtr& people)
  {
    double min_dist = std::numeric_limits<double>::max();

    if (!people || people->people.empty()) {
      return min_dist;
    }

    for (const auto& person : people->people) {
      double dx = person.x - robot.x;
      double dy = person.y - robot.y;
      double dist = std::hypot(dx, dy);
      min_dist = std::min(min_dist, dist);
    }

    return min_dist;
  }

  bool checkImmediateCollision()
  {
    if (!last_laser_) return false;

    for (size_t i = 0; i < last_laser_->ranges.size(); ++i) {
      float range = last_laser_->ranges[i];

      if (std::isnan(range) || std::isinf(range)) continue;
      if (range < min_valid_laser_range_) continue;

      // Check for very close obstacles in front
      double angle = last_laser_->angle_min + i * last_laser_->angle_increment;
      if (std::abs(angle) < M_PI / 4.0 && range < hard_min_obstacle_distance_ * 2.0) {
        return true;
      }
    }

    return false;
  }

  void logStep(const RobotState& robot_state, const SocialContract& contract, double v_cmd, double w_cmd)
  {
    if (!log_to_csv_) return;

    double timestamp = now().seconds();

    // Compute min distances
    double min_person_dist = computeMinPersonDistance(robot_state, last_people_);
    int num_people = last_people_ ? last_people_->people.size() : 0;

    // DWA log (equivalent to mpc_log)
    if (mpc_log_.is_open()) {
      mpc_log_ << std::fixed << std::setprecision(3)
               << timestamp << ","
               << robot_state.x << "," << robot_state.y << "," << robot_state.yaw << ","
               << goal_x_ << "," << goal_y_ << ","
               << v_cmd << "," << w_cmd << ","
               << contract.v_max << "," << contract.w_goal << "," << contract.w_social << ","
               << min_person_dist << "," << num_people << "\n";
      mpc_log_.flush();
    }

    // Obstacle avoidance log
    if (obstacle_log_.is_open()) {
      double min_obstacle_dist = getMinObstacleDistance(robot_state);
      int avoidance_active = (min_obstacle_dist < 2.0) ? 1 : 0;

      obstacle_log_ << std::fixed << std::setprecision(3)
                    << timestamp << ","
                    << robot_state.x << "," << robot_state.y << "," << robot_state.yaw << ","
                    << min_obstacle_dist << ","
                    << v_cmd << "," << w_cmd << ","
                    << avoidance_active << "\n";
      obstacle_log_.flush();
    }

    // Pedestrian avoidance log
    if (pedestrian_log_.is_open()) {
      double nearest_ped_x = 0.0;
      double nearest_ped_y = 0.0;

      if (last_people_ && !last_people_->people.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& person : last_people_->people) {
          double dx = person.x - robot_state.x;
          double dy = person.y - robot_state.y;
          double dist = std::hypot(dx, dy);
          if (dist < min_dist) {
            min_dist = dist;
            nearest_ped_x = person.x;
            nearest_ped_y = person.y;
          }
        }
      }

      pedestrian_log_ << std::fixed << std::setprecision(3)
                      << timestamp << ","
                      << robot_state.x << "," << robot_state.y << "," << robot_state.yaw << ","
                      << num_people << ","
                      << min_person_dist << ","
                      << nearest_ped_x << "," << nearest_ped_y << ","
                      << v_cmd << "," << w_cmd << "\n";
      pedestrian_log_.flush();
    }
  }

  // Parameters
  double goal_x_, goal_y_;
  double goal_tolerance_;
  double control_rate_hz_;
  double dt_;
  double predict_time_;
  double default_v_max_;
  double omega_max_;
  double v_resolution_;
  double w_resolution_;
  double acc_lim_v_;
  double acc_lim_w_;
  double smooth_weight_;
  double obstacle_weight_;
  double heading_weight_;
  double w_vlm_directional_;
  double w_vlm_action_;
  double w_vlm_scene_;
  double w_vlm_personal_;
  double min_obstacle_distance_;
  double hard_min_obstacle_distance_;
  double min_valid_laser_range_;
  bool safety_check_enabled_;
  bool log_to_csv_;
  bool enable_debug_logging_;
  std::string log_directory_;
  std::string crowd_topic_;

  // State
  double prev_v_;
  double prev_w_;
  bool goal_reached_{false};

  // ROS communication
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<social_mpc_nav::msg::People2D>::SharedPtr people_sub_;
  rclcpp::Subscription<social_mpc_nav::msg::VLMParameters>::SharedPtr vlm_params_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Cached data
  std::mutex state_mutex_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_;
  social_mpc_nav::msg::People2D::SharedPtr last_people_;
  social_mpc_nav::msg::VLMParameters::SharedPtr last_vlm_params_;

  // Social contract helper
  SocialContractHelper contract_helper_;

  // Logging
  std::ofstream mpc_log_;
  std::ofstream obstacle_log_;
  std::ofstream pedestrian_log_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DWAControllerNode>();

  RCLCPP_INFO(node->get_logger(), "DWA Controller Node spinning...");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
