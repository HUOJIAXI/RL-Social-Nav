#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "person_tracker/msg/person_info_array.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/msg/person2_d.hpp"

namespace
{
struct RobotState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct DWAWindow
{
  double min_v;
  double max_v;
  double min_w;
  double max_w;
};
}  // namespace

/**
 * @brief Pure DWA Local Planner using ground truth localization
 *
 * Uses TF to get robot position from map frame (ground truth from gt_localization_node)
 * Classic DWA cost function: obstacle avoidance + goal reaching + heading + smoothness + social distance
 */
class DWAPureController : public rclcpp::Node
{
public:
  DWAPureController()
  : Node("dwa_controller_node")
  {
    // Goal and control parameters
    goal_x_ = declare_parameter<double>("goal_x", 2.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.3);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);

    // DWA parameters
    dt_ = declare_parameter<double>("dt", 0.2);
    predict_time_ = declare_parameter<double>("predict_time", 3.0);
    v_max_ = declare_parameter<double>("default_v_max", 0.6);
    omega_max_ = declare_parameter<double>("omega_max", 0.9);
    v_resolution_ = declare_parameter<double>("v_resolution", 0.05);
    w_resolution_ = declare_parameter<double>("w_resolution", 0.1);
    acc_lim_v_ = declare_parameter<double>("acc_lim_v", 0.5);
    acc_lim_w_ = declare_parameter<double>("acc_lim_w", 1.0);

    // Cost weights (pure DWA)
    w_goal_ = declare_parameter<double>("w_goal", 1.0);
    w_heading_ = declare_parameter<double>("w_heading", 0.5);
    w_obstacle_ = declare_parameter<double>("w_obstacle", 2.0);
    w_social_ = declare_parameter<double>("w_social", 1.5);
    w_smooth_ = declare_parameter<double>("w_smooth", 0.3);

    // Safety parameters
    min_obstacle_distance_ = declare_parameter<double>("min_obstacle_distance", 0.3);
    hard_min_obstacle_distance_ = declare_parameter<double>("hard_min_obstacle_distance", 0.1);
    min_valid_laser_range_ = declare_parameter<double>("min_valid_laser_range", 0.15);

    // TF parameters
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "tiago_base/base_footprint");

    // Logging
    log_directory_ = declare_parameter<std::string>("log_directory",
      std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") + "/ros2_logs/dwa_local_planner");
    log_to_csv_ = declare_parameter<bool>("log_to_csv", true);
    enable_debug_logging_ = declare_parameter<bool>("enable_debug_logging", false);

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    std::string scan_topic = declare_parameter<std::string>("scan_topic", "/task_generator_node/tiago_base/lidar");
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&DWAPureController::laserCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to laser scan: %s", scan_topic.c_str());

    std::string people_topic = declare_parameter<std::string>("people_topic", "/person_tracker/person_info");
    person_info_sub_ = create_subscription<person_tracker::msg::PersonInfoArray>(
      people_topic, rclcpp::QoS(10),
      std::bind(&DWAPureController::personInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to people: %s", people_topic.c_str());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&DWAPureController::goalCallback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_path", 10,
      std::bind(&DWAPureController::pathCallback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/task_generator_node/tiago_base/cmd_vel", 10);

    // Control loop timer
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DWAPureController::controlLoop, this));

    // Setup logging
    if (log_to_csv_) {
      setupLogging();
    }

    RCLCPP_INFO(get_logger(), "Pure DWA Controller initialized");
    RCLCPP_INFO(get_logger(), "  Using TF: %s -> %s", map_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  Goal: (%.2f, %.2f)", goal_x_, goal_y_);
    RCLCPP_INFO(get_logger(), "  Max velocity: %.2f m/s, %.2f rad/s", v_max_, omega_max_);
  }

  ~DWAPureController()
  {
    auto twist = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist);
    if (dwa_log_.is_open()) dwa_log_.close();
  }

private:
  void setupLogging()
  {
    namespace fs = std::filesystem;
    try {
      // Expand ~ in path
      std::string expanded_log_dir = log_directory_;
      if (!expanded_log_dir.empty() && expanded_log_dir[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) {
          expanded_log_dir = std::string(home) + expanded_log_dir.substr(1);
        }
      }

      fs::path log_dir(expanded_log_dir);
      if (!fs::exists(log_dir)) {
        fs::create_directories(log_dir);
      }

      log_directory_ = expanded_log_dir;  // Update with expanded path

      auto timestamp = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(timestamp);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

      std::string log_path = log_directory_ + "/dwa_log_" + ss.str() + ".csv";
      dwa_log_.open(log_path);
      if (dwa_log_.is_open()) {
        dwa_log_ << "timestamp,x,y,yaw,v_cmd,w_cmd,goal_dist,min_obs_dist,min_person_dist\n";
        RCLCPP_INFO(get_logger(), "DWA log: %s", log_path.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to setup logging: %s", e.what());
    }
  }

  bool getRobotPose(RobotState& robot)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);

      robot.x = transform.transform.translation.x;
      robot.y = transform.transform.translation.y;
      robot.yaw = tf2::getYaw(transform.transform.rotation);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Could not get robot pose: %s", ex.what());
      return false;
    }
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_laser_ = msg;
  }

  void personInfoCallback(const person_tracker::msg::PersonInfoArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Convert PersonInfoArray to People2D for internal use
    auto people_msg = std::make_shared<social_mpc_nav::msg::People2D>();
    people_msg->stamp = msg->header.stamp;

    for (const auto& person : msg->persons) {
      social_mpc_nav::msg::Person2D person_2d;
      person_2d.name = std::to_string(person.person_id);
      person_2d.x = static_cast<float>(person.position.x);
      person_2d.y = static_cast<float>(person.position.y);
      person_2d.vx = static_cast<float>(person.velocity.x);
      person_2d.vy = static_cast<float>(person.velocity.y);
      people_msg->people.push_back(person_2d);
    }

    last_people_ = people_msg;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_reached_ = false;
    current_waypoint_idx_ = 0;  // Reset waypoint tracking
    RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path");
      return;
    }

    global_path_ = msg;
    current_waypoint_idx_ = 0;
    RCLCPP_INFO(get_logger(), "Received global path with %zu waypoints", msg->poses.size());
  }

  void updateCurrentWaypoint(const RobotState& robot)
  {
    if (!global_path_ || global_path_->poses.empty()) {
      return;
    }

    // Find closest waypoint ahead
    const double lookahead_dist = 1.0;  // 1 meter lookahead

    while (current_waypoint_idx_ < global_path_->poses.size()) {
      const auto& wp = global_path_->poses[current_waypoint_idx_];
      double dx = wp.pose.position.x - robot.x;
      double dy = wp.pose.position.y - robot.y;
      double dist = std::hypot(dx, dy);

      if (dist > lookahead_dist) {
        // Update current goal to this waypoint
        goal_x_ = wp.pose.position.x;
        goal_y_ = wp.pose.position.y;
        return;
      }

      // Move to next waypoint if too close
      current_waypoint_idx_++;
    }

    // Reached end of path, use final goal
    if (current_waypoint_idx_ >= global_path_->poses.size()) {
      const auto& final_wp = global_path_->poses.back();
      goal_x_ = final_wp.pose.position.x;
      goal_y_ = final_wp.pose.position.y;
    }
  }

  void controlLoop()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Get robot pose from TF (ground truth)
    RobotState robot;
    if (!getRobotPose(robot)) {
      return;
    }

    // Update current waypoint from global path
    updateCurrentWaypoint(robot);

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

    // Run DWA (also returns min distances along best trajectory)
    double min_obstacle_dist_trajectory = std::numeric_limits<double>::infinity();
    double min_person_dist_trajectory = std::numeric_limits<double>::infinity();
    auto [best_v, best_w] = solveDWA(robot, min_obstacle_dist_trajectory, min_person_dist_trajectory);

    // Publish velocity
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = best_v;
    twist.angular.z = best_w;
    cmd_vel_pub_->publish(twist);

    // Update previous velocities
    prev_v_ = best_v;
    prev_w_ = best_w;

    // Logging - use min distances from best trajectory (like MPC casadi)
    if (log_to_csv_ && dwa_log_.is_open()) {
      dwa_log_ << std::fixed << std::setprecision(3)
               << now().seconds() << ","
               << robot.x << "," << robot.y << "," << robot.yaw << ","
               << best_v << "," << best_w << ","
               << dist_to_goal << ","
               << min_obstacle_dist_trajectory << "," << min_person_dist_trajectory << "\n";
      dwa_log_.flush();
    }

    if (enable_debug_logging_) {
      RCLCPP_INFO(get_logger(), "DWA: v=%.2f, w=%.2f | Goal dist=%.2f",
                  best_v, best_w, dist_to_goal);
    }
  }

  std::pair<double, double> solveDWA(const RobotState& robot,
                                      double& min_obs_dist_out,
                                      double& min_person_dist_out)
  {
    DWAWindow window = computeDynamicWindow();

    double best_v = 0.0;
    double best_w = 0.0;
    double best_cost = std::numeric_limits<double>::max();
    double best_min_obs_dist = std::numeric_limits<double>::infinity();
    double best_min_person_dist = std::numeric_limits<double>::infinity();

    // Sample velocities
    int valid_trajectories = 0;
    for (double v = window.min_v; v <= window.max_v; v += v_resolution_) {
      for (double w = window.min_w; w <= window.max_w; w += w_resolution_) {
        double traj_min_obs_dist = std::numeric_limits<double>::infinity();
        double traj_min_person_dist = std::numeric_limits<double>::infinity();
        double cost = evaluateTrajectory(v, w, robot, traj_min_obs_dist, traj_min_person_dist);

        if (cost < 1e8) {  // Valid trajectory (not infinity)
          valid_trajectories++;
        }

        if (cost < best_cost) {
          best_cost = cost;
          best_v = v;
          best_w = w;
          best_min_obs_dist = traj_min_obs_dist;
          best_min_person_dist = traj_min_person_dist;
        }
      }
    }

    // Recovery behavior: if no valid trajectories found, try rotation in place
    if (valid_trajectories == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "⚠️  DWA: No valid trajectories found! Trying rotation recovery...");

      // Find best rotation direction by checking goal direction
      double goal_angle = std::atan2(goal_y_ - robot.y, goal_x_ - robot.x);
      double angle_diff = goal_angle - robot.yaw;
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

      // Rotate towards goal at moderate speed
      best_v = 0.0;
      best_w = (angle_diff > 0) ? 0.5 : -0.5;  // Moderate rotation speed

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Recovery: Rotating in place (w=%.2f) to align with goal", best_w);
    }

    min_obs_dist_out = best_min_obs_dist;
    min_person_dist_out = best_min_person_dist;
    return {best_v, best_w};
  }

  DWAWindow computeDynamicWindow()
  {
    DWAWindow window;

    // Velocity limits
    window.min_v = 0.0;
    window.max_v = v_max_;
    window.min_w = -omega_max_;
    window.max_w = omega_max_;

    // Dynamic window from acceleration limits
    double v_min_acc = prev_v_ - acc_lim_v_ * dt_;
    double v_max_acc = prev_v_ + acc_lim_v_ * dt_;
    double w_min_acc = prev_w_ - acc_lim_w_ * dt_;
    double w_max_acc = prev_w_ + acc_lim_w_ * dt_;

    window.min_v = std::max(window.min_v, v_min_acc);
    window.max_v = std::min(window.max_v, v_max_acc);
    window.min_w = std::max(window.min_w, w_min_acc);
    window.max_w = std::min(window.max_w, w_max_acc);

    return window;
  }

  double evaluateTrajectory(double v, double w, const RobotState& robot,
                            double& min_obs_dist_out,
                            double& min_person_dist_out)
  {
    const double infinity = 1e9;
    const double eps = 1e-6;
    double total_cost = 0.0;

    // Track minimum distances along trajectory (like MPC casadi)
    min_obs_dist_out = std::numeric_limits<double>::infinity();
    min_person_dist_out = std::numeric_limits<double>::infinity();

    // Simulate trajectory
    RobotState pred = robot;
    int num_steps = static_cast<int>(predict_time_ / dt_);

    for (int i = 0; i < num_steps; ++i) {
      // Unicycle dynamics
      pred.x += v * std::cos(pred.yaw) * dt_;
      pred.y += v * std::sin(pred.yaw) * dt_;
      pred.yaw += w * dt_;

      // Normalize yaw
      while (pred.yaw > M_PI) pred.yaw -= 2.0 * M_PI;
      while (pred.yaw < -M_PI) pred.yaw += 2.0 * M_PI;

      // Obstacle cost - check distance from predicted position
      double min_obs_dist = getMinObstacleDistance(robot, pred);
      min_obs_dist_out = std::min(min_obs_dist_out, min_obs_dist);
      if (min_obs_dist < hard_min_obstacle_distance_) {
        return infinity;  // Hard constraint
      }
      if (min_obs_dist < 2.0) {
        total_cost += w_obstacle_ / (min_obs_dist + eps);
      }

      // Social distance cost - track minimum distance along trajectory
      if (last_people_) {
        for (const auto& person : last_people_->people) {
          double dx = person.x - pred.x;
          double dy = person.y - pred.y;
          double dist = std::hypot(dx, dy);
          min_person_dist_out = std::min(min_person_dist_out, dist);
          if (dist < 3.0) {
            total_cost += w_social_ / (dist + eps);
          }
        }
      }
    }

    // Terminal costs
    RobotState final_pred = robot;
    for (int i = 0; i < num_steps; ++i) {
      final_pred.x += v * std::cos(final_pred.yaw) * dt_;
      final_pred.y += v * std::sin(final_pred.yaw) * dt_;
      final_pred.yaw += w * dt_;
    }

    // Goal cost
    double dx_goal = goal_x_ - final_pred.x;
    double dy_goal = goal_y_ - final_pred.y;
    total_cost += w_goal_ * (dx_goal * dx_goal + dy_goal * dy_goal);

    // Heading cost
    double desired_yaw = std::atan2(dy_goal, dx_goal);
    double yaw_error = desired_yaw - final_pred.yaw;
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    total_cost += w_heading_ * (yaw_error * yaw_error);

    // Smoothness cost
    double dv = v - prev_v_;
    double dw = w - prev_w_;
    total_cost += w_smooth_ * (dv * dv + dw * dw);

    return total_cost;
  }

  double getMinObstacleDistance(const RobotState& current_robot, const RobotState& pred_robot)
  {
    if (!last_laser_) {
      return std::numeric_limits<double>::max();
    }

    double min_dist = std::numeric_limits<double>::max();

    // Transform laser scan points from current robot frame to predicted position
    for (size_t i = 0; i < last_laser_->ranges.size(); ++i) {
      float range = last_laser_->ranges[i];

      if (std::isnan(range) || std::isinf(range)) continue;
      if (range < min_valid_laser_range_ || range > last_laser_->range_max) continue;

      // Obstacle position in current robot frame
      double angle = last_laser_->angle_min + i * last_laser_->angle_increment;
      double obs_x_robot = range * std::cos(angle);
      double obs_y_robot = range * std::sin(angle);

      // Transform to map frame
      double obs_x_map = current_robot.x + obs_x_robot * std::cos(current_robot.yaw)
                                        - obs_y_robot * std::sin(current_robot.yaw);
      double obs_y_map = current_robot.y + obs_x_robot * std::sin(current_robot.yaw)
                                        + obs_y_robot * std::cos(current_robot.yaw);

      // Distance from predicted robot position
      double dist = std::hypot(obs_x_map - pred_robot.x, obs_y_map - pred_robot.y);
      min_dist = std::min(min_dist, dist);
    }

    return min_dist;
  }

  double getMinPersonDistance(const RobotState& robot)
  {
    if (!last_people_ || last_people_->people.empty()) {
      return std::numeric_limits<double>::max();
    }

    double min_dist = std::numeric_limits<double>::max();
    for (const auto& person : last_people_->people) {
      double dx = person.x - robot.x;
      double dy = person.y - robot.y;
      double dist = std::hypot(dx, dy);
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  // Parameters
  double goal_x_, goal_y_, goal_tolerance_;
  double control_rate_hz_;
  double dt_, predict_time_;
  double v_max_, omega_max_;
  double v_resolution_, w_resolution_;
  double acc_lim_v_, acc_lim_w_;
  double w_goal_, w_heading_, w_obstacle_, w_social_, w_smooth_;
  double min_obstacle_distance_, hard_min_obstacle_distance_;
  double min_valid_laser_range_;
  std::string map_frame_, base_frame_;
  std::string log_directory_;
  bool log_to_csv_, enable_debug_logging_;

  // State
  double prev_v_{0.0}, prev_w_{0.0};
  bool goal_reached_{false};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<person_tracker::msg::PersonInfoArray>::SharedPtr person_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Data
  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_;
  social_mpc_nav::msg::People2D::SharedPtr last_people_;
  nav_msgs::msg::Path::SharedPtr global_path_;
  size_t current_waypoint_idx_{0};

  // Logging
  std::ofstream dwa_log_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DWAPureController>();
  RCLCPP_INFO(node->get_logger(), "Pure DWA Controller spinning...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
