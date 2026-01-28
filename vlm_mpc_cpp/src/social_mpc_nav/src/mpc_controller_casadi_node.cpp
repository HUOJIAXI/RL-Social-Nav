/**
 * @file mpc_controller_casadi_node.cpp
 * @brief CasADi-based MPC controller with hard safety constraints and VLM integration
 *
 * This controller uses CasADi nonlinear optimization to solve the MPC problem with:
 * - Hard constraints: Collision avoidance, velocity limits
 * - Soft constraints: Social proximity, goal reaching, smoothness
 * - VLM integration: Semantic scene understanding in objective function
 */

#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <vector>
#include <mutex>
#include <optional>
#include <fstream>
#include <filesystem>
#include <iomanip>

#include <casadi/casadi.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "person_tracker/msg/person_info_array.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/msg/vlm_parameters.hpp"
#include "social_mpc_nav/social_contract.hpp"
#include "social_mpc_nav/mpc_vlm_helpers.hpp"

using namespace casadi;

// Forward declaration for signal handler
class MPCControllerCasADiNode;
static MPCControllerCasADiNode* g_node_ptr = nullptr;
void signalHandler(int signum);

namespace
{
struct RobotState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ObstaclePoint
{
  double x;
  double y;
};
}  // namespace

/**
 * @brief CasADi-based MPC controller with optimization-based trajectory planning
 */
class MPCControllerCasADiNode : public rclcpp::Node
{
public:
  /**
   * @brief Emergency stop - publish zero velocity and log
   */
  void emergencyStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "Emergency stop: published zero velocity");
  }

  MPCControllerCasADiNode()
  : Node("mpc_controller_casadi_node")
  {
    // Parameters
    goal_x_ = declare_parameter<double>("goal_x", 10.0);
    goal_y_ = declare_parameter<double>("goal_y", 5.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 1.0);

    dt_ = declare_parameter<double>("dt", 0.3);  // Larger timestep
    N_ = declare_parameter<int>("N", 6);  // Much smaller horizon for tractability
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);

    default_v_max_ = declare_parameter<double>("default_v_max", 0.6);
    omega_max_ = declare_parameter<double>("omega_max", 0.9);
    v_min_ = declare_parameter<double>("v_min", 0.0);  // Minimum velocity (0 allows stopping)

    // Acceleration limits for smooth control
    v_accel_max_ = declare_parameter<double>("v_accel_max", 0.5);  // m/s^2
    omega_accel_max_ = declare_parameter<double>("omega_accel_max", 1.5);  // rad/s^2

    w_goal_ = declare_parameter<double>("w_goal", 20.0);  // Terminal goal weight (increased to favor progress)
    w_smooth_ = declare_parameter<double>("w_smooth", 0.1);  // Smoothness weight (matches simple MPC)
    w_obstacle_ = declare_parameter<double>("w_obstacle", 1.0);  // Obstacle avoidance (reduced to allow forward motion)

    // Orientation control parameters
    w_terminal_orientation_ = declare_parameter<double>("w_terminal_orientation", 6.0);
    w_running_orientation_ = declare_parameter<double>("w_running_orientation", 3.0);
    orientation_activation_distance_ = declare_parameter<double>("orientation_activation_distance", 2.0);
    rotation_penalty_distance_threshold_ = declare_parameter<double>("rotation_penalty_distance_threshold", 1.5);

    // VLM integration
    enable_vlm_ = declare_parameter<bool>("enable_vlm", false);  // VLM parameter adjustment disabled by default

    // VLM cost weights (only used if enable_vlm is true)
    w_vlm_directional_ = declare_parameter<double>("w_vlm_directional", 1.0);
    w_vlm_action_ = declare_parameter<double>("w_vlm_action", 2.0);
    w_vlm_scene_ = declare_parameter<double>("w_vlm_scene", 1.5);
    w_vlm_personal_ = declare_parameter<double>("w_vlm_personal", 5.0);

    min_obstacle_distance_ = declare_parameter<double>("min_obstacle_distance", 0.3);
    hard_min_obstacle_distance_ = declare_parameter<double>("hard_min_obstacle_distance", 0.05);
    min_valid_laser_range_ = declare_parameter<double>("min_valid_laser_range", 0.5);
    min_person_safety_distance_ = declare_parameter<double>("min_person_safety_distance", 0.5);

    // Topics
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/task_generator_node/tiago_base/cmd_vel");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/task_generator_node/tiago_base/lidar");
    crowd_topic_ = declare_parameter<std::string>("crowd_topic", "/person_tracker/person_info");
    global_path_topic_ = declare_parameter<std::string>("global_path_topic", "/global_path");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "tiago_base/base_footprint");

    // Logging
    log_directory_ = declare_parameter<std::string>("log_directory", "~/ros2_logs/social_mpc_nav");
    log_mpc_to_csv_ = declare_parameter<bool>("log_mpc_to_csv", true);

    // Initialize social contract helper
    contract_helper_ = std::make_unique<social_mpc_nav::SocialContractHelper>(
      this->get_logger(), log_directory_, log_mpc_to_csv_);

    // Initialize CasADi optimization problem
    setupCasADiProblem();

    // Publishers and subscribers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    people_sub_ = create_subscription<person_tracker::msg::PersonInfoArray>(
      crowd_topic_, 10,
      std::bind(&MPCControllerCasADiNode::onPersonInfo, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),  // Use sensor QoS (BEST_EFFORT) matching VLM node
      std::bind(&MPCControllerCasADiNode::onScan, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, rclcpp::QoS(10).transient_local(),
      std::bind(&MPCControllerCasADiNode::onPath, this, std::placeholders::_1));

    vlm_params_sub_ = create_subscription<social_mpc_nav::msg::VLMParameters>(
      "/vlm/mpc_parameters", 10,
      std::bind(&MPCControllerCasADiNode::onVLMParameters, this, std::placeholders::_1));

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&MPCControllerCasADiNode::onGoalPose, this, std::placeholders::_1));

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Robot pose update timer (high frequency for smooth updates)
    pose_update_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz
      std::bind(&MPCControllerCasADiNode::updateRobotPose, this));

    // Control timer
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MPCControllerCasADiNode::controlLoop, this));

    // Setup logging
    if (log_mpc_to_csv_) {
      setupLogging();
    }

    // Register signal handler for graceful shutdown
    g_node_ptr = this;
    std::signal(SIGINT, signalHandler);

    RCLCPP_INFO(get_logger(), "CasADi MPC controller started. Goal: (%.2f, %.2f) dt=%.2f N=%d",
                goal_x_, goal_y_, dt_, N_);
    RCLCPP_INFO(get_logger(), "Using CasADi optimization with hard safety constraints");
    RCLCPP_INFO(get_logger(), "VLM integration: %s", enable_vlm_ ? "ENABLED" : "DISABLED (using default/user-defined parameters)");

    // VLM warmup requirement
    if (enable_vlm_) {
      vlm_ready_ = false;  // Must wait for VLM warmup
      RCLCPP_INFO(get_logger(), "⏳ VLM warmup required - waiting for first valid VLM response before navigation...");
      RCLCPP_INFO(get_logger(), "   VLM integration node should send a warmup request on startup");
    } else {
      vlm_ready_ = true;   // VLM disabled, no warmup needed
    }

    RCLCPP_INFO(get_logger(), "Signal handler registered - robot will stop immediately on Ctrl+C");
    RCLCPP_INFO(get_logger(), "✅ Post-optimization safety verification ENABLED");
    RCLCPP_INFO(get_logger(), "   - Obstacle safety distance: %.2fm", min_obstacle_distance_);
    RCLCPP_INFO(get_logger(), "   - Personal space distance: %.2fm", min_person_safety_distance_);
    RCLCPP_INFO(get_logger(), "   - Fallback strategy: Last safe command or emergency stop");
  }

private:
  void setupCasADiProblem()
  {
    // Decision variables: [v_0, w_0, v_1, w_1, ..., v_{N-1}, w_{N-1}]
    // Total: 2*N variables

    opti_ = Opti();

    // Create decision variables
    v_var_ = opti_.variable(N_);  // Linear velocities
    w_var_ = opti_.variable(N_);  // Angular velocities

    // Create parameters (updated each iteration)
    x0_param_ = opti_.parameter();      // Initial x
    y0_param_ = opti_.parameter();      // Initial y
    yaw0_param_ = opti_.parameter();    // Initial yaw

    goal_x_param_ = opti_.parameter();  // Goal x
    goal_y_param_ = opti_.parameter();  // Goal y
    goal_yaw_param_ = opti_.parameter(); // Goal yaw (orientation)

    v_max_param_ = opti_.parameter();   // Max velocity (from social contract)
    w_goal_param_ = opti_.parameter();  // Goal weight (from social contract)
    w_social_param_ = opti_.parameter(); // Social weight (from social contract)

    prev_v_param_ = opti_.parameter();  // Previous linear velocity (for acceleration constraints)
    prev_w_param_ = opti_.parameter();  // Previous angular velocity

    // Orientation control parameters
    w_terminal_orientation_param_ = opti_.parameter();
    w_running_orientation_param_ = opti_.parameter();
    orientation_activation_distance_param_ = opti_.parameter();
    rotation_threshold_param_ = opti_.parameter();

    // VLM cost parameters (numerical modifiers derived from VLM scene understanding)
    vlm_side_preference_param_ = opti_.parameter();     // -1.0 (left), 0.0 (neutral), +1.0 (right)
    vlm_action_speed_penalty_param_ = opti_.parameter(); // 0.0 (go_ahead), 0.5 (slow_down), 1.0 (stop)
    vlm_scene_caution_param_ = opti_.parameter();       // 0.0 (open), 0.5 (corridor), 1.0 (doorway/crossing)

    // Adaptive soft obstacle distance (VLM-modulated based on environment density)
    adaptive_min_obstacle_distance_param_ = opti_.parameter(); // Adjusted based on crowd density/scene

    // Fixed-size arrays for obstacles and people (max capacity)
    // REDUCED for tractability - fewer constraints = more likely to solve
    max_obstacles_ = 5;  // Was 20
    max_people_ = 5;     // Was 10

    obstacles_x_param_ = opti_.parameter(max_obstacles_);
    obstacles_y_param_ = opti_.parameter(max_obstacles_);

    people_x_param_ = opti_.parameter(max_people_);
    people_y_param_ = opti_.parameter(max_people_);

    // ===== BUILD OPTIMIZATION PROBLEM (ONCE!) =====

    // Initialize state trajectory symbolically
    MX x = x0_param_;
    MX y = y0_param_;
    MX yaw = yaw0_param_;

    // Cost function
    MX cost = 0;

    // Simulate forward and accumulate cost
    for (int k = 0; k < N_; ++k) {
      // Store previous position for progress calculation
      MX x_prev = x;
      MX y_prev = y;

      // Dynamics (unicycle model)
      x = x + v_var_(k) * cos(yaw) * dt_;
      y = y + v_var_(k) * sin(yaw) * dt_;
      yaw = yaw + w_var_(k) * dt_;

      // Progress reward: Encourage motion that reduces distance to goal
      // This is critical for optimization-based MPC to avoid local minima
      MX dx_to_goal_before = x_prev - goal_x_param_;
      MX dy_to_goal_before = y_prev - goal_y_param_;
      MX dist_before_sq = dx_to_goal_before * dx_to_goal_before + dy_to_goal_before * dy_to_goal_before;

      MX dx_to_goal_after = x - goal_x_param_;
      MX dy_to_goal_after = y - goal_y_param_;
      MX dist_after_sq = dx_to_goal_after * dx_to_goal_after + dy_to_goal_after * dy_to_goal_after;

      // Reward progress (negative cost when distance decreases)
      MX progress = dist_before_sq - dist_after_sq;  // Positive when getting closer
      cost = cost - 15.0 * progress;  // Strong reward for progress towards goal

      // Forward motion incentive - balanced to encourage motion without forcing max speed
      cost = cost - 2.0 * v_var_(k);  // Moderate reward for forward motion

      // Gentle penalty for near-zero velocity (discourages stopping but allows slow speed)
      MX zero_velocity_penalty = 3.0 / (v_var_(k) + 0.1);  // Penalty when v approaches 0
      cost = cost + zero_velocity_penalty;

      // Penalty for rotating in place (high angular velocity with low linear velocity)
      // Encourages the robot to move forward while turning rather than pure rotation
      MX rotation_penalty = w_var_(k) * w_var_(k) / (v_var_(k) + 0.1);  // Penalty when v is small and w is large
      cost = cost + 0.8 * rotation_penalty;

      // Calculate distance to goal (needed for orientation cost below)
      MX dist_to_goal = sqrt(dx_to_goal_after * dx_to_goal_after + dy_to_goal_after * dy_to_goal_after + 0.01);

      // WEAK orientation guidance: encourage heading toward goal POSITION (not goal_yaw)
      // This prevents curved/arc paths while avoiding detours
      // Only active at medium distances - disabled when very close to prevent unstable direction calculation

      // Distance-based activation: only guide heading when robot is at reasonable distance
      // Too close (< 0.8m): direction-to-goal becomes unstable, disable guidance
      // Medium distance (0.8m - 3m): active guidance to prevent curved paths
      // Far (> 3m): full guidance
      MX distance_activation = 1.0 / (1.0 + exp(-(dist_to_goal - 0.8) * 8.0));

      MX desired_heading_to_goal = atan2(goal_y_param_ - y, goal_x_param_ - x);
      MX heading_error = desired_heading_to_goal - yaw;
      MX heading_error_wrapped = atan2(sin(heading_error), cos(heading_error));

      // Large dead zone: only activate if heading error > 30° (0.52 rad)
      MX heading_deadzone = 0.52;  // ~30 degrees - large tolerance
      MX heading_scale = 1.0 / (1.0 + exp(-(fabs(heading_error_wrapped) - heading_deadzone) * 10.0));

      // Very weak weight to nudge toward goal without forcing, disabled when close
      // DISABLED: This orientation guidance was causing detours when approaching the goal (~2m)
      // The progress reward and terminal cost are sufficient for goal-directed navigation
      // cost = cost + distance_activation * heading_scale * 0.8 * heading_error_wrapped * heading_error_wrapped;

      // Smoothness cost (rate of change between steps) - matches simple MPC
      if (k > 0) {
        cost = cost + w_smooth_ * (pow(v_var_(k) - v_var_(k-1), 2) +
                                    pow(w_var_(k) - w_var_(k-1), 2));
      }

      // Social cost (soft constraint) - repel from people
      // Note: Inactive people will be at position (1e6, 1e6), so dist is huge and cost → 0
      for (int i = 0; i < max_people_; ++i) {
        MX dx_person = x - people_x_param_(i);
        MX dy_person = y - people_y_param_(i);
        MX dist_sq = dx_person * dx_person + dy_person * dy_person + 0.01;

        // Inverse distance penalty (like simple MPC)
        MX dist = sqrt(dist_sq);
        cost = cost + w_social_param_ * (1.0 / (dist + 0.1));
      }

      // Obstacle cost (soft penalty, heavy if collision)
      // Note: Inactive obstacles will be at position (1e6, 1e6), so dist is huge and cost → 0
      for (int i = 0; i < max_obstacles_; ++i) {
        MX dx_obs = x - obstacles_x_param_(i);
        MX dy_obs = y - obstacles_y_param_(i);
        MX dist_sq_obs = dx_obs * dx_obs + dy_obs * dy_obs + 0.01;

        MX dist_obs = sqrt(dist_sq_obs);

        // HARD SAFETY CONSTRAINT: Robot CANNOT get closer than hard_min_obstacle_distance_
        // This ensures safety regardless of VLM recommendations or soft constraint tuning
        // Note: Inactive obstacles at (1e6, 1e6) have dist_obs ~ 1e6, so constraint is trivially satisfied
        opti_.subject_to(dist_obs >= hard_min_obstacle_distance_);

        // Adaptive soft barrier function (VLM-modulated based on environment density)
        // Uses exponential penalty that grows smoothly as distance decreases
        // Dense environment → smaller threshold (allows tighter navigation)
        // Sparse environment → larger threshold (maintains comfortable distance)
        MX safety_margin = adaptive_min_obstacle_distance_param_ * 1.2;  // Slightly larger margin for smoothness
        MX barrier_penalty = if_else(
          dist_obs < safety_margin,
          50.0 * exp(-dist_obs + adaptive_min_obstacle_distance_param_),  // VLM-adaptive exponential barrier
          0.0
        );
        cost = cost + barrier_penalty;

        // Inverse distance penalty ONLY for nearby obstacles (within 1m)
        // This prevents distant obstacles from dominating the cost function
        MX obstacle_influence_range = 1.0;  // Only penalize obstacles within 1 meter
        MX nearby_penalty = if_else(
          dist_obs < obstacle_influence_range,
          w_obstacle_ * (1.0 / (dist_obs + 0.1)),
          0.0
        );
        cost = cost + nearby_penalty;
      }

      // ===== VLM-BASED COSTS =====
      // These costs integrate VLM scene understanding into navigation

      // 1. Directional preference cost (side_preference: left/right)
      // Encourages robot to stay on preferred side (e.g., in corridors, doorways)
      // vlm_side_preference_param: -1.0 (prefer left), 0.0 (neutral), +1.0 (prefer right)
      // Calculate lateral position relative to path to goal
      MX dx_to_goal_current = goal_x_param_ - x;
      MX dy_to_goal_current = goal_y_param_ - y;
      MX path_direction = atan2(dy_to_goal_current, dx_to_goal_current);

      // Lateral offset in path frame (positive = right of path, negative = left of path)
      // This approximates lateral deviation from the direct line to goal
      MX lateral_offset = (y - y0_param_) * cos(path_direction) - (x - x0_param_) * sin(path_direction);

      // Penalize being on the wrong side:
      // The negative sign ensures correct penalty/reward behavior:
      // If prefer_left (-1.0): negative lateral (left) reduces cost (reward), positive lateral (right) increases cost (penalty)
      // If prefer_right (+1.0): positive lateral (right) reduces cost (reward), negative lateral (left) increases cost (penalty)
      MX directional_cost = -w_vlm_directional_ * vlm_side_preference_param_ * lateral_offset;
      cost = cost + directional_cost;

      // 2. Action-based speed penalty (recommended_action: go_ahead/slow_down/stop_and_wait)
      // vlm_action_speed_penalty: 0.0 (go_ahead), 0.5 (slow_down), 1.0 (stop_and_wait)
      // Higher penalty = discourage speed more strongly
      MX action_cost = w_vlm_action_ * vlm_action_speed_penalty_param_ * v_var_(k);
      cost = cost + action_cost;

      // 3. Scene-specific caution (scene_type: open/corridor/doorway/crossing)
      // vlm_scene_caution: 0.0 (open space), 0.5 (corridor), 1.0 (doorway/crossing/queue)
      // Penalize speeds above cautious threshold (0.3 m/s) in constrained scenes
      MX cautious_speed = 0.3;  // Desired speed in cautious scenes
      MX speed_excess = v_var_(k) - cautious_speed;
      MX scene_cost = w_vlm_scene_ * vlm_scene_caution_param_ *
                      if_else(speed_excess > 0.0, speed_excess * speed_excess, 0.0);
      cost = cost + scene_cost;
    }

    // Terminal goal cost (position + orientation)
    MX dx_final = x - goal_x_param_;
    MX dy_final = y - goal_y_param_;
    cost = cost + w_goal_param_ * (dx_final * dx_final + dy_final * dy_final);

    // Terminal orientation cost DISABLED - orientation alignment is handled separately
    // MPC only navigates to goal POSITION, then a separate controller handles turning to goal_yaw
    // This prevents detours and keeps the MPC focused on position reaching
    // (Terminal orientation cost removed entirely - goal_yaw alignment done post-navigation)

    // Soft acceleration penalties (instead of hard constraints for speed)
    double w_accel = 0.5;  // Weight for acceleration penalty
    cost = cost + w_accel * pow(v_var_(0) - prev_v_param_, 2);
    cost = cost + w_accel * pow(w_var_(0) - prev_w_param_, 2);

    // Set objective
    opti_.minimize(cost);

    // Store cost expression for later extraction
    cost_expr_ = cost;

    // Hard constraints: Velocity limits
    opti_.subject_to(v_var_ >= v_min_);
    opti_.subject_to(v_var_ <= v_max_param_);
    opti_.subject_to(w_var_ >= -omega_max_);
    opti_.subject_to(w_var_ <= omega_max_);

    // Solver options - IPOPT for robustness with non-convex problem
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.sb"] = "yes";  // Suppress IPOPT banner

    // Accept solutions even if not fully converged (best effort)
    opts["error_on_fail"] = false;  // Don't throw error, return best available solution

    // Speed optimizations for IPOPT - balanced for small horizon
    opts["ipopt.max_iter"] = 150;  // Increased for complex scenarios with obstacles
    opts["ipopt.tol"] = 1e-2;  // Reasonable tolerance
    opts["ipopt.acceptable_tol"] = 5e-1;  // More lenient for real-time MPC
    opts["ipopt.acceptable_iter"] = 5;  // Accept after 5 iterations at acceptable level
    opts["ipopt.acceptable_obj_change_tol"] = 1e-1;  // More lenient objective change

    // Warm start settings (critical for speed)
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.warm_start_bound_push"] = 1e-6;
    opts["ipopt.warm_start_mult_bound_push"] = 1e-6;
    opts["ipopt.mu_init"] = 1e-2;

    // Adaptive barrier strategy for faster convergence
    opts["ipopt.mu_strategy"] = "adaptive";
    opts["ipopt.adaptive_mu_globalization"] = "kkt-error";

    // Increase accept limits to prevent assertion failures
    opts["ipopt.acceptable_constr_viol_tol"] = 1e-1;
    opts["ipopt.acceptable_dual_inf_tol"] = 1e10;
    opts["ipopt.acceptable_compl_inf_tol"] = 1e-1;

    // Linear solver (mumps is default, but ma27 is faster if available)
    opts["ipopt.linear_solver"] = "mumps";

    // Hessian approximation for speed (limited-memory is faster)
    opts["ipopt.hessian_approximation"] = "limited-memory";  // Much faster than exact Hessian

    opti_.solver("ipopt", opts);

    RCLCPP_INFO(get_logger(), "CasADi problem setup complete (IPOPT solver with limited-memory Hessian, max_obstacles=%d, max_people=%d)",
                max_obstacles_, max_people_);
  }

  void setupLogging()
  {
    // Expand ~ to home directory
    std::string log_dir = log_directory_;
    if (log_dir[0] == '~') {
      const char* home = getenv("HOME");
      if (home) {
        log_dir = std::string(home) + log_dir.substr(1);
      }
    }

    // Create directory if it doesn't exist
    std::filesystem::create_directories(log_dir);

    // Open log file
    std::string log_file = log_dir + "/mpc_casadi_log.csv";
    log_stream_.open(log_file, std::ios::out);

    if (log_stream_.good()) {
      log_stream_ << "timestamp,x,y,yaw,v_cmd,w_cmd,goal_dist,solve_time_ms,cost,"
                  << "safety_verified,min_obs_dist,min_person_dist,"
                  << "total_checks,total_violations,violation_rate\n";
      RCLCPP_INFO(get_logger(), "CasADi MPC logging to: %s", log_file.c_str());
    }
  }

  void updateRobotPose()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Directly query TF tree for robot position in map frame
    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(
          map_frame_,
          base_link_frame_,
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
        base_link_frame_.c_str(), ex.what());
      pose_received_ = false;
    }
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

  void onVLMParameters(const social_mpc_nav::msg::VLMParameters::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(vlm_params_mutex_);
    latest_vlm_params_ = msg;

    // Check if VLM is warmed up and ready (received valid VLM output, not fallback)
    if (enable_vlm_ && !vlm_ready_ && msg->source == "vlm") {
      vlm_ready_ = true;
      RCLCPP_INFO(get_logger(),
        "✅ VLM WARMED UP and READY! Received first valid VLM response (confidence: %.2f, scene: %s)",
        msg->confidence, msg->scene_type.c_str());
      RCLCPP_INFO(get_logger(),
        "   VLM is now providing scene understanding. Navigation can proceed when goal is set.");
    }
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Transform goal pose to map frame if needed
    geometry_msgs::msg::PoseStamped goal_in_map;
    try {
      if (msg->header.frame_id.empty() || msg->header.frame_id == map_frame_) {
        // Goal is already in map frame or frame_id is not specified (assume map frame)
        goal_in_map = *msg;
      } else {
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

      RCLCPP_INFO(get_logger(),
                  "🎯 New goal received from RViz2: (%.2f, %.2f, yaw: %.2f°) in %s frame - Starting navigation",
                  goal_x_, goal_y_, goal_yaw_ * 180.0 / M_PI, map_frame_.c_str());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(),
                  "Failed to transform goal pose from %s to %s: %s",
                  msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    }
  }

  void controlLoop()
  {
    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for robot pose from TF...");
      return;
    }

    // Wait for goal from RViz2 before starting navigation
    if (!goal_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "⏸️  Waiting for goal from RViz2 (use 2D Goal Pose tool)...");
      return;
    }

    // Wait for VLM warmup before starting navigation (only if VLM enabled)
    if (enable_vlm_ && !vlm_ready_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "⏳ Waiting for VLM warmup... (waiting for first valid VLM response)");
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "   VLM integration node should be processing camera images and warming up API");
      return;
    }

    // Get robot state in map frame
    RobotState robot;
    if (!getRobotStateInMap(robot)) {
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
    if (dist_to_final_goal < goal_tolerance_) {
      // Position reached, now align orientation

      // Check for ±180° singularity ONLY when very close (within 5° of exactly opposite)
      // This catches 175-180° where oscillation actually occurs
      const double singularity_threshold = 0.087;  // ~5° margin (very tight!)
      if (std::abs(yaw_error) > (M_PI - singularity_threshold)) {  // Error > 175° or < -175°
        if (!goal_reached_) {
          RCLCPP_WARN(
            get_logger(),
            "⚠️  Orientation %.1f° apart (exactly opposite) - skipping alignment to avoid oscillation",
            std::abs(yaw_error) * 180.0 / M_PI);
          RCLCPP_INFO(
            get_logger(),
            "✅ GOAL POSITION REACHED! Orientation alignment skipped (±180° singularity)");
          goal_reached_ = true;
        }
        publishZeroVelocity();
        return;
      }

      if (std::abs(yaw_error) > orientation_tolerance) {
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

        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "🔄 Position reached, aligning orientation: %.1f° remaining (ω=%.2f rad/s)",
          std::abs(yaw_error) * 180.0 / M_PI, cmd.angular.z);
        return;
      }

      // Both position and orientation reached
      if (!goal_reached_) {
        RCLCPP_INFO(
          get_logger(),
          "✅ GOAL FULLY REACHED! Position: %.3fm, Orientation: %.1f° - Stopping robot",
          dist_to_final_goal, std::abs(yaw_error) * 180.0 / M_PI);
        goal_reached_ = true;
      }
      publishZeroVelocity();
      return;
    }

    // Reset goal_reached flag if we've moved away from goal (e.g., new goal set)
    if (goal_reached_ && dist_to_final_goal > goal_tolerance_ * 2.0)
    {
      RCLCPP_INFO(get_logger(), "Resuming navigation - distance to goal: %.2fm", dist_to_final_goal);
      goal_reached_ = false;
    }

    // Get social contract
    social_mpc_nav::msg::People2D::SharedPtr people;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      people = latest_people_;
    }
    auto contract = contract_helper_->compute(
      robot.x, robot.y, robot.yaw, default_v_max_, people);
    
    // Adaptive velocity reduction when approaching waypoint
    // This prevents overshooting and reduces oscillation
    if (tracking_waypoint) {
      const double dist_to_target = std::hypot(robot.x - target_x, robot.y - target_y);
      const double slowdown_distance = 1.0;  // Start slowing down at 1.0m from waypoint
      const double min_approach_velocity = 0.15;  // Minimum velocity when very close
      
      if (dist_to_target < slowdown_distance) {
        // Linear velocity reduction as we approach waypoint
        const double velocity_scale = std::max(0.0, dist_to_target / slowdown_distance);
        contract.v_max = min_approach_velocity + 
                         (contract.v_max - min_approach_velocity) * velocity_scale;
        
        RCLCPP_DEBUG(get_logger(),
          "Approaching waypoint (%.2fm) - reducing v_max to %.2f",
          dist_to_target, contract.v_max);
      }
    }

    // VLM modulation of social contract (only if enabled and NOT in fallback)
    if (enable_vlm_) {
      std::lock_guard<std::mutex> lock(vlm_params_mutex_);
      if (latest_vlm_params_) {
        // Check if VLM is in fallback mode
        bool is_fallback = (latest_vlm_params_->source != "vlm");

        if (is_fallback) {
          // VLM in fallback - use default MPC parameters (no adjustments)
          RCLCPP_DEBUG(get_logger(), "VLM in fallback mode (%s) - using default MPC parameters",
                       latest_vlm_params_->source.c_str());
          // Contract remains at default values computed above
        } else {
          // VLM active with new output - apply adjustments
          contract.v_max *= latest_vlm_params_->speed_scale;
          contract.w_social *= (latest_vlm_params_->min_personal_distance / 1.0);
          if (latest_vlm_params_->need_to_wait) {
            contract.v_max = 0.0;
            contract.w_social = 10.0;
          }
          // Convert VLM string parameters to numerical cost modifiers
          // VLM translator provides strings; we convert to numerical values for CasADi cost function

          // 1. Side preference: "left" → -1.0, "right" → +1.0, "neutral" → 0.0
          vlm_side_preference_value_ = 0.0;  // Default: neutral
          if (latest_vlm_params_->side_preference == "left") {
            vlm_side_preference_value_ = -1.0;
          } else if (latest_vlm_params_->side_preference == "right") {
            vlm_side_preference_value_ = 1.0;
          }

          // 2. Recommended action: Maps to speed penalty (0.0 = go freely, 1.0 = stop)
          vlm_action_speed_penalty_value_ = 0.0;  // Default: go ahead
          const std::string& action = latest_vlm_params_->recommended_action;
          if (action == "stop_and_wait") {
            vlm_action_speed_penalty_value_ = 1.0;  // Strong penalty on speed
          } else if (action == "slow_down_and_go") {
            vlm_action_speed_penalty_value_ = 0.5;  // Moderate penalty
          } else if (action == "yield_to_pedestrian") {
            vlm_action_speed_penalty_value_ = 0.7;  // High penalty
          } else if (action == "keep_left" || action == "keep_right") {
            vlm_action_speed_penalty_value_ = 0.2;  // Slight caution
          }
          // "go_ahead" → 0.0 (no penalty)

          // 3. Scene type: Maps to caution level (0.0 = open, 1.0 = highly constrained)
          vlm_scene_caution_value_ = 0.0;  // Default: open space
          const std::string& scene = latest_vlm_params_->scene_type;
          if (scene == "doorway" || scene == "crossing" || scene == "queue") {
            vlm_scene_caution_value_ = 1.0;  // High caution
          } else if (scene == "corridor" || scene == "lobby") {
            vlm_scene_caution_value_ = 0.5;  // Moderate caution
          }
          // "open_space", "unknown" → 0.0 (low caution)

          // 4. Adaptive soft obstacle distance (computed by VLM translator)
          // Dense environment → smaller threshold (allows tighter navigation)
          // Sparse environment → larger threshold (maintains comfortable distance)
          adaptive_min_obstacle_distance_value_ = latest_vlm_params_->adaptive_min_obstacle_distance;

          // Ensure it's always above hard constraint
          adaptive_min_obstacle_distance_value_ = std::max(
            adaptive_min_obstacle_distance_value_,
            hard_min_obstacle_distance_ + 0.05);  // At least 5cm above hard constraint

          RCLCPP_DEBUG(get_logger(), "VLM adjustments applied: speed_scale=%.2f, min_personal_distance=%.2f",
                       latest_vlm_params_->speed_scale, latest_vlm_params_->min_personal_distance);
          RCLCPP_DEBUG(get_logger(),
            "VLM cost modifiers: side_pref=%.1f, action_penalty=%.1f, scene_caution=%.1f, adaptive_obs_dist=%.2fm",
            vlm_side_preference_value_, vlm_action_speed_penalty_value_, vlm_scene_caution_value_,
            adaptive_min_obstacle_distance_value_);
        }
      }
    }

    // VLM fallback: Set neutral/zero values if VLM disabled or in fallback
    if (!enable_vlm_ || !latest_vlm_params_ ||
        (latest_vlm_params_ && latest_vlm_params_->source != "vlm")) {
      vlm_side_preference_value_ = 0.0;
      vlm_action_speed_penalty_value_ = 0.0;
      vlm_scene_caution_value_ = 0.0;
      adaptive_min_obstacle_distance_value_ = min_obstacle_distance_;  // Use default config value
    }

    // Extract obstacles from laser scan
    std::vector<ObstaclePoint> obstacles = extractObstacles(robot);

    // Temporarily override goal for MPC to track waypoint
    const double original_goal_x = goal_x_;
    const double original_goal_y = goal_y_;
    
    // Check if target changed significantly (waypoint switch)
    const double target_change = std::hypot(target_x - original_goal_x, target_y - original_goal_y);
    static double prev_target_x = target_x;
    static double prev_target_y = target_y;
    const double target_switch_dist = std::hypot(target_x - prev_target_x, target_y - prev_target_y);
    
    // If target switched significantly, reset warm start to avoid instability
    // But only if the switch is large enough (avoid resetting on small changes)
    if (target_switch_dist > 1.0) {
      RCLCPP_DEBUG(get_logger(), "Target switched significantly (%.2fm), resetting warm start", target_switch_dist);
      has_previous_solution_ = false;  // Reset warm start
    }
    prev_target_x = target_x;
    prev_target_y = target_y;
    
    goal_x_ = target_x;
    goal_y_ = target_y;

    // Solve CasADi optimization problem
    auto start_time = std::chrono::high_resolution_clock::now();

    geometry_msgs::msg::Twist cmd;
    double cost = 0.0;
    double min_obs_dist_log = 0.0;
    double min_person_dist_log = 0.0;
    bool safety_verified_log = false;

    try {
      solveMPC(robot, contract, obstacles, people, cmd, cost, min_obs_dist_log, min_person_dist_log);
      // Safety metrics are now returned by solveMPC
      safety_verified_log = (cost != std::numeric_limits<double>::infinity());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "CasADi solver failed: %s", e.what());
      publishZeroVelocity();
      // Restore original goal before returning
      goal_x_ = original_goal_x;
      goal_y_ = original_goal_y;
      return;
    }

    // Restore original goal after MPC
    goal_x_ = original_goal_x;
    goal_y_ = original_goal_y;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double solve_time_ms = duration.count() / 1000.0;

    // Publish command
    cmd_vel_pub_->publish(cmd);

    // Store previous command for acceleration constraints
    prev_v_cmd_ = cmd.linear.x;
    prev_w_cmd_ = cmd.angular.z;

    // Calculate safety metrics for logging
    uint32_t total_safety_checks = safety_checks_passed_ + safety_violations_;
    double violation_rate = total_safety_checks > 0 ?
                           (static_cast<double>(safety_violations_) / total_safety_checks) : 0.0;

    // Logging
    if (log_mpc_to_csv_ && log_stream_.good()) {
      log_stream_ << std::fixed << std::setprecision(3)
                  << now().seconds() << ","
                  << robot.x << "," << robot.y << "," << robot.yaw << ","
                  << cmd.linear.x << "," << cmd.angular.z << ","
                  << dist_to_final_goal << "," << solve_time_ms << "," << cost << ","
                  << (safety_verified_log ? 1 : 0) << ","
                  << min_obs_dist_log << ","
                  << min_person_dist_log << ","
                  << total_safety_checks << ","
                  << safety_violations_ << ","
                  << violation_rate << "\n";
    }

    // Periodic safety statistics reporting
    static int safety_log_counter = 0;
    if (++safety_log_counter % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "📊 Safety Stats: %u checks, %u violations (%.2f%% rate) | "
        "Current: %s",
        total_safety_checks, safety_violations_, violation_rate * 100.0,
        safety_verified_log ? "SAFE ✅" : "UNSAFE ⚠️");
    }

    // Log navigation status with context
    if (tracking_waypoint) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "🚀 CasADi MPC: Waypoint %zu/%zu (%.2fm) | Final goal: %.2fm | v=%.2f m/s, ω=%.2f rad/s | Solve: %.1fms",
        current_waypoint_index_ + 1,
        global_path_ ? global_path_->poses.size() : 0,
        dist_to_target,
        dist_to_final_goal,
        cmd.linear.x, cmd.angular.z, solve_time_ms);
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "🚀 CasADi MPC: Goal %.2fm | v=%.2f m/s, ω=%.2f rad/s | Solve: %.1fms | Cost: %.2f",
                           dist_to_final_goal, cmd.linear.x, cmd.angular.z, solve_time_ms, cost);
    }
  }

  bool getRobotStateInMap(RobotState& robot)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Waiting for robot pose from TF...");
      return false;
    }

    // Use cached pose from updateRobotPose (already in map frame)
    robot.x = current_robot_x_;
    robot.y = current_robot_y_;
    robot.yaw = current_robot_yaw_;

    return true;
  }

  std::vector<ObstaclePoint> extractObstacles(const RobotState& robot)
  {
    std::vector<ObstaclePoint> obstacles;

    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      scan = latest_scan_;
    }

    if (!scan || scan->ranges.empty()) {
      return obstacles;
    }

    const double forward_cone_angle = M_PI / 2.0;  // 90 degrees
    const int max_obstacles = 20;  // Limit for faster optimization

    // Collect all candidate obstacles
    std::vector<std::pair<double, ObstaclePoint>> obstacle_distances;

    for (size_t i = 0; i < scan->ranges.size(); i += 2) {  // Downsample by 2x
      const float range = scan->ranges[i];

      if (std::isnan(range) || std::isinf(range) ||
          range < min_valid_laser_range_ || range > scan->range_max) {
        continue;
      }

      const double ray_angle = scan->angle_min + i * scan->angle_increment;

      // Only consider obstacles in forward cone
      if (std::abs(ray_angle) > forward_cone_angle / 2.0) {
        continue;
      }

      // Convert to global coordinates
      const double local_x = range * std::cos(ray_angle);
      const double local_y = range * std::sin(ray_angle);

      const double global_x = robot.x + local_x * std::cos(robot.yaw) - local_y * std::sin(robot.yaw);
      const double global_y = robot.y + local_x * std::sin(robot.yaw) + local_y * std::cos(robot.yaw);

      double dist = std::hypot(global_x - robot.x, global_y - robot.y);
      obstacle_distances.push_back({dist, {global_x, global_y}});
    }

    // Sort by distance and take closest N obstacles
    std::sort(obstacle_distances.begin(), obstacle_distances.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (size_t i = 0; i < std::min(obstacle_distances.size(), static_cast<size_t>(max_obstacles)); ++i) {
      obstacles.push_back(obstacle_distances[i].second);
    }

    return obstacles;
  }

  /**
   * @brief Verify that the optimized trajectory is safe
   *
   * Propagates the robot state using the control sequence and checks for:
   * - Collision with obstacles (distance >= min_obstacle_distance)
   * - Personal space violations with people (distance >= min_person_safety_distance)
   *
   * @param robot Current robot state
   * @param v_sol Linear velocity control sequence
   * @param w_sol Angular velocity control sequence
   * @param obstacles List of obstacle positions
   * @param people List of detected people
   * @param min_obs_dist [output] Minimum distance to obstacles along trajectory
   * @param min_person_dist [output] Minimum distance to people along trajectory
   * @return true if trajectory is safe, false otherwise
   */
  bool verifySolutionSafety(
    const RobotState& robot,
    const std::vector<double>& v_sol,
    const std::vector<double>& w_sol,
    const std::vector<ObstaclePoint>& obstacles,
    const social_mpc_nav::msg::People2D::SharedPtr& people,
    double& min_obs_dist,
    double& min_person_dist)
  {
    // Initialize with large values
    min_obs_dist = std::numeric_limits<double>::infinity();
    min_person_dist = std::numeric_limits<double>::infinity();

    // Propagate trajectory and check constraints
    RobotState pred = robot;
    bool is_safe = true;

    for (size_t k = 0; k < v_sol.size(); ++k) {
      // Propagate dynamics (unicycle model)
      pred.x += v_sol[k] * std::cos(pred.yaw) * dt_;
      pred.y += v_sol[k] * std::sin(pred.yaw) * dt_;
      pred.yaw += w_sol[k] * dt_;

      // Check collision with obstacles
      for (const auto& obs : obstacles) {
        double dist = std::hypot(pred.x - obs.x, pred.y - obs.y);
        min_obs_dist = std::min(min_obs_dist, dist);

        if (dist < min_obstacle_distance_) {
          RCLCPP_DEBUG(get_logger(),
            "Safety violation at step %zu: Obstacle at (%.2f, %.2f), "
            "predicted robot at (%.2f, %.2f), dist=%.3fm < %.2fm",
            k, obs.x, obs.y, pred.x, pred.y, dist, min_obstacle_distance_);
          is_safe = false;
        }
      }

      // Check personal space violations with people
      if (people && !people->people.empty()) {
        for (const auto& person : people->people) {
          double dist = std::hypot(pred.x - person.x, pred.y - person.y);
          min_person_dist = std::min(min_person_dist, dist);

          if (dist < min_person_safety_distance_) {
            RCLCPP_DEBUG(get_logger(),
              "Safety violation at step %zu: Person at (%.2f, %.2f), "
              "predicted robot at (%.2f, %.2f), dist=%.3fm < %.2fm",
              k, person.x, person.y, pred.x, pred.y, dist, min_person_safety_distance_);
            is_safe = false;
          }
        }
      }
    }

    return is_safe;
  }

  void solveMPC(const RobotState& robot,
                const social_mpc_nav::SocialContract& contract,
                const std::vector<ObstaclePoint>& obstacles,
                const social_mpc_nav::msg::People2D::SharedPtr& people,
                geometry_msgs::msg::Twist& cmd,
                double& total_cost,
                double& min_obs_dist_out,
                double& min_person_dist_out)
  {
    // Debug: Log initial state and goal
    static bool first_call = true;
    if (first_call) {
      RCLCPP_INFO(get_logger(), "=== CasADi MPC First Call Debug ===");
      RCLCPP_INFO(get_logger(), "Robot: (%.2f, %.2f, yaw=%.2f rad = %.1f deg)",
                  robot.x, robot.y, robot.yaw, robot.yaw * 180.0 / M_PI);
      RCLCPP_INFO(get_logger(), "Goal:  (%.2f, %.2f)", goal_x_, goal_y_);

      double dist = std::hypot(robot.x - goal_x_, robot.y - goal_y_);
      double desired_yaw = std::atan2(goal_y_ - robot.y, goal_x_ - robot.x);
      double yaw_error = desired_yaw - robot.yaw;
      // Normalize to [-pi, pi]
      while (yaw_error > M_PI) yaw_error -= 2*M_PI;
      while (yaw_error < -M_PI) yaw_error += 2*M_PI;

      RCLCPP_INFO(get_logger(), "Distance to goal: %.2fm", dist);
      RCLCPP_INFO(get_logger(), "Desired heading: %.2f rad (%.1f deg)",
                  desired_yaw, desired_yaw * 180.0 / M_PI);
      RCLCPP_INFO(get_logger(), "Heading error: %.2f rad (%.1f deg) - Need to turn %s",
                  yaw_error, yaw_error * 180.0 / M_PI,
                  yaw_error > 0 ? "LEFT" : "RIGHT");

      RCLCPP_INFO(get_logger(), "Cost weights: w_goal=%.2f (terminal only), w_social=%.2f, w_obstacle=%.2f, w_smooth=%.2f",
                  contract.w_goal, contract.w_social, w_obstacle_, w_smooth_);
      RCLCPP_INFO(get_logger(), "Contract: v_max=%.2f m/s (velocity limit)", contract.v_max);
      RCLCPP_INFO(get_logger(), "Obstacles detected: %zu", obstacles.size());
      first_call = false;
    }

    // ===== ONLY UPDATE PARAMETERS (NO RECONSTRUCTION!) =====

    // Set state parameters
    opti_.set_value(x0_param_, robot.x);
    opti_.set_value(y0_param_, robot.y);
    opti_.set_value(yaw0_param_, robot.yaw);

    // Set goal parameters
    opti_.set_value(goal_x_param_, goal_x_);
    opti_.set_value(goal_y_param_, goal_y_);
    opti_.set_value(goal_yaw_param_, goal_yaw_);

    // Set social contract parameters
    opti_.set_value(v_max_param_, contract.v_max);
    opti_.set_value(w_goal_param_, contract.w_goal);
    opti_.set_value(w_social_param_, contract.w_social);

    // Set orientation control parameters
    opti_.set_value(w_terminal_orientation_param_, w_terminal_orientation_);
    opti_.set_value(w_running_orientation_param_, w_running_orientation_);
    opti_.set_value(orientation_activation_distance_param_, orientation_activation_distance_);
    opti_.set_value(rotation_threshold_param_, rotation_penalty_distance_threshold_);

    // Set VLM cost modifier parameters
    opti_.set_value(vlm_side_preference_param_, vlm_side_preference_value_);
    opti_.set_value(vlm_action_speed_penalty_param_, vlm_action_speed_penalty_value_);
    opti_.set_value(vlm_scene_caution_param_, vlm_scene_caution_value_);
    opti_.set_value(adaptive_min_obstacle_distance_param_, adaptive_min_obstacle_distance_value_);

    // Diagnostic logging (throttled)
    static int diag_counter = 0;
    if (diag_counter++ % 20 == 0) {  // Log every 20 iterations
      size_t num_people = people ? people->people.size() : 0;
      RCLCPP_INFO(get_logger(), "[MPC Diagnostics] Robot pos: (%.2f, %.2f, yaw=%.2f°)",
                  robot.x, robot.y, robot.yaw * 180.0 / M_PI);
      RCLCPP_INFO(get_logger(), "[MPC Diagnostics] v_max=%.3f, obstacles=%zu, people=%zu, prev_v_cmd=%.3f",
                  contract.v_max, obstacles.size(), num_people, prev_v_cmd_);
      if (!obstacles.empty()) {
        double min_obs_dist = 1000.0;
        for (const auto& obs : obstacles) {
          double dist = std::hypot(obs.x - robot.x, obs.y - robot.y);
          min_obs_dist = std::min(min_obs_dist, dist);
        }
        RCLCPP_INFO(get_logger(), "[MPC Diagnostics] Closest obstacle: %.2fm", min_obs_dist);
      }
    }

    // Set previous control for acceleration penalty
    opti_.set_value(prev_v_param_, prev_v_cmd_);
    opti_.set_value(prev_w_param_, prev_w_cmd_);

    // Prepare obstacle parameters (pad with far-away positions for inactive ones)
    const double far_away = 1e6;  // Very far away so cost → 0
    std::vector<double> obs_x(max_obstacles_, far_away);
    std::vector<double> obs_y(max_obstacles_, far_away);
    size_t num_obs = std::min(obstacles.size(), static_cast<size_t>(max_obstacles_));
    for (size_t i = 0; i < num_obs; ++i) {
      obs_x[i] = obstacles[i].x;
      obs_y[i] = obstacles[i].y;
    }
    opti_.set_value(obstacles_x_param_, obs_x);
    opti_.set_value(obstacles_y_param_, obs_y);

    // Prepare people parameters (pad with far-away positions for inactive ones)
    std::vector<double> people_x(max_people_, far_away);
    std::vector<double> people_y(max_people_, far_away);
    size_t num_people = 0;
    if (people && !people->people.empty()) {
      num_people = std::min(people->people.size(), static_cast<size_t>(max_people_));
      for (size_t i = 0; i < num_people; ++i) {
        people_x[i] = people->people[i].x;
        people_y[i] = people->people[i].y;
      }
    }
    opti_.set_value(people_x_param_, people_x);
    opti_.set_value(people_y_param_, people_y);

    // Initial guess: Use previous solution (warm start) or conservative ramp-up
    std::vector<double> v_init(N_);
    std::vector<double> w_init(N_);

    // Check if we're stuck (previous command was zero or very small)
    bool is_stuck = (prev_v_cmd_ < 0.05);
    
    if (has_previous_solution_ && prev_v_sol_.size() >= static_cast<size_t>(N_)
        && prev_w_sol_.size() >= static_cast<size_t>(N_) && !is_stuck) {
      // Warm start: Shift previous solution and extrapolate last value
      for (int k = 0; k < N_ - 1; ++k) {
        v_init[k] = prev_v_sol_[k + 1];  // Shift by one timestep
        w_init[k] = prev_w_sol_[k + 1];
      }
      // Extrapolate last step
      v_init[N_ - 1] = prev_v_sol_[N_ - 1];
      w_init[N_ - 1] = prev_w_sol_[N_ - 1];

      // Ensure not stuck at zero - add small forward bias
      for (int k = 0; k < N_; ++k) {
        if (v_init[k] < 0.1) {
          v_init[k] = 0.2;  // Minimum initial forward velocity
        }
      }
    } else {
      // Cold start: Conservative forward motion towards goal
      double dx = goal_x_ - robot.x;
      double dy = goal_y_ - robot.y;
      double desired_yaw = std::atan2(dy, dx);
      double yaw_error = desired_yaw - robot.yaw;

      // Normalize to [-pi, pi]
      while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

      // If facing roughly towards goal, move forward; otherwise turn first
      // Adaptive initial guess based on distance to goal
      const double dist_to_goal = std::hypot(robot.x - goal_x_, robot.y - goal_y_);
      
      // Reduce speed when close to goal to avoid overshooting
      double max_speed_when_aligned = dist_to_goal < 1.0 ? 0.25 : 0.4;
      double max_speed_when_turning = dist_to_goal < 1.0 ? 0.15 : 0.2;
      
      double target_v = std::abs(yaw_error) < M_PI / 3.0 ? 
                        std::min(max_speed_when_aligned, contract.v_max * 0.7) :  // Higher initial speed when aligned
                        std::min(max_speed_when_turning, contract.v_max * 0.3);    // Lower speed when turning
      double target_w = yaw_error * 0.8;  // More aggressive turn (increased from 0.5)
      target_w = std::max(-omega_max_, std::min(omega_max_, target_w));

      // Adaptive initial velocity based on distance to goal
      // Reduce speed when close to avoid overshooting
      const double dist_to_goal_init = std::hypot(robot.x - goal_x_, robot.y - goal_y_);
      if (dist_to_goal_init < 1.0) {
        // Close to goal - use lower initial velocity
        target_v = std::min(target_v, 0.2);
      }
      
      // Ensure minimum forward velocity to avoid getting stuck
      if (target_v < 0.12) {
        target_v = 0.12;  // Slightly lower minimum to allow more careful approach
      }

      for (int k = 0; k < N_; ++k) {
        v_init[k] = target_v;
        w_init[k] = target_w;
      }
    }

    opti_.set_initial(v_var_, v_init);
    opti_.set_initial(w_var_, w_init);

    // Debug: Log initial guess for first few iterations
    static int init_debug_counter = 0;
    if (init_debug_counter < 3) {
      RCLCPP_INFO(get_logger(), "[MPC Init Guess %d] v_init: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  init_debug_counter,
                  v_init[0], v_init[1], v_init[2], v_init[3], v_init[4], v_init[5]);
      RCLCPP_INFO(get_logger(), "[MPC Init Guess %d] w_init: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  init_debug_counter,
                  w_init[0], w_init[1], w_init[2], w_init[3], w_init[4], w_init[5]);
      init_debug_counter++;
    }

    // Solve (problem was built once in setupCasADiProblem!)
    OptiSol sol = opti_.solve();

    // Extract solution
    std::vector<double> v_sol = sol.value(v_var_).get_elements();
    std::vector<double> w_sol = sol.value(w_var_).get_elements();

    // ========== POST-OPTIMIZATION SAFETY VERIFICATION ==========
    double min_predicted_obstacle_dist = std::numeric_limits<double>::infinity();
    double min_predicted_person_dist = std::numeric_limits<double>::infinity();
    bool trajectory_is_safe = verifySolutionSafety(
      robot, v_sol, w_sol, obstacles, people,
      min_predicted_obstacle_dist, min_predicted_person_dist);

    if (!trajectory_is_safe) {
      // Safety verification FAILED - apply fallback strategy
      safety_violations_++;

      RCLCPP_WARN(get_logger(),
        "⚠️  SAFETY VERIFICATION FAILED! Min obstacle dist: %.3fm (required: %.2fm), "
        "Min person dist: %.3fm (required: %.2fm) - Using fallback",
        min_predicted_obstacle_dist, min_obstacle_distance_,
        min_predicted_person_dist, min_person_safety_distance_);

      // Fallback strategy: Use last safe command if available, else stop
      if (has_safe_fallback_command_) {
        cmd.linear.x = safe_fallback_v_;
        cmd.angular.z = safe_fallback_w_;
        RCLCPP_INFO(get_logger(), "Using last safe command: v=%.2f, w=%.2f",
                    cmd.linear.x, cmd.angular.z);
      } else {
        // No safe fallback - emergency stop
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_ERROR(get_logger(), "No safe fallback available - EMERGENCY STOP");
      }

      // Do NOT store this solution for warm start (it's unsafe)
      // Keep previous safe solution instead

      total_cost = std::numeric_limits<double>::infinity();  // Mark as invalid

    } else {
      // Solution is SAFE - proceed normally
      safety_checks_passed_++;

      // Store solution for next iteration's warm start
      prev_v_sol_ = v_sol;
      prev_w_sol_ = w_sol;
      has_previous_solution_ = true;

      // Apply first control
      cmd.linear.x = v_sol[0];
      cmd.angular.z = w_sol[0];

      // Store as safe fallback for future use
      safe_fallback_v_ = v_sol[0];
      safe_fallback_w_ = w_sol[0];
      has_safe_fallback_command_ = true;

      // Extract actual cost value
      total_cost = static_cast<double>(sol.value(cost_expr_));

      // Log safety margins for analysis
      RCLCPP_DEBUG(get_logger(),
        "✅ Safety verified: Min obstacle dist: %.3fm, Min person dist: %.3fm",
        min_predicted_obstacle_dist, min_predicted_person_dist);
    }
    // ========== END SAFETY VERIFICATION ==========

    // Return safety metrics for logging
    min_obs_dist_out = min_predicted_obstacle_dist;
    min_person_dist_out = min_predicted_person_dist;

    // Debug: Log first few iterations with detailed diagnostics
    static int debug_counter = 0;
    if (debug_counter < 10) {
      // Calculate what would happen if we apply this control
      double pred_x = robot.x + cmd.linear.x * std::cos(robot.yaw) * dt_;
      double pred_y = robot.y + cmd.linear.x * std::sin(robot.yaw) * dt_;
      double pred_yaw = robot.yaw + cmd.angular.z * dt_;

      double new_dist = std::hypot(pred_x - goal_x_, pred_y - goal_y_);
      double current_dist = std::hypot(robot.x - goal_x_, robot.y - goal_y_);
      double progress = current_dist - new_dist;

      RCLCPP_INFO(get_logger(),
                  "[Iter %d] Robot: (%.2f, %.2f, %.1f°) | Cmd: v=%.3f, ω=%.3f | "
                  "Dist: %.2fm → %.2fm (progress: %+.3fm) | Cost: %.2f",
                  debug_counter, robot.x, robot.y, robot.yaw * 180.0 / M_PI,
                  cmd.linear.x, cmd.angular.z,
                  current_dist, new_dist, progress, total_cost);

      RCLCPP_INFO(get_logger(), "  Contract: v_max=%.2f, w_social=%.2f, w_goal=%.2f",
                  contract.v_max, contract.w_social, contract.w_goal);

      // Show full control sequence
      RCLCPP_INFO(get_logger(), "  v_seq: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  v_sol[0],
                  v_sol.size() > 1 ? v_sol[1] : 0.0,
                  v_sol.size() > 2 ? v_sol[2] : 0.0,
                  v_sol.size() > 3 ? v_sol[3] : 0.0,
                  v_sol.size() > 4 ? v_sol[4] : 0.0,
                  v_sol.size() > 5 ? v_sol[5] : 0.0);
      RCLCPP_INFO(get_logger(), "  w_seq: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  w_sol[0],
                  w_sol.size() > 1 ? w_sol[1] : 0.0,
                  w_sol.size() > 2 ? w_sol[2] : 0.0,
                  w_sol.size() > 3 ? w_sol[3] : 0.0,
                  w_sol.size() > 4 ? w_sol[4] : 0.0,
                  w_sol.size() > 5 ? w_sol[5] : 0.0);

      // Simulate trajectory and show predicted states
      double sim_x = robot.x, sim_y = robot.y, sim_yaw = robot.yaw;
      RCLCPP_INFO(get_logger(), "  Predicted trajectory:");
      for (size_t k = 0; k < std::min(v_sol.size(), size_t(6)); ++k) {
        sim_x += v_sol[k] * std::cos(sim_yaw) * dt_;
        sim_y += v_sol[k] * std::sin(sim_yaw) * dt_;
        sim_yaw += w_sol[k] * dt_;
        double dist_at_k = std::hypot(sim_x - goal_x_, sim_y - goal_y_);
        RCLCPP_INFO(get_logger(), "    Step %zu: (%.2f, %.2f, %.1f°) dist=%.2fm",
                    k+1, sim_x, sim_y, sim_yaw * 180.0 / M_PI, dist_at_k);
      }

      debug_counter++;
    }
  }

  void publishZeroVelocity()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    // Reset previous command for smooth restart
    prev_v_cmd_ = 0.0;
    prev_w_cmd_ = 0.0;
  }

  /**
   * @brief Update current waypoint index based on robot position
   * Uses lookahead mechanism to select waypoint ahead of robot
   * Must be called with data_mutex_ locked
   */
  void updateCurrentWaypoint(double robot_x, double robot_y)
  {
    if (!global_path_ || global_path_->poses.empty())
    {
      return;
    }

    // Minimum distance before advancing to next waypoint
    // Increased to prevent rapid switching between close waypoints
    // Simple waypoint switching logic (matching VLM node)
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

  // Parameters
  double goal_x_, goal_y_, goal_yaw_, goal_tolerance_;
  double dt_;
  int N_;
  double control_rate_hz_;
  double default_v_max_, omega_max_, v_min_;
  double v_accel_max_, omega_accel_max_;  // Acceleration limits
  double w_goal_, w_smooth_, w_obstacle_;
  double w_terminal_orientation_, w_running_orientation_;
  double orientation_activation_distance_;
  double rotation_penalty_distance_threshold_;
  bool enable_vlm_;  // Enable VLM parameter adjustments
  double w_vlm_directional_, w_vlm_action_, w_vlm_scene_, w_vlm_personal_;
  double min_obstacle_distance_, min_valid_laser_range_;
  double hard_min_obstacle_distance_;  // Hard safety constraint - robot CANNOT get closer than this
  double min_person_safety_distance_;  // Minimum safe distance to people

  // Previous control commands for acceleration constraints
  double prev_v_cmd_{0.0};
  double prev_w_cmd_{0.0};

  // VLM cost modifier values (converted from VLM string parameters)
  double vlm_side_preference_value_{0.0};      // -1.0 (left), 0.0 (neutral), 1.0 (right)
  double vlm_action_speed_penalty_value_{0.0}; // 0.0 (go), 0.5 (slow), 1.0 (stop)
  double vlm_scene_caution_value_{0.0};        // 0.0 (open), 0.5 (corridor), 1.0 (doorway)
  double adaptive_min_obstacle_distance_value_{0.3}; // VLM-adaptive soft threshold (0.25-0.5m)

  // Safety verification tracking
  uint32_t safety_checks_passed_{0};
  uint32_t safety_violations_{0};

  // Safe fallback command
  double safe_fallback_v_{0.0};
  double safe_fallback_w_{0.0};
  bool has_safe_fallback_command_{false};

  // Previous solution for warm starting
  std::vector<double> prev_v_sol_;
  std::vector<double> prev_w_sol_;
  bool has_previous_solution_{false};

  std::string cmd_vel_topic_, scan_topic_, crowd_topic_, global_path_topic_;
  std::string map_frame_, base_link_frame_;
  std::string log_directory_;
  bool log_mpc_to_csv_;
  bool goal_reached_{false};
  bool goal_received_{false};  // Flag to prevent navigation until goal is set via RViz2
  bool vlm_ready_{false};      // Flag to prevent navigation until VLM is warmed up (only if VLM enabled)

  // CasADi optimization
  Opti opti_;
  MX v_var_, w_var_;
  MX x0_param_, y0_param_, yaw0_param_;
  MX goal_x_param_, goal_y_param_, goal_yaw_param_;
  MX v_max_param_, w_goal_param_, w_social_param_;
  MX prev_v_param_, prev_w_param_;  // Previous controls for acceleration penalty
  MX w_terminal_orientation_param_, w_running_orientation_param_;
  MX orientation_activation_distance_param_, rotation_threshold_param_;
  MX vlm_side_preference_param_, vlm_action_speed_penalty_param_, vlm_scene_caution_param_;  // VLM cost modifiers
  MX adaptive_min_obstacle_distance_param_;  // VLM-adaptive soft obstacle distance
  MX cost_expr_;  // Cost expression for extraction after solve

  // Parameters for obstacles and people (fixed-size arrays)
  MX obstacles_x_param_, obstacles_y_param_;
  MX people_x_param_, people_y_param_;
  int max_obstacles_;
  int max_people_;

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<person_tracker::msg::PersonInfoArray>::SharedPtr people_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<social_mpc_nav::msg::VLMParameters>::SharedPtr vlm_params_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Latest data
  double current_robot_x_{0.0};
  double current_robot_y_{0.0};
  double current_robot_yaw_{0.0};
  bool pose_received_{false};
  social_mpc_nav::msg::People2D::SharedPtr latest_people_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Path::SharedPtr global_path_;
  bool path_received_{false};
  size_t current_waypoint_index_{0};
  std::mutex data_mutex_;
  social_mpc_nav::msg::VLMParameters::SharedPtr latest_vlm_params_;
  std::mutex vlm_params_mutex_;

  // Social contract
  std::unique_ptr<social_mpc_nav::SocialContractHelper> contract_helper_;

  // Logging
  std::ofstream log_stream_;
};

// Signal handler for graceful shutdown (defined after class)
void signalHandler(int signum)
{
  if (g_node_ptr) {
    RCLCPP_INFO(g_node_ptr->get_logger(), "Interrupt signal (%d) received. Stopping robot...", signum);
    g_node_ptr->emergencyStop();
  }
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPCControllerCasADiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
