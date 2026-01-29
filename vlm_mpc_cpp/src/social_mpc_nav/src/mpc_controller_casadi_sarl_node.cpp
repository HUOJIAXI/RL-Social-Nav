/**
 * @file mpc_controller_casadi_sarl_node.cpp
 * @brief CasADi-based MPC controller with SARL attention-weighted social costs and VLM integration
 *
 * Extends mpc_controller_casadi_node with SARL (Socially Attentive RL) integration:
 * - Replaces uniform 1/distance social cost with attention-weighted per-person cost
 * - Scene-dependent multipliers from VLM condition SARL cost weights
 * - Publishes NavigationExplanation merging VLM + SARL interpretability
 * - Terminal V(s) via linear approximation (finite-difference gradient from batch service)
 * - SARL recommended action as soft reference velocity
 *
 * Hard constraints (collision avoidance, velocity limits) remain via IPOPT.
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
#include <sstream>

#include <casadi/casadi.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "person_tracker/msg/person_info_array.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"
#include "social_mpc_nav/msg/vlm_parameters.hpp"
#include "social_mpc_nav/msg/sarl_output.hpp"
#include "social_mpc_nav/msg/navigation_explanation.hpp"
#include "social_mpc_nav/social_contract.hpp"
#include "social_mpc_nav/mpc_vlm_helpers.hpp"
#include "social_mpc_nav/sarl_helpers.hpp"
#include "social_mpc_nav/srv/evaluate_sarl_batch.hpp"

#include <future>

using namespace casadi;

// Forward declaration for signal handler
class MPCControllerCasADiSARLNode;
static MPCControllerCasADiSARLNode* g_node_ptr = nullptr;
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
 * @brief CasADi-based MPC controller with SARL attention-weighted social costs
 */
class MPCControllerCasADiSARLNode : public rclcpp::Node
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

  MPCControllerCasADiSARLNode()
  : Node("mpc_controller_casadi_sarl_node")
  {
    // Parameters
    goal_x_ = declare_parameter<double>("goal_x", 10.0);
    goal_y_ = declare_parameter<double>("goal_y", 5.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 1.0);

    dt_ = declare_parameter<double>("dt", 0.3);
    N_ = declare_parameter<int>("N", 6);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);

    default_v_max_ = declare_parameter<double>("default_v_max", 0.6);
    omega_max_ = declare_parameter<double>("omega_max", 0.9);
    v_min_ = declare_parameter<double>("v_min", 0.0);

    // Acceleration limits for smooth control
    v_accel_max_ = declare_parameter<double>("v_accel_max", 0.5);
    omega_accel_max_ = declare_parameter<double>("omega_accel_max", 1.5);

    w_goal_ = declare_parameter<double>("w_goal", 20.0);
    w_smooth_ = declare_parameter<double>("w_smooth", 0.1);
    w_obstacle_ = declare_parameter<double>("w_obstacle", 1.0);

    // Orientation control parameters
    w_terminal_orientation_ = declare_parameter<double>("w_terminal_orientation", 6.0);
    w_running_orientation_ = declare_parameter<double>("w_running_orientation", 3.0);
    orientation_activation_distance_ = declare_parameter<double>("orientation_activation_distance", 2.0);
    rotation_penalty_distance_threshold_ = declare_parameter<double>("rotation_penalty_distance_threshold", 1.5);

    // VLM integration
    enable_vlm_ = declare_parameter<bool>("enable_vlm", false);

    // VLM cost weights
    w_vlm_directional_ = declare_parameter<double>("w_vlm_directional", 1.0);
    w_vlm_action_ = declare_parameter<double>("w_vlm_action", 2.0);
    w_vlm_scene_ = declare_parameter<double>("w_vlm_scene", 1.5);
    w_vlm_personal_ = declare_parameter<double>("w_vlm_personal", 5.0);

    // SARL integration parameters
    w_sarl_attention_ = declare_parameter<double>("w_sarl_attention", 1.0);
    sarl_staleness_sec_ = declare_parameter<double>("sarl_staleness_sec", 0.5);
    sarl_output_topic_ = declare_parameter<std::string>("sarl_output_topic", "/sarl/output");

    // Enhancement A: Terminal V(s) linearization
    w_sarl_terminal_ = declare_parameter<double>("w_sarl_terminal", 2.0);
    sarl_terminal_delta_ = declare_parameter<double>("sarl_terminal_delta", 0.3);
    sarl_service_name_ = declare_parameter<std::string>("sarl_service_name", "/sarl/evaluate_batch");

    // Enhancement C: SARL recommended action reference
    w_sarl_ref_speed_ = declare_parameter<double>("w_sarl_ref_speed", 0.5);
    w_sarl_ref_heading_ = declare_parameter<double>("w_sarl_ref_heading", 1.0);
    sarl_ref_horizon_ = declare_parameter<int>("sarl_ref_horizon", 3);
    sarl_ref_decay_rate_ = declare_parameter<double>("sarl_ref_decay_rate", 0.5);

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
    explanation_pub_ = create_publisher<social_mpc_nav::msg::NavigationExplanation>(
      "/navigation/explanation", rclcpp::QoS(10));
    sarl_mpc_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/sarl/mpc_markers", rclcpp::QoS(10));

    people_sub_ = create_subscription<person_tracker::msg::PersonInfoArray>(
      crowd_topic_, 10,
      std::bind(&MPCControllerCasADiSARLNode::onPersonInfo, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MPCControllerCasADiSARLNode::onScan, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, rclcpp::QoS(10).transient_local(),
      std::bind(&MPCControllerCasADiSARLNode::onPath, this, std::placeholders::_1));

    vlm_params_sub_ = create_subscription<social_mpc_nav::msg::VLMParameters>(
      "/vlm/mpc_parameters", 10,
      std::bind(&MPCControllerCasADiSARLNode::onVLMParameters, this, std::placeholders::_1));

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&MPCControllerCasADiSARLNode::onGoalPose, this, std::placeholders::_1));

    // SARL subscription
    sarl_sub_ = create_subscription<social_mpc_nav::msg::SARLOutput>(
      sarl_output_topic_, rclcpp::QoS(10),
      std::bind(&MPCControllerCasADiSARLNode::onSARLOutput, this, std::placeholders::_1));

    // SARL batch evaluation service client (Enhancement A)
    sarl_client_ = create_client<social_mpc_nav::srv::EvaluateSARLBatch>(sarl_service_name_);

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Robot pose update timer (high frequency for smooth updates)
    pose_update_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz
      std::bind(&MPCControllerCasADiSARLNode::updateRobotPose, this));

    // Control timer
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MPCControllerCasADiSARLNode::controlLoop, this));

    // Setup logging
    if (log_mpc_to_csv_) {
      setupLogging();
    }

    // Register signal handler for graceful shutdown
    g_node_ptr = this;
    std::signal(SIGINT, signalHandler);

    RCLCPP_INFO(get_logger(), "CasADi+SARL MPC controller started. Goal: (%.2f, %.2f) dt=%.2f N=%d",
                goal_x_, goal_y_, dt_, N_);
    RCLCPP_INFO(get_logger(), "Using CasADi optimization with hard safety constraints + SARL attention");
    RCLCPP_INFO(get_logger(), "SARL integration: ENABLED (attention-weighted social cost, w_sarl_attention=%.2f)",
                w_sarl_attention_);
    RCLCPP_INFO(get_logger(), "SARL topic: %s (staleness threshold: %.1fs)",
                sarl_output_topic_.c_str(), sarl_staleness_sec_);
    RCLCPP_INFO(get_logger(), "SARL terminal V(s): w=%.2f, delta=%.2fm, service=%s",
                w_sarl_terminal_, sarl_terminal_delta_, sarl_service_name_.c_str());
    RCLCPP_INFO(get_logger(), "SARL action ref: w_speed=%.2f, w_heading=%.2f, horizon=%d, decay=%.2f",
                w_sarl_ref_speed_, w_sarl_ref_heading_, sarl_ref_horizon_, sarl_ref_decay_rate_);
    RCLCPP_INFO(get_logger(), "VLM integration: %s", enable_vlm_ ? "ENABLED" : "DISABLED");

    // VLM warmup requirement
    if (enable_vlm_) {
      vlm_ready_ = false;
      RCLCPP_INFO(get_logger(), "VLM warmup required - waiting for first valid VLM response...");
    } else {
      vlm_ready_ = true;
    }

    RCLCPP_INFO(get_logger(), "Signal handler registered - robot will stop immediately on Ctrl+C");
    RCLCPP_INFO(get_logger(), "Post-optimization safety verification ENABLED");
    RCLCPP_INFO(get_logger(), "   - Obstacle safety distance: %.2fm", min_obstacle_distance_);
    RCLCPP_INFO(get_logger(), "   - Personal space distance: %.2fm", min_person_safety_distance_);
    RCLCPP_INFO(get_logger(), "   - Fallback strategy: Last safe command or emergency stop");
  }

private:
  void setupCasADiProblem()
  {
    opti_ = Opti();

    // Create decision variables
    v_var_ = opti_.variable(N_);
    w_var_ = opti_.variable(N_);

    // Create parameters (updated each iteration)
    x0_param_ = opti_.parameter();
    y0_param_ = opti_.parameter();
    yaw0_param_ = opti_.parameter();

    goal_x_param_ = opti_.parameter();
    goal_y_param_ = opti_.parameter();
    goal_yaw_param_ = opti_.parameter();

    v_max_param_ = opti_.parameter();
    w_goal_param_ = opti_.parameter();
    w_social_param_ = opti_.parameter();

    prev_v_param_ = opti_.parameter();
    prev_w_param_ = opti_.parameter();

    // Orientation control parameters
    w_terminal_orientation_param_ = opti_.parameter();
    w_running_orientation_param_ = opti_.parameter();
    orientation_activation_distance_param_ = opti_.parameter();
    rotation_threshold_param_ = opti_.parameter();

    // VLM cost parameters
    vlm_side_preference_param_ = opti_.parameter();
    vlm_action_speed_penalty_param_ = opti_.parameter();
    vlm_scene_caution_param_ = opti_.parameter();
    adaptive_min_obstacle_distance_param_ = opti_.parameter();

    // Fixed-size arrays for obstacles and people
    max_obstacles_ = 5;
    max_people_ = 5;

    obstacles_x_param_ = opti_.parameter(max_obstacles_);
    obstacles_y_param_ = opti_.parameter(max_obstacles_);

    people_x_param_ = opti_.parameter(max_people_);
    people_y_param_ = opti_.parameter(max_people_);

    // SARL parameters: per-person attention weights and scene multiplier
    attn_weights_param_ = opti_.parameter(max_people_);
    scene_mult_param_ = opti_.parameter();

    // Enhancement A: Terminal V(s) linearization parameters
    terminal_v_grad_x_param_ = opti_.parameter();
    terminal_v_grad_y_param_ = opti_.parameter();
    terminal_ref_x_param_ = opti_.parameter();
    terminal_ref_y_param_ = opti_.parameter();
    terminal_v_base_param_ = opti_.parameter();
    scene_mult_terminal_param_ = opti_.parameter();
    w_sarl_terminal_param_ = opti_.parameter();

    // Enhancement C: SARL action reference parameters
    sarl_ref_speed_param_ = opti_.parameter();
    sarl_ref_heading_param_ = opti_.parameter();
    w_sarl_ref_speed_param_ = opti_.parameter();
    w_sarl_ref_heading_param_ = opti_.parameter();

    // ===== BUILD OPTIMIZATION PROBLEM (ONCE!) =====

    MX x = x0_param_;
    MX y = y0_param_;
    MX yaw = yaw0_param_;

    MX cost = 0;

    for (int k = 0; k < N_; ++k) {
      MX x_prev = x;
      MX y_prev = y;

      // Dynamics (unicycle model)
      x = x + v_var_(k) * cos(yaw) * dt_;
      y = y + v_var_(k) * sin(yaw) * dt_;
      yaw = yaw + w_var_(k) * dt_;

      // Progress reward
      MX dx_to_goal_before = x_prev - goal_x_param_;
      MX dy_to_goal_before = y_prev - goal_y_param_;
      MX dist_before_sq = dx_to_goal_before * dx_to_goal_before + dy_to_goal_before * dy_to_goal_before;

      MX dx_to_goal_after = x - goal_x_param_;
      MX dy_to_goal_after = y - goal_y_param_;
      MX dist_after_sq = dx_to_goal_after * dx_to_goal_after + dy_to_goal_after * dy_to_goal_after;

      MX progress = dist_before_sq - dist_after_sq;
      cost = cost - 15.0 * progress;

      // Forward motion incentive
      cost = cost - 2.0 * v_var_(k);

      // Zero velocity penalty
      MX zero_velocity_penalty = 3.0 / (v_var_(k) + 0.1);
      cost = cost + zero_velocity_penalty;

      // Rotation penalty
      MX rotation_penalty = w_var_(k) * w_var_(k) / (v_var_(k) + 0.1);
      cost = cost + 0.8 * rotation_penalty;

      // Distance to goal (for orientation guidance)
      MX dist_to_goal = sqrt(dx_to_goal_after * dx_to_goal_after + dy_to_goal_after * dy_to_goal_after + 0.01);

      // Orientation guidance (disabled — same as base CasADi node)

      // Smoothness cost
      if (k > 0) {
        cost = cost + w_smooth_ * (pow(v_var_(k) - v_var_(k-1), 2) +
                                    pow(w_var_(k) - w_var_(k-1), 2));
      }

      // ===== Enhancement C: SARL ACTION REFERENCE COST =====
      // Apply to first K steps with exponential decay
      if (k < sarl_ref_horizon_) {
        double decay = std::exp(-k * sarl_ref_decay_rate_);
        // Speed reference: penalize deviation from SARL's recommended speed
        MX speed_error = v_var_(k) - sarl_ref_speed_param_;
        cost = cost + w_sarl_ref_speed_param_ * decay * speed_error * speed_error;
        // Heading reference: (1 - cos) is smooth, wrapping-free, zero when aligned
        MX heading_error_cost = 1.0 - cos(yaw - sarl_ref_heading_param_);
        cost = cost + w_sarl_ref_heading_param_ * decay * heading_error_cost;
      }

      // ===== SARL ATTENTION-WEIGHTED SOCIAL COST =====
      // Replaces uniform 1/distance with: w_social * N * attn_weight_i * scene_mult * (1/dist)
      // attn_weights_param_ and scene_mult_param_ are set before each solve
      for (int i = 0; i < max_people_; ++i) {
        MX dx_person = x - people_x_param_(i);
        MX dy_person = y - people_y_param_(i);
        MX dist_sq = dx_person * dx_person + dy_person * dy_person + 0.01;
        MX dist = sqrt(dist_sq);

        // Attention-weighted cost: w_social * N_people * attention_i * scene_mult * (1/dist)
        // N_people scaling preserves total cost magnitude when using normalized attention weights
        cost = cost + w_social_param_
                    * static_cast<double>(max_people_) * attn_weights_param_(i)
                    * scene_mult_param_
                    * (1.0 / (dist + 0.1));
      }

      // Obstacle cost (soft penalty + hard constraint)
      for (int i = 0; i < max_obstacles_; ++i) {
        MX dx_obs = x - obstacles_x_param_(i);
        MX dy_obs = y - obstacles_y_param_(i);
        MX dist_sq_obs = dx_obs * dx_obs + dy_obs * dy_obs + 0.01;
        MX dist_obs = sqrt(dist_sq_obs);

        // HARD SAFETY CONSTRAINT
        opti_.subject_to(dist_obs >= hard_min_obstacle_distance_);

        // Adaptive soft barrier function (VLM-modulated)
        MX safety_margin = adaptive_min_obstacle_distance_param_ * 1.2;
        MX barrier_penalty = if_else(
          dist_obs < safety_margin,
          50.0 * exp(-dist_obs + adaptive_min_obstacle_distance_param_),
          0.0
        );
        cost = cost + barrier_penalty;

        // Inverse distance penalty for nearby obstacles (within 1m)
        MX obstacle_influence_range = 1.0;
        MX nearby_penalty = if_else(
          dist_obs < obstacle_influence_range,
          w_obstacle_ * (1.0 / (dist_obs + 0.1)),
          0.0
        );
        cost = cost + nearby_penalty;
      }

      // ===== VLM-BASED COSTS =====

      // 1. Directional preference cost
      MX dx_to_goal_current = goal_x_param_ - x;
      MX dy_to_goal_current = goal_y_param_ - y;
      MX path_direction = atan2(dy_to_goal_current, dx_to_goal_current);
      MX lateral_offset = (y - y0_param_) * cos(path_direction) - (x - x0_param_) * sin(path_direction);
      MX directional_cost = -w_vlm_directional_ * vlm_side_preference_param_ * lateral_offset;
      cost = cost + directional_cost;

      // 2. Action-based speed penalty
      MX action_cost = w_vlm_action_ * vlm_action_speed_penalty_param_ * v_var_(k);
      cost = cost + action_cost;

      // 3. Scene-specific caution
      MX cautious_speed = 0.3;
      MX speed_excess = v_var_(k) - cautious_speed;
      MX scene_cost = w_vlm_scene_ * vlm_scene_caution_param_ *
                      if_else(speed_excess > 0.0, speed_excess * speed_excess, 0.0);
      cost = cost + scene_cost;
    }

    // Terminal goal cost
    MX dx_final = x - goal_x_param_;
    MX dy_final = y - goal_y_param_;
    cost = cost + w_goal_param_ * (dx_final * dx_final + dy_final * dy_final);

    // ===== Enhancement A: Terminal V(s) cost via linear approximation =====
    // V(s_terminal) ≈ V_base + dV/dx * (x_N - x_ref) + dV/dy * (y_N - y_ref)
    // Higher V(s) = better social state, so negate for minimization
    MX terminal_v_linear = terminal_v_base_param_
        + terminal_v_grad_x_param_ * (x - terminal_ref_x_param_)
        + terminal_v_grad_y_param_ * (y - terminal_ref_y_param_);
    cost = cost - w_sarl_terminal_param_ * scene_mult_terminal_param_ * terminal_v_linear;

    // Soft acceleration penalties
    double w_accel = 0.5;
    cost = cost + w_accel * pow(v_var_(0) - prev_v_param_, 2);
    cost = cost + w_accel * pow(w_var_(0) - prev_w_param_, 2);

    // Set objective
    opti_.minimize(cost);
    cost_expr_ = cost;

    // Hard constraints: Velocity limits
    opti_.subject_to(v_var_ >= v_min_);
    opti_.subject_to(v_var_ <= v_max_param_);
    opti_.subject_to(w_var_ >= -omega_max_);
    opti_.subject_to(w_var_ <= omega_max_);

    // Solver options - IPOPT
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.sb"] = "yes";
    opts["error_on_fail"] = false;

    opts["ipopt.max_iter"] = 150;
    opts["ipopt.tol"] = 1e-2;
    opts["ipopt.acceptable_tol"] = 5e-1;
    opts["ipopt.acceptable_iter"] = 5;
    opts["ipopt.acceptable_obj_change_tol"] = 1e-1;

    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.warm_start_bound_push"] = 1e-6;
    opts["ipopt.warm_start_mult_bound_push"] = 1e-6;
    opts["ipopt.mu_init"] = 1e-2;

    opts["ipopt.mu_strategy"] = "adaptive";
    opts["ipopt.adaptive_mu_globalization"] = "kkt-error";

    opts["ipopt.acceptable_constr_viol_tol"] = 1e-1;
    opts["ipopt.acceptable_dual_inf_tol"] = 1e10;
    opts["ipopt.acceptable_compl_inf_tol"] = 1e-1;

    opts["ipopt.linear_solver"] = "mumps";
    opts["ipopt.hessian_approximation"] = "limited-memory";

    opti_.solver("ipopt", opts);

    RCLCPP_INFO(get_logger(),
      "CasADi+SARL problem setup complete (IPOPT, max_obstacles=%d, max_people=%d, attention-weighted social cost)",
      max_obstacles_, max_people_);
  }

  void setupLogging()
  {
    std::string log_dir = log_directory_;
    if (!log_dir.empty() && log_dir[0] == '~') {
      const char* home = getenv("HOME");
      if (home) {
        log_dir = std::string(home) + log_dir.substr(1);
      }
    }

    std::filesystem::create_directories(log_dir);

    std::string log_file = log_dir + "/mpc_casadi_sarl_log.csv";
    log_stream_.open(log_file, std::ios::out);

    if (log_stream_.good()) {
      log_stream_ << "timestamp,x,y,yaw,v_cmd,w_cmd,goal_dist,solve_time_ms,cost,"
                  << "safety_verified,min_obs_dist,min_person_dist,"
                  << "total_checks,total_violations,violation_rate,"
                  << "sarl_active,scene_mult,max_attention,"
                  << "terminal_v_valid,terminal_v_base,terminal_grad_mag,"
                  << "sarl_ref_valid,sarl_ref_speed,sarl_ref_heading\n";
      RCLCPP_INFO(get_logger(), "CasADi+SARL MPC logging to: %s", log_file.c_str());
    }
  }

  void updateRobotPose()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(map_frame_, base_link_frame_, rclcpp::Time(0));

      current_robot_x_ = transform_stamped.transform.translation.x;
      current_robot_y_ = transform_stamped.transform.translation.y;
      current_robot_yaw_ = tf2::getYaw(transform_stamped.transform.rotation);
      pose_received_ = true;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Could not get robot pose from TF: %s", ex.what());
      pose_received_ = false;
    }
  }

  void onPersonInfo(const person_tracker::msg::PersonInfoArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

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
    current_waypoint_index_ = 0;

    if (msg && !msg->poses.empty())
    {
      RCLCPP_INFO(get_logger(),
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

    if (enable_vlm_ && !vlm_ready_ && msg->source == "vlm") {
      vlm_ready_ = true;
      RCLCPP_INFO(get_logger(),
        "VLM WARMED UP! First valid VLM response (confidence: %.2f, scene: %s)",
        msg->confidence, msg->scene_type.c_str());
    }
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    geometry_msgs::msg::PoseStamped goal_in_map;
    try {
      if (msg->header.frame_id.empty() || msg->header.frame_id == map_frame_) {
        goal_in_map = *msg;
      } else {
        tf_buffer_->transform(*msg, goal_in_map, map_frame_, tf2::durationFromSec(0.1));
      }

      goal_x_ = goal_in_map.pose.position.x;
      goal_y_ = goal_in_map.pose.position.y;
      goal_yaw_ = tf2::getYaw(goal_in_map.pose.orientation);
      goal_reached_ = false;
      goal_received_ = true;

      RCLCPP_INFO(get_logger(),
                  "New goal received: (%.2f, %.2f, yaw: %.2f deg) in %s frame",
                  goal_x_, goal_y_, goal_yaw_ * 180.0 / M_PI, map_frame_.c_str());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "Failed to transform goal pose: %s", ex.what());
    }
  }

  void onSARLOutput(const social_mpc_nav::msg::SARLOutput::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sarl_mutex_);
    latest_sarl_ = msg;
  }

  /**
   * @brief Publish SARL MPC visualization markers to /sarl/mpc_markers
   *
   * Visualizes:
   * - Terminal V(s) gradient arrow at predicted terminal state (purple)
   * - Predicted terminal state sphere (magenta)
   * - SARL reference heading arrow at robot (cyan)
   * - MPC actual command arrow at robot (green)
   * - Integration info text
   */
  void publishSARLMPCMarkers(const geometry_msgs::msg::Twist & cmd)
  {
    visualization_msgs::msg::MarkerArray markers;
    auto stamp = now();
    int id = 0;

    double rx = current_robot_x_;
    double ry = current_robot_y_;
    double ryaw = current_robot_yaw_;

    // --- Marker 1: SARL reference heading arrow (cyan) ---
    if (sarl_ref_valid_log_) {
      visualization_msgs::msg::Marker ref_arrow;
      ref_arrow.header.frame_id = map_frame_;
      ref_arrow.header.stamp = stamp;
      ref_arrow.ns = "sarl_ref_heading";
      ref_arrow.id = id++;
      ref_arrow.type = visualization_msgs::msg::Marker::ARROW;
      ref_arrow.action = visualization_msgs::msg::Marker::ADD;

      double arrow_len = std::min(sarl_ref_speed_log_ * 2.0, 2.0);
      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = rx;
      p_start.y = ry;
      p_start.z = 0.3;
      p_end.x = rx + arrow_len * std::cos(sarl_ref_heading_log_);
      p_end.y = ry + arrow_len * std::sin(sarl_ref_heading_log_);
      p_end.z = 0.3;
      ref_arrow.points.push_back(p_start);
      ref_arrow.points.push_back(p_end);

      ref_arrow.scale.x = 0.06;  // shaft diameter
      ref_arrow.scale.y = 0.12;  // head diameter
      ref_arrow.scale.z = 0.12;  // head length
      ref_arrow.color.r = 0.0;
      ref_arrow.color.g = 0.8;
      ref_arrow.color.b = 1.0;
      ref_arrow.color.a = 0.8;
      ref_arrow.lifetime.sec = 0;
      ref_arrow.lifetime.nanosec = 200000000;
      markers.markers.push_back(ref_arrow);
    }

    // --- Marker 2: MPC actual command arrow (green) ---
    {
      double cmd_speed = cmd.linear.x;
      if (cmd_speed > 0.02) {
        visualization_msgs::msg::Marker cmd_arrow;
        cmd_arrow.header.frame_id = map_frame_;
        cmd_arrow.header.stamp = stamp;
        cmd_arrow.ns = "mpc_cmd_arrow";
        cmd_arrow.id = id++;
        cmd_arrow.type = visualization_msgs::msg::Marker::ARROW;
        cmd_arrow.action = visualization_msgs::msg::Marker::ADD;

        double arrow_len = std::min(cmd_speed * 2.0, 2.0);
        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = rx;
        p_start.y = ry;
        p_start.z = 0.4;
        p_end.x = rx + arrow_len * std::cos(ryaw);
        p_end.y = ry + arrow_len * std::sin(ryaw);
        p_end.z = 0.4;
        cmd_arrow.points.push_back(p_start);
        cmd_arrow.points.push_back(p_end);

        cmd_arrow.scale.x = 0.06;
        cmd_arrow.scale.y = 0.12;
        cmd_arrow.scale.z = 0.12;
        cmd_arrow.color.r = 0.2;
        cmd_arrow.color.g = 1.0;
        cmd_arrow.color.b = 0.2;
        cmd_arrow.color.a = 0.8;
        cmd_arrow.lifetime.sec = 0;
        cmd_arrow.lifetime.nanosec = 200000000;
        markers.markers.push_back(cmd_arrow);
      }
    }

    // --- Marker 3: Terminal state sphere (magenta) ---
    if (terminal_v_valid_log_ && has_previous_solution_ && !prev_v_sol_.empty()) {
      // Propagate to terminal state
      double tx = rx, ty = ry, tyaw = ryaw;
      for (size_t k = 0; k < prev_v_sol_.size(); ++k) {
        tx += prev_v_sol_[k] * std::cos(tyaw) * dt_;
        ty += prev_v_sol_[k] * std::sin(tyaw) * dt_;
        tyaw += prev_w_sol_[k] * dt_;
      }

      visualization_msgs::msg::Marker term_sphere;
      term_sphere.header.frame_id = map_frame_;
      term_sphere.header.stamp = stamp;
      term_sphere.ns = "terminal_state";
      term_sphere.id = id++;
      term_sphere.type = visualization_msgs::msg::Marker::SPHERE;
      term_sphere.action = visualization_msgs::msg::Marker::ADD;
      term_sphere.pose.position.x = tx;
      term_sphere.pose.position.y = ty;
      term_sphere.pose.position.z = 0.3;
      term_sphere.pose.orientation.w = 1.0;
      term_sphere.scale.x = 0.25;
      term_sphere.scale.y = 0.25;
      term_sphere.scale.z = 0.25;
      // Magenta
      term_sphere.color.r = 0.9;
      term_sphere.color.g = 0.2;
      term_sphere.color.b = 0.9;
      term_sphere.color.a = 0.7;
      term_sphere.lifetime.sec = 0;
      term_sphere.lifetime.nanosec = 200000000;
      markers.markers.push_back(term_sphere);

      // --- Marker 4: Terminal V(s) gradient arrow (purple) ---
      double grad_mag = std::hypot(terminal_grad_x_log_, terminal_grad_y_log_);
      if (grad_mag > 0.01) {
        visualization_msgs::msg::Marker grad_arrow;
        grad_arrow.header.frame_id = map_frame_;
        grad_arrow.header.stamp = stamp;
        grad_arrow.ns = "terminal_v_gradient";
        grad_arrow.id = id++;
        grad_arrow.type = visualization_msgs::msg::Marker::ARROW;
        grad_arrow.action = visualization_msgs::msg::Marker::ADD;

        // Normalize and scale for visibility
        double scale = std::min(grad_mag * 2.0, 1.5);
        double gx_n = terminal_grad_x_log_ / grad_mag;
        double gy_n = terminal_grad_y_log_ / grad_mag;

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = tx;
        p_start.y = ty;
        p_start.z = 0.5;
        p_end.x = tx + gx_n * scale;
        p_end.y = ty + gy_n * scale;
        p_end.z = 0.5;
        grad_arrow.points.push_back(p_start);
        grad_arrow.points.push_back(p_end);

        grad_arrow.scale.x = 0.05;
        grad_arrow.scale.y = 0.10;
        grad_arrow.scale.z = 0.10;
        // Purple
        grad_arrow.color.r = 0.7;
        grad_arrow.color.g = 0.0;
        grad_arrow.color.b = 1.0;
        grad_arrow.color.a = 0.9;
        grad_arrow.lifetime.sec = 0;
        grad_arrow.lifetime.nanosec = 200000000;
        markers.markers.push_back(grad_arrow);
      }

      // --- Marker 5: Terminal V(s) info text ---
      visualization_msgs::msg::Marker term_text;
      term_text.header.frame_id = map_frame_;
      term_text.header.stamp = stamp;
      term_text.ns = "terminal_v_info";
      term_text.id = id++;
      term_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      term_text.action = visualization_msgs::msg::Marker::ADD;
      term_text.pose.position.x = tx;
      term_text.pose.position.y = ty;
      term_text.pose.position.z = 0.8;
      term_text.scale.z = 0.18;
      term_text.color.r = 0.9;
      term_text.color.g = 0.6;
      term_text.color.b = 1.0;
      term_text.color.a = 1.0;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "V(s_N)=" << terminal_v_base_log_
          << " |grad|=" << grad_mag;
      term_text.text = oss.str();
      term_text.lifetime.sec = 0;
      term_text.lifetime.nanosec = 200000000;
      markers.markers.push_back(term_text);
    }

    sarl_mpc_marker_pub_->publish(markers);
  }

  /**
   * @brief Publish NavigationExplanation merging VLM + SARL interpretability
   */
  void publishNavigationExplanation(const social_mpc_nav::SocialContract & contract)
  {
    auto explanation = social_mpc_nav::msg::NavigationExplanation();
    explanation.stamp = builtin_interfaces::msg::Time(now());

    // VLM fields
    {
      std::lock_guard<std::mutex> lock(vlm_params_mutex_);
      if (latest_vlm_params_) {
        explanation.scene_type = latest_vlm_params_->scene_type;
        explanation.crowd_density = latest_vlm_params_->crowd_density;
        explanation.recommended_action = latest_vlm_params_->recommended_action;
        explanation.vlm_confidence = latest_vlm_params_->confidence;
      } else {
        explanation.scene_type = "unknown";
        explanation.crowd_density = "unknown";
        explanation.recommended_action = "go_ahead";
        explanation.vlm_confidence = 0.0f;
      }
    }

    // SARL fields
    social_mpc_nav::msg::SARLOutput::SharedPtr sarl_data;
    {
      std::lock_guard<std::mutex> lock(sarl_mutex_);
      sarl_data = latest_sarl_;
    }

    bool sarl_valid = sarl_data && sarl_data->is_valid &&
        (now() - rclcpp::Time(sarl_data->stamp)).seconds() < sarl_staleness_sec_;

    if (sarl_valid) {
      explanation.person_names = sarl_data->person_names;
      explanation.attention_weights = sarl_data->attention_weights;
      explanation.state_value = sarl_data->state_value;
    } else {
      explanation.state_value = 0.0f;
    }

    // Person distances and motions
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (latest_people_ && !latest_people_->people.empty()) {
        for (const auto & person : latest_people_->people) {
          float dist = std::hypot(
            current_robot_x_ - static_cast<double>(person.x),
            current_robot_y_ - static_cast<double>(person.y));
          explanation.person_distances.push_back(dist);

          float speed = std::hypot(person.vx, person.vy);
          if (speed < 0.1f) {
            explanation.person_motions.push_back("stationary");
          } else if (speed < 0.5f) {
            explanation.person_motions.push_back("slow");
          } else {
            explanation.person_motions.push_back("walking");
          }
        }
      }
    }

    // Applied control parameters
    explanation.applied_v_max = static_cast<float>(contract.v_max);
    explanation.applied_w_social = static_cast<float>(contract.w_social);

    // Scene multiplier
    social_mpc_nav::sarl_helpers::SceneMultiplier scene_mult{1.0, 0.5};
    {
      std::lock_guard<std::mutex> lock(vlm_params_mutex_);
      if (latest_vlm_params_) {
        scene_mult = social_mpc_nav::sarl_helpers::getSceneMultiplier(
          latest_vlm_params_->scene_type, latest_vlm_params_->crowd_density);
      }
    }
    explanation.sarl_scene_multiplier = static_cast<float>(scene_mult.attention_mult);

    // Human-readable explanation
    std::ostringstream oss;
    oss << "Scene: " << explanation.scene_type
        << " (" << explanation.crowd_density << ")";
    if (sarl_valid) {
      oss << " | SARL active: V(s)=" << explanation.state_value
          << ", attention_mult=" << scene_mult.attention_mult;
      if (!explanation.attention_weights.empty()) {
        size_t max_idx = 0;
        float max_w = 0.0f;
        for (size_t i = 0; i < explanation.attention_weights.size(); ++i) {
          if (explanation.attention_weights[i] > max_w) {
            max_w = explanation.attention_weights[i];
            max_idx = i;
          }
        }
        if (max_idx < explanation.person_names.size()) {
          oss << ", focus: person " << explanation.person_names[max_idx]
              << " (w=" << max_w << ")";
        }
      }
    } else {
      oss << " | SARL: inactive (fallback to uniform social cost)";
    }
    // Enhancement A: Terminal V(s) info
    if (terminal_v_valid_log_) {
      oss << " | TermV: base=" << terminal_v_base_log_
          << ", grad=(" << terminal_grad_x_log_ << "," << terminal_grad_y_log_ << ")";
    }

    // Enhancement C: SARL action reference info
    if (sarl_ref_valid_log_) {
      oss << " | SARLref: spd=" << sarl_ref_speed_log_
          << ", hdg=" << sarl_ref_heading_log_;
    }

    oss << " | Action: " << explanation.recommended_action
        << " | v_max=" << contract.v_max
        << " | Solver: CasADi/IPOPT";

    explanation.explanation_text = oss.str();

    explanation_pub_->publish(explanation);
  }

  void controlLoop()
  {
    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for robot pose from TF...");
      return;
    }

    if (!goal_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for goal from RViz2 (use 2D Goal Pose tool)...");
      return;
    }

    if (enable_vlm_ && !vlm_ready_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for VLM warmup...");
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
        updateCurrentWaypoint(robot.x, robot.y);

        if (current_waypoint_index_ < global_path_->poses.size())
        {
          const auto& wp = global_path_->poses[current_waypoint_index_];
          target_x = wp.pose.position.x;
          target_y = wp.pose.position.y;
          tracking_waypoint = true;

          const double dist_to_wp = std::hypot(robot.x - target_x, robot.y - target_y);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Tracking waypoint %zu/%zu at (%.2f, %.2f), distance: %.2fm",
            current_waypoint_index_, global_path_->poses.size()-1,
            target_x, target_y, dist_to_wp);
        }
      }
    }

    // Check if goal/waypoint is reached
    const double dist_to_target = std::hypot(robot.x - target_x, robot.y - target_y);
    const double dist_to_final_goal = std::hypot(robot.x - goal_x_, robot.y - goal_y_);

    double yaw_error = goal_yaw_ - robot.yaw;
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

    const double orientation_tolerance = 0.175;  // ~10 degrees

    if (dist_to_final_goal < goal_tolerance_) {
      const double singularity_threshold = 0.087;
      if (std::abs(yaw_error) > (M_PI - singularity_threshold)) {
        if (!goal_reached_) {
          RCLCPP_WARN(get_logger(),
            "Orientation %.1f deg apart - skipping alignment to avoid oscillation",
            std::abs(yaw_error) * 180.0 / M_PI);
          RCLCPP_INFO(get_logger(), "GOAL POSITION REACHED! Orientation alignment skipped.");
          goal_reached_ = true;
        }
        publishZeroVelocity();
        return;
      }

      if (std::abs(yaw_error) > orientation_tolerance) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        double k_rot = 0.5;
        double max_rotation_speed = 0.2;
        double desired_omega = k_rot * yaw_error;
        double omega_normalized = desired_omega / max_rotation_speed;
        cmd.angular.z = max_rotation_speed * std::tanh(omega_normalized);
        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Position reached, aligning orientation: %.1f deg remaining",
          std::abs(yaw_error) * 180.0 / M_PI);
        return;
      }

      if (!goal_reached_) {
        RCLCPP_INFO(get_logger(),
          "GOAL FULLY REACHED! Position: %.3fm, Orientation: %.1f deg",
          dist_to_final_goal, std::abs(yaw_error) * 180.0 / M_PI);
        goal_reached_ = true;
      }
      publishZeroVelocity();
      return;
    }

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
    if (tracking_waypoint) {
      const double dist_wp = std::hypot(robot.x - target_x, robot.y - target_y);
      const double slowdown_distance = 1.0;
      const double min_approach_velocity = 0.15;

      if (dist_wp < slowdown_distance) {
        const double velocity_scale = std::max(0.0, dist_wp / slowdown_distance);
        contract.v_max = min_approach_velocity +
                         (contract.v_max - min_approach_velocity) * velocity_scale;
      }
    }

    // VLM modulation of social contract
    if (enable_vlm_) {
      std::lock_guard<std::mutex> lock(vlm_params_mutex_);
      if (latest_vlm_params_) {
        bool is_fallback = (latest_vlm_params_->source != "vlm");

        if (!is_fallback) {
          contract.v_max *= latest_vlm_params_->speed_scale;
          contract.w_social *= (latest_vlm_params_->min_personal_distance / 1.0);
          if (latest_vlm_params_->need_to_wait) {
            contract.v_max = 0.0;
            contract.w_social = 10.0;
          }

          // Side preference
          vlm_side_preference_value_ = 0.0;
          if (latest_vlm_params_->side_preference == "left") {
            vlm_side_preference_value_ = -1.0;
          } else if (latest_vlm_params_->side_preference == "right") {
            vlm_side_preference_value_ = 1.0;
          }

          // Action speed penalty
          vlm_action_speed_penalty_value_ = 0.0;
          const std::string& action = latest_vlm_params_->recommended_action;
          if (action == "stop_and_wait") {
            vlm_action_speed_penalty_value_ = 1.0;
          } else if (action == "slow_down_and_go") {
            vlm_action_speed_penalty_value_ = 0.5;
          } else if (action == "yield_to_pedestrian") {
            vlm_action_speed_penalty_value_ = 0.7;
          } else if (action == "keep_left" || action == "keep_right") {
            vlm_action_speed_penalty_value_ = 0.2;
          }

          // Scene caution
          vlm_scene_caution_value_ = 0.0;
          const std::string& scene = latest_vlm_params_->scene_type;
          if (scene == "doorway" || scene == "crossing" || scene == "queue") {
            vlm_scene_caution_value_ = 1.0;
          } else if (scene == "corridor" || scene == "lobby") {
            vlm_scene_caution_value_ = 0.5;
          }

          // Adaptive obstacle distance
          adaptive_min_obstacle_distance_value_ = latest_vlm_params_->adaptive_min_obstacle_distance;
          adaptive_min_obstacle_distance_value_ = std::max(
            adaptive_min_obstacle_distance_value_,
            hard_min_obstacle_distance_ + 0.05);
        }
      }
    }

    // VLM fallback
    if (!enable_vlm_ || !latest_vlm_params_ ||
        (latest_vlm_params_ && latest_vlm_params_->source != "vlm")) {
      vlm_side_preference_value_ = 0.0;
      vlm_action_speed_penalty_value_ = 0.0;
      vlm_scene_caution_value_ = 0.0;
      adaptive_min_obstacle_distance_value_ = min_obstacle_distance_;
    }

    // Extract obstacles from laser scan
    std::vector<ObstaclePoint> obstacles = extractObstacles(robot);

    // Temporarily override goal for MPC to track waypoint
    const double original_goal_x = goal_x_;
    const double original_goal_y = goal_y_;

    static double prev_target_x = target_x;
    static double prev_target_y = target_y;
    const double target_switch_dist = std::hypot(target_x - prev_target_x, target_y - prev_target_y);

    if (target_switch_dist > 1.0) {
      has_previous_solution_ = false;
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
      safety_verified_log = (cost != std::numeric_limits<double>::infinity());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "CasADi solver failed: %s", e.what());
      publishZeroVelocity();
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

    // Publish navigation explanation
    publishNavigationExplanation(contract);

    // Publish SARL MPC visualization markers
    publishSARLMPCMarkers(cmd);

    // Store previous command
    prev_v_cmd_ = cmd.linear.x;
    prev_w_cmd_ = cmd.angular.z;

    // Safety metrics for logging
    uint32_t total_safety_checks = safety_checks_passed_ + safety_violations_;
    double violation_rate = total_safety_checks > 0 ?
                           (static_cast<double>(safety_violations_) / total_safety_checks) : 0.0;

    // SARL metrics for logging
    float max_attn = 0.0f;
    {
      std::lock_guard<std::mutex> lock(sarl_mutex_);
      if (latest_sarl_ && !latest_sarl_->attention_weights.empty()) {
        for (auto w : latest_sarl_->attention_weights) {
          max_attn = std::max(max_attn, w);
        }
      }
    }

    // CSV Logging
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
                  << violation_rate << ","
                  << (current_sarl_active_ ? 1 : 0) << ","
                  << current_scene_mult_ << ","
                  << max_attn << ","
                  << (terminal_v_valid_log_ ? 1 : 0) << ","
                  << terminal_v_base_log_ << ","
                  << std::hypot(terminal_grad_x_log_, terminal_grad_y_log_) << ","
                  << (sarl_ref_valid_log_ ? 1 : 0) << ","
                  << sarl_ref_speed_log_ << ","
                  << sarl_ref_heading_log_ << "\n";
    }

    // Periodic safety statistics
    static int safety_log_counter = 0;
    if (++safety_log_counter % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "Safety Stats: %u checks, %u violations (%.2f%%) | SARL: %s | Scene mult: %.2f",
        total_safety_checks, safety_violations_, violation_rate * 100.0,
        current_sarl_active_ ? "ACTIVE" : "inactive",
        current_scene_mult_);
    }

    // Log navigation status
    if (tracking_waypoint) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "CasADi+SARL MPC: WP %zu/%zu (%.2fm) | Goal: %.2fm | v=%.2f w=%.2f | %.1fms | SARL:%s",
        current_waypoint_index_ + 1,
        global_path_ ? global_path_->poses.size() : 0,
        dist_to_target, dist_to_final_goal,
        cmd.linear.x, cmd.angular.z, solve_time_ms,
        current_sarl_active_ ? "ON" : "off");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "CasADi+SARL MPC: Goal %.2fm | v=%.2f w=%.2f | %.1fms | Cost: %.2f | SARL:%s",
        dist_to_final_goal, cmd.linear.x, cmd.angular.z,
        solve_time_ms, cost,
        current_sarl_active_ ? "ON" : "off");
    }
  }

  bool getRobotStateInMap(RobotState& robot)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for robot pose from TF...");
      return false;
    }

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

    const double forward_cone_angle = M_PI / 2.0;
    const int max_obstacles = 20;

    std::vector<std::pair<double, ObstaclePoint>> obstacle_distances;

    for (size_t i = 0; i < scan->ranges.size(); i += 2) {
      const float range = scan->ranges[i];

      if (std::isnan(range) || std::isinf(range) ||
          range < min_valid_laser_range_ || range > scan->range_max) {
        continue;
      }

      const double ray_angle = scan->angle_min + i * scan->angle_increment;

      if (std::abs(ray_angle) > forward_cone_angle / 2.0) {
        continue;
      }

      const double local_x = range * std::cos(ray_angle);
      const double local_y = range * std::sin(ray_angle);

      const double global_x = robot.x + local_x * std::cos(robot.yaw) - local_y * std::sin(robot.yaw);
      const double global_y = robot.y + local_x * std::sin(robot.yaw) + local_y * std::cos(robot.yaw);

      double dist = std::hypot(global_x - robot.x, global_y - robot.y);
      obstacle_distances.push_back({dist, {global_x, global_y}});
    }

    std::sort(obstacle_distances.begin(), obstacle_distances.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (size_t i = 0; i < std::min(obstacle_distances.size(), static_cast<size_t>(max_obstacles)); ++i) {
      obstacles.push_back(obstacle_distances[i].second);
    }

    return obstacles;
  }

  /**
   * @brief Compute linearized V(s) gradient at a reference point via finite differences.
   *
   * Calls EvaluateSARLBatch with 5 states: center + 4 perturbations (+-dx, +-dy).
   * Returns gradient and base value for CasADi terminal cost.
   */
  bool computeTerminalVGradient(
    double ref_x, double ref_y, double ref_yaw, double ref_v,
    const social_mpc_nav::msg::People2D::SharedPtr& people,
    double& grad_x, double& grad_y, double& v_base)
  {
    grad_x = 0.0;
    grad_y = 0.0;
    v_base = 0.0;

    if (!sarl_client_ || !sarl_client_->service_is_ready()) {
      return false;
    }
    if (!people || people->people.empty()) {
      return false;
    }

    const double delta = sarl_terminal_delta_;

    auto request = std::make_shared<social_mpc_nav::srv::EvaluateSARLBatch::Request>();
    request->num_rollouts = 5;
    request->goal_x = static_cast<float>(goal_x_);
    request->goal_y = static_cast<float>(goal_y_);

    // Body-frame velocity for unicycle: vx_body = v, vy_body = 0
    float vx_body = static_cast<float>(ref_v);
    float vy_body = 0.0f;

    // 5 states: center, +dx, -dx, +dy, -dy
    float fx = static_cast<float>(ref_x);
    float fy = static_cast<float>(ref_y);
    float fyaw = static_cast<float>(ref_yaw);
    float fd = static_cast<float>(delta);

    // State 0: center
    for (float val : {fx, fy, fyaw, vx_body, vy_body})
      request->robot_states.push_back(val);
    // State 1: +delta_x
    for (float val : {fx + fd, fy, fyaw, vx_body, vy_body})
      request->robot_states.push_back(val);
    // State 2: -delta_x
    for (float val : {fx - fd, fy, fyaw, vx_body, vy_body})
      request->robot_states.push_back(val);
    // State 3: +delta_y
    for (float val : {fx, fy + fd, fyaw, vx_body, vy_body})
      request->robot_states.push_back(val);
    // State 4: -delta_y
    for (float val : {fx, fy - fd, fyaw, vx_body, vy_body})
      request->robot_states.push_back(val);

    // Pack people states
    request->num_people = static_cast<int32_t>(people->people.size());
    for (const auto& person : people->people) {
      request->people_states.push_back(person.x);
      request->people_states.push_back(person.y);
      request->people_states.push_back(person.vx);
      request->people_states.push_back(person.vy);
    }

    // Synchronous call with short timeout
    auto future = sarl_client_->async_send_request(request);
    auto status = future.wait_for(std::chrono::milliseconds(10));

    if (status != std::future_status::ready) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
        "SARL terminal gradient: service timed out");
      return false;
    }

    auto response = future.get();
    if (!response->success || response->values.size() != 5) {
      return false;
    }

    double v_center = static_cast<double>(response->values[0]);
    double v_plus_x = static_cast<double>(response->values[1]);
    double v_minus_x = static_cast<double>(response->values[2]);
    double v_plus_y = static_cast<double>(response->values[3]);
    double v_minus_y = static_cast<double>(response->values[4]);

    // Central finite difference
    grad_x = (v_plus_x - v_minus_x) / (2.0 * delta);
    grad_y = (v_plus_y - v_minus_y) / (2.0 * delta);
    v_base = v_center;

    return true;
  }

  bool verifySolutionSafety(
    const RobotState& robot,
    const std::vector<double>& v_sol,
    const std::vector<double>& w_sol,
    const std::vector<ObstaclePoint>& obstacles,
    const social_mpc_nav::msg::People2D::SharedPtr& people,
    double& min_obs_dist,
    double& min_person_dist)
  {
    min_obs_dist = std::numeric_limits<double>::infinity();
    min_person_dist = std::numeric_limits<double>::infinity();

    RobotState pred = robot;
    bool is_safe = true;

    for (size_t k = 0; k < v_sol.size(); ++k) {
      pred.x += v_sol[k] * std::cos(pred.yaw) * dt_;
      pred.y += v_sol[k] * std::sin(pred.yaw) * dt_;
      pred.yaw += w_sol[k] * dt_;

      for (const auto& obs : obstacles) {
        double dist = std::hypot(pred.x - obs.x, pred.y - obs.y);
        min_obs_dist = std::min(min_obs_dist, dist);
        if (dist < min_obstacle_distance_) {
          is_safe = false;
        }
      }

      if (people && !people->people.empty()) {
        for (const auto& person : people->people) {
          double dist = std::hypot(pred.x - person.x, pred.y - person.y);
          min_person_dist = std::min(min_person_dist, dist);
          if (dist < min_person_safety_distance_) {
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
    // Debug first call
    static bool first_call = true;
    if (first_call) {
      RCLCPP_INFO(get_logger(), "=== CasADi+SARL MPC First Call ===");
      RCLCPP_INFO(get_logger(), "Robot: (%.2f, %.2f, yaw=%.1f deg)",
                  robot.x, robot.y, robot.yaw * 180.0 / M_PI);
      RCLCPP_INFO(get_logger(), "Goal:  (%.2f, %.2f)", goal_x_, goal_y_);
      RCLCPP_INFO(get_logger(), "Obstacles: %zu", obstacles.size());
      first_call = false;
    }

    // ===== SET PARAMETERS =====

    opti_.set_value(x0_param_, robot.x);
    opti_.set_value(y0_param_, robot.y);
    opti_.set_value(yaw0_param_, robot.yaw);

    opti_.set_value(goal_x_param_, goal_x_);
    opti_.set_value(goal_y_param_, goal_y_);
    opti_.set_value(goal_yaw_param_, goal_yaw_);

    opti_.set_value(v_max_param_, contract.v_max);
    opti_.set_value(w_goal_param_, contract.w_goal);
    opti_.set_value(w_social_param_, contract.w_social);

    opti_.set_value(w_terminal_orientation_param_, w_terminal_orientation_);
    opti_.set_value(w_running_orientation_param_, w_running_orientation_);
    opti_.set_value(orientation_activation_distance_param_, orientation_activation_distance_);
    opti_.set_value(rotation_threshold_param_, rotation_penalty_distance_threshold_);

    opti_.set_value(vlm_side_preference_param_, vlm_side_preference_value_);
    opti_.set_value(vlm_action_speed_penalty_param_, vlm_action_speed_penalty_value_);
    opti_.set_value(vlm_scene_caution_param_, vlm_scene_caution_value_);
    opti_.set_value(adaptive_min_obstacle_distance_param_, adaptive_min_obstacle_distance_value_);

    opti_.set_value(prev_v_param_, prev_v_cmd_);
    opti_.set_value(prev_w_param_, prev_w_cmd_);

    // Prepare obstacle parameters
    const double far_away = 1e6;
    std::vector<double> obs_x(max_obstacles_, far_away);
    std::vector<double> obs_y(max_obstacles_, far_away);
    size_t num_obs = std::min(obstacles.size(), static_cast<size_t>(max_obstacles_));
    for (size_t i = 0; i < num_obs; ++i) {
      obs_x[i] = obstacles[i].x;
      obs_y[i] = obstacles[i].y;
    }
    opti_.set_value(obstacles_x_param_, obs_x);
    opti_.set_value(obstacles_y_param_, obs_y);

    // Prepare people parameters
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

    // ===== SET SARL ATTENTION WEIGHTS AND SCENE MULTIPLIER =====

    social_mpc_nav::msg::SARLOutput::SharedPtr sarl_data;
    {
      std::lock_guard<std::mutex> lock(sarl_mutex_);
      sarl_data = latest_sarl_;
    }

    bool sarl_valid = sarl_data && sarl_data->is_valid &&
        (now() - rclcpp::Time(sarl_data->stamp)).seconds() < sarl_staleness_sec_;

    current_sarl_active_ = sarl_valid;

    // Compute scene multiplier from VLM
    social_mpc_nav::sarl_helpers::SceneMultiplier scene_mult{1.0, 0.5};
    {
      std::lock_guard<std::mutex> lock(vlm_params_mutex_);
      if (latest_vlm_params_) {
        scene_mult = social_mpc_nav::sarl_helpers::getSceneMultiplier(
          latest_vlm_params_->scene_type, latest_vlm_params_->crowd_density);
      }
    }
    current_scene_mult_ = scene_mult.attention_mult;

    opti_.set_value(scene_mult_param_, scene_mult.attention_mult);

    // Build attention weight array
    std::vector<double> attn_vector(max_people_, 0.0);
    if (sarl_valid && num_people > 0) {
      // Use SARL attention weights
      for (size_t i = 0; i < num_people; ++i) {
        if (i < sarl_data->attention_weights.size()) {
          attn_vector[i] = static_cast<double>(sarl_data->attention_weights[i]);
        } else {
          // Fallback to uniform for people without SARL weights
          attn_vector[i] = 1.0 / static_cast<double>(num_people);
        }
      }
      // Inactive slots remain 0.0 (these people are at 1e6,1e6 anyway)
    } else if (num_people > 0) {
      // SARL not available — uniform fallback
      double uniform_weight = 1.0 / static_cast<double>(num_people);
      for (size_t i = 0; i < num_people; ++i) {
        attn_vector[i] = uniform_weight;
      }
    }
    // If no people, all weights stay 0.0

    opti_.set_value(attn_weights_param_, attn_vector);

    // ===== Enhancement A: TERMINAL V(s) GRADIENT =====
    double terminal_grad_x = 0.0;
    double terminal_grad_y = 0.0;
    double terminal_v_base = 0.0;
    double terminal_ref_x = robot.x;
    double terminal_ref_y = robot.y;
    bool terminal_valid = false;

    if (sarl_valid && num_people > 0) {
      if (has_previous_solution_ && !prev_v_sol_.empty()) {
        // Propagate previous solution to get terminal position
        RobotState pred_terminal = robot;
        for (size_t k = 0; k < prev_v_sol_.size(); ++k) {
          pred_terminal.x += prev_v_sol_[k] * std::cos(pred_terminal.yaw) * dt_;
          pred_terminal.y += prev_v_sol_[k] * std::sin(pred_terminal.yaw) * dt_;
          pred_terminal.yaw += prev_w_sol_[k] * dt_;
        }
        terminal_ref_x = pred_terminal.x;
        terminal_ref_y = pred_terminal.y;

        double terminal_v = prev_v_sol_.back();
        terminal_valid = computeTerminalVGradient(
          terminal_ref_x, terminal_ref_y, pred_terminal.yaw, terminal_v,
          people, terminal_grad_x, terminal_grad_y, terminal_v_base);
      } else {
        // First iteration: use current robot state
        terminal_valid = computeTerminalVGradient(
          robot.x, robot.y, robot.yaw, prev_v_cmd_,
          people, terminal_grad_x, terminal_grad_y, terminal_v_base);
        terminal_ref_x = robot.x;
        terminal_ref_y = robot.y;
      }
    }

    // Set terminal V(s) CasADi parameters
    opti_.set_value(terminal_v_grad_x_param_, terminal_valid ? terminal_grad_x : 0.0);
    opti_.set_value(terminal_v_grad_y_param_, terminal_valid ? terminal_grad_y : 0.0);
    opti_.set_value(terminal_ref_x_param_, terminal_ref_x);
    opti_.set_value(terminal_ref_y_param_, terminal_ref_y);
    opti_.set_value(terminal_v_base_param_, terminal_valid ? terminal_v_base : 0.0);
    opti_.set_value(scene_mult_terminal_param_, terminal_valid ? scene_mult.terminal_mult : 0.0);
    opti_.set_value(w_sarl_terminal_param_, terminal_valid ? w_sarl_terminal_ : 0.0);

    // Store for logging
    terminal_v_valid_log_ = terminal_valid;
    terminal_v_base_log_ = terminal_v_base;
    terminal_grad_x_log_ = terminal_grad_x;
    terminal_grad_y_log_ = terminal_grad_y;

    // ===== Enhancement C: SARL ACTION REFERENCE =====
    double sarl_ref_speed = 0.0;
    double sarl_ref_heading = 0.0;
    bool sarl_ref_valid = false;

    if (sarl_valid && sarl_data->has_recommended_action) {
      sarl_ref_speed = static_cast<double>(sarl_data->recommended_speed);
      sarl_ref_heading = static_cast<double>(sarl_data->recommended_heading);
      sarl_ref_valid = (sarl_ref_speed > 0.01);
    }

    opti_.set_value(sarl_ref_speed_param_, sarl_ref_speed);
    opti_.set_value(sarl_ref_heading_param_, sarl_ref_heading);
    opti_.set_value(w_sarl_ref_speed_param_, sarl_ref_valid ? w_sarl_ref_speed_ : 0.0);
    opti_.set_value(w_sarl_ref_heading_param_, sarl_ref_valid ? w_sarl_ref_heading_ : 0.0);

    // Store for logging
    sarl_ref_valid_log_ = sarl_ref_valid;
    sarl_ref_speed_log_ = sarl_ref_speed;
    sarl_ref_heading_log_ = sarl_ref_heading;

    // ===== INITIAL GUESS =====

    std::vector<double> v_init(N_);
    std::vector<double> w_init(N_);

    bool is_stuck = (prev_v_cmd_ < 0.05);

    if (has_previous_solution_ && prev_v_sol_.size() >= static_cast<size_t>(N_)
        && prev_w_sol_.size() >= static_cast<size_t>(N_) && !is_stuck) {
      // Warm start
      for (int k = 0; k < N_ - 1; ++k) {
        v_init[k] = prev_v_sol_[k + 1];
        w_init[k] = prev_w_sol_[k + 1];
      }
      v_init[N_ - 1] = prev_v_sol_[N_ - 1];
      w_init[N_ - 1] = prev_w_sol_[N_ - 1];

      for (int k = 0; k < N_; ++k) {
        if (v_init[k] < 0.1) {
          v_init[k] = 0.2;
        }
      }
    } else {
      // Cold start
      double dx = goal_x_ - robot.x;
      double dy = goal_y_ - robot.y;
      double desired_yaw = std::atan2(dy, dx);
      double yaw_err = desired_yaw - robot.yaw;

      while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

      const double dist_to_goal = std::hypot(robot.x - goal_x_, robot.y - goal_y_);

      double max_speed_when_aligned = dist_to_goal < 1.0 ? 0.25 : 0.4;
      double max_speed_when_turning = dist_to_goal < 1.0 ? 0.15 : 0.2;

      double target_v = std::abs(yaw_err) < M_PI / 3.0 ?
                        std::min(max_speed_when_aligned, contract.v_max * 0.7) :
                        std::min(max_speed_when_turning, contract.v_max * 0.3);
      double target_w = yaw_err * 0.8;
      target_w = std::max(-omega_max_, std::min(omega_max_, target_w));

      if (dist_to_goal < 1.0) {
        target_v = std::min(target_v, 0.2);
      }
      if (target_v < 0.12) {
        target_v = 0.12;
      }

      for (int k = 0; k < N_; ++k) {
        v_init[k] = target_v;
        w_init[k] = target_w;
      }
    }

    opti_.set_initial(v_var_, v_init);
    opti_.set_initial(w_var_, w_init);

    // ===== SOLVE =====
    OptiSol sol = opti_.solve();

    std::vector<double> v_sol = sol.value(v_var_).get_elements();
    std::vector<double> w_sol = sol.value(w_var_).get_elements();

    // ===== POST-OPTIMIZATION SAFETY VERIFICATION =====
    double min_predicted_obstacle_dist = std::numeric_limits<double>::infinity();
    double min_predicted_person_dist = std::numeric_limits<double>::infinity();
    bool trajectory_is_safe = verifySolutionSafety(
      robot, v_sol, w_sol, obstacles, people,
      min_predicted_obstacle_dist, min_predicted_person_dist);

    if (!trajectory_is_safe) {
      safety_violations_++;

      RCLCPP_WARN(get_logger(),
        "SAFETY VERIFICATION FAILED! Min obs dist: %.3fm, Min person dist: %.3fm - Using fallback",
        min_predicted_obstacle_dist, min_predicted_person_dist);

      if (has_safe_fallback_command_) {
        cmd.linear.x = safe_fallback_v_;
        cmd.angular.z = safe_fallback_w_;
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_ERROR(get_logger(), "No safe fallback available - EMERGENCY STOP");
      }

      total_cost = std::numeric_limits<double>::infinity();

    } else {
      safety_checks_passed_++;

      prev_v_sol_ = v_sol;
      prev_w_sol_ = w_sol;
      has_previous_solution_ = true;

      cmd.linear.x = v_sol[0];
      cmd.angular.z = w_sol[0];

      safe_fallback_v_ = v_sol[0];
      safe_fallback_w_ = w_sol[0];
      has_safe_fallback_command_ = true;

      total_cost = static_cast<double>(sol.value(cost_expr_));
    }

    min_obs_dist_out = min_predicted_obstacle_dist;
    min_person_dist_out = min_predicted_person_dist;
  }

  void publishZeroVelocity()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    prev_v_cmd_ = 0.0;
    prev_w_cmd_ = 0.0;
  }

  void updateCurrentWaypoint(double robot_x, double robot_y)
  {
    if (!global_path_ || global_path_->poses.empty())
    {
      return;
    }

    const double lookahead_dist = 0.3;

    if (current_waypoint_index_ < global_path_->poses.size())
    {
      const auto& current_wp = global_path_->poses[current_waypoint_index_];
      const double dist = std::hypot(
        robot_x - current_wp.pose.position.x,
        robot_y - current_wp.pose.position.y);

      if (dist < lookahead_dist)
      {
        bool is_last_waypoint = (current_waypoint_index_ == global_path_->poses.size() - 1);
        const double dist_to_goal = std::hypot(robot_x - goal_x_, robot_y - goal_y_);

        if (is_last_waypoint && dist_to_goal > goal_tolerance_)
        {
          return;
        }

        current_waypoint_index_++;

        if (current_waypoint_index_ < global_path_->poses.size())
        {
          RCLCPP_INFO(get_logger(),
            "Waypoint %zu reached! Advancing to waypoint %zu/%zu",
            current_waypoint_index_,
            current_waypoint_index_ + 1,
            global_path_->poses.size());
        }
        else
        {
          RCLCPP_INFO(get_logger(), "All waypoints reached! Approaching final goal.");
        }
      }
    }
  }

  // ========== MEMBER VARIABLES ==========

  // Parameters
  double goal_x_, goal_y_, goal_yaw_, goal_tolerance_;
  double dt_;
  int N_;
  double control_rate_hz_;
  double default_v_max_, omega_max_, v_min_;
  double v_accel_max_, omega_accel_max_;
  double w_goal_, w_smooth_, w_obstacle_;
  double w_terminal_orientation_, w_running_orientation_;
  double orientation_activation_distance_;
  double rotation_penalty_distance_threshold_;
  bool enable_vlm_;
  double w_vlm_directional_, w_vlm_action_, w_vlm_scene_, w_vlm_personal_;
  double min_obstacle_distance_, min_valid_laser_range_;
  double hard_min_obstacle_distance_;
  double min_person_safety_distance_;

  // SARL parameters
  double w_sarl_attention_;
  double sarl_staleness_sec_;
  std::string sarl_output_topic_;

  // Enhancement A: Terminal V(s) parameters
  double w_sarl_terminal_;
  double sarl_terminal_delta_;
  std::string sarl_service_name_;

  // Enhancement C: SARL action reference parameters
  double w_sarl_ref_speed_;
  double w_sarl_ref_heading_;
  int sarl_ref_horizon_;
  double sarl_ref_decay_rate_;

  // Previous control commands
  double prev_v_cmd_{0.0};
  double prev_w_cmd_{0.0};

  // VLM cost modifier values
  double vlm_side_preference_value_{0.0};
  double vlm_action_speed_penalty_value_{0.0};
  double vlm_scene_caution_value_{0.0};
  double adaptive_min_obstacle_distance_value_{0.3};

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

  // SARL state tracking (for logging)
  bool current_sarl_active_{false};
  double current_scene_mult_{1.0};

  // Enhancement A/C logging state
  bool terminal_v_valid_log_{false};
  double terminal_v_base_log_{0.0};
  double terminal_grad_x_log_{0.0};
  double terminal_grad_y_log_{0.0};
  bool sarl_ref_valid_log_{false};
  double sarl_ref_speed_log_{0.0};
  double sarl_ref_heading_log_{0.0};

  std::string cmd_vel_topic_, scan_topic_, crowd_topic_, global_path_topic_;
  std::string map_frame_, base_link_frame_;
  std::string log_directory_;
  bool log_mpc_to_csv_;
  bool goal_reached_{false};
  bool goal_received_{false};
  bool vlm_ready_{false};

  // CasADi optimization
  Opti opti_;
  MX v_var_, w_var_;
  MX x0_param_, y0_param_, yaw0_param_;
  MX goal_x_param_, goal_y_param_, goal_yaw_param_;
  MX v_max_param_, w_goal_param_, w_social_param_;
  MX prev_v_param_, prev_w_param_;
  MX w_terminal_orientation_param_, w_running_orientation_param_;
  MX orientation_activation_distance_param_, rotation_threshold_param_;
  MX vlm_side_preference_param_, vlm_action_speed_penalty_param_, vlm_scene_caution_param_;
  MX adaptive_min_obstacle_distance_param_;
  MX cost_expr_;

  // CasADi parameters for obstacles and people
  MX obstacles_x_param_, obstacles_y_param_;
  MX people_x_param_, people_y_param_;
  int max_obstacles_;
  int max_people_;

  // SARL CasADi parameters
  MX attn_weights_param_;
  MX scene_mult_param_;

  // Enhancement A: Terminal V(s) CasADi parameters
  MX terminal_v_grad_x_param_, terminal_v_grad_y_param_;
  MX terminal_ref_x_param_, terminal_ref_y_param_;
  MX terminal_v_base_param_;
  MX scene_mult_terminal_param_;
  MX w_sarl_terminal_param_;

  // Enhancement C: SARL action reference CasADi parameters
  MX sarl_ref_speed_param_, sarl_ref_heading_param_;
  MX w_sarl_ref_speed_param_, w_sarl_ref_heading_param_;

  // ROS publishers/subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<social_mpc_nav::msg::NavigationExplanation>::SharedPtr explanation_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sarl_mpc_marker_pub_;
  rclcpp::Subscription<person_tracker::msg::PersonInfoArray>::SharedPtr people_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<social_mpc_nav::msg::VLMParameters>::SharedPtr vlm_params_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<social_mpc_nav::msg::SARLOutput>::SharedPtr sarl_sub_;
  rclcpp::Client<social_mpc_nav::srv::EvaluateSARLBatch>::SharedPtr sarl_client_;

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

  // SARL data
  social_mpc_nav::msg::SARLOutput::SharedPtr latest_sarl_;
  std::mutex sarl_mutex_;

  // Social contract
  std::unique_ptr<social_mpc_nav::SocialContractHelper> contract_helper_;

  // Logging
  std::ofstream log_stream_;
};

// Signal handler for graceful shutdown
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
  auto node = std::make_shared<MPCControllerCasADiSARLNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
