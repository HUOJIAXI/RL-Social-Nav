#pragma once

#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "social_mpc_nav/msg/people2_d.hpp"

namespace social_mpc_nav
{

struct SocialContract
{
  double v_max{0.6};
  double w_goal{2.0};
  double w_social{1.0};
  double min_person_distance{std::numeric_limits<double>::infinity()};
  bool person_in_front{false};
};

/**
 * @brief Helper responsible for building and logging a simple social contract.
 *
 * This helper encodes the rule-based heuristics described in the prompt and provides a single
 * entry point to update the contract each MPC tick. Logging to CSV is handled internally so the
 * controller remains focused on optimization logic and ROS wiring.
 */
class SocialContractHelper
{
public:
  SocialContractHelper(
    rclcpp::Logger logger,
    const std::string & log_directory,
    bool enable_csv_logging);

  SocialContract compute(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double default_v_max,
    const social_mpc_nav::msg::People2D::SharedPtr & people_msg);

private:
  void ensureLogFile();
  void writeCsvRow(const rclcpp::Time & stamp, const SocialContract & contract);

  rclcpp::Logger logger_;
  std::string log_directory_;
  bool log_to_csv_{true};
  std::mutex log_mutex_;
  std::optional<std::ofstream> contract_csv_;
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

}  // namespace social_mpc_nav

