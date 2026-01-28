#include "social_mpc_nav/social_contract.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>

#include "rclcpp/clock.hpp"

namespace social_mpc_nav
{

namespace
{
double computeYawDot(const double yaw, const double dx, const double dy)
{
  const double heading_x = std::cos(yaw);
  const double heading_y = std::sin(yaw);
  return dx * heading_x + dy * heading_y;
}
}  // namespace

SocialContractHelper::SocialContractHelper(
  rclcpp::Logger logger,
  const std::string & log_directory,
  bool enable_csv_logging)
: logger_(logger), log_directory_(log_directory), log_to_csv_(enable_csv_logging)
{
  if (!log_directory_.empty())
  {
    std::error_code ec;
    std::filesystem::create_directories(log_directory_, ec);
    if (ec)
    {
      RCLCPP_WARN(
        logger_, "Failed to create log directory '%s': %s",
        log_directory_.c_str(), ec.message().c_str());
    }
  }
}

SocialContract SocialContractHelper::compute(
  double robot_x,
  double robot_y,
  double robot_yaw,
  double default_v_max,
  const social_mpc_nav::msg::People2D::SharedPtr & people_msg)
{
  SocialContract contract;
  contract.v_max = default_v_max;

  if (!people_msg || people_msg->people.empty())
  {
    // No people - use default goal-focused behavior
    contract.w_goal = 2.5;
    contract.w_social = 0.5;

    if (log_to_csv_)
    {
      writeCsvRow(clock_.now(), contract);
    }
    return contract;
  }

  double min_distance = std::numeric_limits<double>::infinity();
  bool person_in_front = false;
  int people_in_front_count = 0;

  for (const auto & person : people_msg->people)
  {
    const double dx = static_cast<double>(person.x) - robot_x;
    const double dy = static_cast<double>(person.y) - robot_y;
    const double dist = std::hypot(dx, dy);
    min_distance = std::min(min_distance, dist);

    // Check if person is in front (within 3m and in front half)
    if (computeYawDot(robot_yaw, dx, dy) > 0.0 && dist < 3.0)
    {
      person_in_front = true;
      people_in_front_count++;
    }
  }

  contract.min_person_distance = min_distance;
  contract.person_in_front = person_in_front;

  // LESS CONSERVATIVE VELOCITY LIMITS FOR CROWDED SCENARIOS
  // Allow robot to maintain reasonable speed even with people nearby
  if (min_distance < 0.5)
  {
    // Very close - slow down significantly but not stop
    contract.v_max = std::min(default_v_max, 0.3);
  }
  else if (person_in_front && people_in_front_count > 2)
  {
    // Multiple people in front - cautious but keep moving
    contract.v_max = std::min(default_v_max, 0.4);
  }
  else if (person_in_front)
  {
    // One person in front - reduce speed moderately
    contract.v_max = std::min(default_v_max, 0.5);
  }
  else
  {
    // No one in immediate path - use default
    contract.v_max = default_v_max;
  }

  // BALANCED WEIGHTS FOR CROWDED NAVIGATION
  // Prioritize goal more to avoid getting stuck
  if (min_distance < 0.6)
  {
    // Very close - high social avoidance but still pursue goal
    contract.w_social = 4.0;
    contract.w_goal = 1.5;  // Increased from 1.0
  }
  else if (min_distance < 1.5)
  {
    // Moderate distance - balanced approach
    contract.w_social = 2.0;
    contract.w_goal = 2.0;  // Equal weighting
  }
  else if (min_distance < 3.0)
  {
    // People nearby but not immediate threat
    contract.w_social = 1.0;
    contract.w_goal = 2.5;
  }
  else
  {
    // No people nearby - focus on goal
    contract.w_social = 0.5;
    contract.w_goal = 3.0;
  }

  const auto stamp = people_msg->stamp;
  const rclcpp::Time time_ros(stamp.sec, stamp.nanosec);

  if (log_to_csv_)
  {
    writeCsvRow(time_ros, contract);
  }

  RCLCPP_INFO(
    logger_, "Contract: v_max=%.2f w_goal=%.2f w_social=%.2f min_dist=%.2f people_front=%d",
    contract.v_max, contract.w_goal, contract.w_social, contract.min_person_distance,
    people_in_front_count);

  return contract;
}

void SocialContractHelper::ensureLogFile()
{
  if (!log_to_csv_ || contract_csv_.has_value())
  {
    return;
  }

  if (log_directory_.empty())
  {
    RCLCPP_WARN(logger_, "log_directory parameter is empty; contract CSV logging disabled.");
    log_to_csv_ = false;
    return;
  }

  std::filesystem::path file_path(log_directory_);
  file_path /= "social_contract.csv";

  contract_csv_.emplace(file_path, std::ios::out | std::ios::app);
  if (!contract_csv_->good())
  {
    RCLCPP_WARN(
      logger_, "Failed to open contract CSV file at '%s'; logging disabled.",
      file_path.string().c_str());
    contract_csv_.reset();
    log_to_csv_ = false;
    return;
  }

  if (contract_csv_->tellp() == 0)
  {
    (*contract_csv_) << "stamp_sec,v_max,w_goal,w_social,min_person_distance,person_in_front\n";
  }
}

void SocialContractHelper::writeCsvRow(const rclcpp::Time & stamp, const SocialContract & contract)
{
  std::lock_guard<std::mutex> lock(log_mutex_);
  ensureLogFile();
  if (!contract_csv_)
  {
    return;
  }

  (*contract_csv_) << std::fixed << std::setprecision(3)
                   << stamp.seconds() << ","
                   << contract.v_max << ","
                   << contract.w_goal << ","
                   << contract.w_social << ","
                   << contract.min_person_distance << ","
                   << static_cast<int>(contract.person_in_front)
                   << "\n";
  contract_csv_->flush();
}

}  // namespace social_mpc_nav
