#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

/**
 * @brief Simple global path planner using A* on occupancy grid.
 *
 * This node:
 * 1. Subscribes to map (occupancy grid)
 * 2. Gets robot pose directly from TF tree (map→base_footprint)
 * 3. Plans a collision-free path from current position to goal
 * 4. Publishes the path as waypoints for MPC to track
 * 5. Replans when goal changes or obstacles detected
 *
 * The MPC controller then tracks this path while handling dynamic obstacles (people).
 */
class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node")
  {
    // Parameters
    goal_x_ = declare_parameter<double>("goal_x", 10.0);
    goal_y_ = declare_parameter<double>("goal_y", 5.0);
    waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 1.0);
    replan_distance_threshold_ = declare_parameter<double>("replan_distance_threshold", 0.5);
    planning_enabled_ = declare_parameter<bool>("planning_enabled", true);
    use_simple_line_planner_ = declare_parameter<bool>("use_simple_line_planner", true);
    occupancy_threshold_ = declare_parameter<int>("occupancy_threshold", 50);
    treat_unknown_as_free_ = declare_parameter<bool>("treat_unknown_as_free", true);
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.5);

    // Path smoothing parameters
    enable_path_smoothing_ = declare_parameter<bool>("enable_path_smoothing", true);
    smoothing_iterations_ = declare_parameter<int>("smoothing_iterations", 50);
    smoothing_weight_data_ = declare_parameter<double>("smoothing_weight_data", 0.5);
    smoothing_weight_smooth_ = declare_parameter<double>("smoothing_weight_smooth", 0.3);
    enable_shortcut_optimization_ = declare_parameter<bool>("enable_shortcut_optimization", true);

    // Declare topic parameters for robot-specific configuration
    std::string map_topic = declare_parameter<std::string>(
      "map_topic", "/task_generator_node/map");
    std::string global_path_topic = declare_parameter<std::string>(
      "global_path_topic", "/global_path");

    // Publishers
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      global_path_topic, rclcpp::QoS(10).transient_local());

    vis_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/global_path_marker", rclcpp::QoS(10));

    // Subscribers
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(10).transient_local(),
      std::bind(&GlobalPlannerNode::onMap, this, std::placeholders::_1));

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoalPose, this, std::placeholders::_1));

    // Initialize TF2 for coordinate frame transformations
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare coordinate frame parameters
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_footprint");

    // Robot pose update timer (high frequency for smooth updates)
    pose_update_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz
      std::bind(&GlobalPlannerNode::updateRobotPose, this));

    // Planning timer
    planning_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&GlobalPlannerNode::planningLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "Global planner started. Goal: (%.2f, %.2f) | Waypoint spacing: %.2fm\n"
      "  Planner: %s | Occupancy threshold: %d | Unknown as free: %s\n"
      "  Inflation radius: %.2fm | Smoothing: %s | Shortcut optimization: %s",
      goal_x_, goal_y_, waypoint_spacing_,
      use_simple_line_planner_ ? "straight line" : "A*",
      occupancy_threshold_,
      treat_unknown_as_free_ ? "yes" : "no",
      inflation_radius_,
      enable_path_smoothing_ ? "enabled" : "disabled",
      enable_shortcut_optimization_ ? "enabled" : "disabled");
  }

private:
  struct Point2D
  {
    double x;
    double y;

    bool operator==(const Point2D & other) const
    {
      return std::abs(x - other.x) < 1e-3 && std::abs(y - other.y) < 1e-3;
    }
  };

  // A* grid node structure
  struct GridNode
  {
    int x;        // Grid x index
    int y;        // Grid y index
    double g;     // Cost from start
    double h;     // Heuristic to goal
    double f;     // Total cost (g + h)
    int parent_x; // Parent grid x
    int parent_y; // Parent grid y

    GridNode(int x_, int y_, double g_, double h_, int px, int py)
    : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent_x(px), parent_y(py) {}

    // Comparison for priority queue (min heap based on f-cost)
    bool operator>(const GridNode & other) const
    {
      return f > other.f;
    }
  };

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

      current_x_ = transform_stamped.transform.translation.x;
      current_y_ = transform_stamped.transform.translation.y;
      pose_received_ = true;

      RCLCPP_DEBUG(get_logger(),
        "Robot position in map frame: (%.2f, %.2f)",
        current_x_, current_y_);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Could not get robot position from TF (map→%s): %s. Waiting for TF tree...",
        base_link_frame_.c_str(), ex.what());
      pose_received_ = false;
    }
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = msg;
    map_received_ = true;

    // Compute map statistics for debugging
    int occupied_count = 0;
    int free_count = 0;
    int unknown_count = 0;

    for (const auto & cell : msg->data)
    {
      if (cell < 0) unknown_count++;
      else if (cell >= 50) occupied_count++;
      else free_count++;
    }

    RCLCPP_INFO(get_logger(),
                "Map received: %dx%d, resolution: %.2fm, origin: (%.2f, %.2f)\n"
                "  Cells: %d free, %d occupied, %d unknown",
                msg->info.width, msg->info.height, msg->info.resolution,
                msg->info.origin.position.x, msg->info.origin.position.y,
                free_count, occupied_count, unknown_count);

    // Inflate obstacles for safer path planning
    inflateMap();
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

      // Enable planning now that we have a goal from RViz2
      goal_received_ = true;

      // Force immediate replanning by clearing the last planned path and setting replan flag
      last_planned_path_.clear();
      path_published_ = false;
      force_replan_ = true;  // Force replanning on next iteration regardless of other conditions

      RCLCPP_INFO(
        get_logger(),
        "🎯 New goal received from RViz2 for global planner: (%.2f, %.2f) in %s frame - Replanning path immediately",
        goal_x_, goal_y_, map_frame_.c_str());
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to transform goal pose from %s to %s: %s",
        msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    }
  }

  void planningLoop()
  {
    if (!planning_enabled_)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!pose_received_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for robot pose from TF...");
      return;
    }

    // Wait for goal from RViz2 before starting path planning
    if (!goal_received_)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "⏸️  Global planner waiting for goal from RViz2 (use 2D Goal Pose tool)...");
      return;
    }

    // Check if we need to replan
    const double dist_to_goal = std::hypot(current_x_ - goal_x_, current_y_ - goal_y_);

    // Stop planning when robot is at goal position (within goal tolerance)
    // This prevents replanning during orientation alignment phase
    const double goal_tolerance = 0.2;  // Slightly larger than MPC tolerance (0.15m)
    if (dist_to_goal < goal_tolerance && !force_replan_)
    {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
        "At goal position (%.2fm < %.2fm), stopping path planning to allow orientation alignment",
        dist_to_goal, goal_tolerance);
      return;
    }

    // Check if we need to replan based on path deviation (skip if forced replan)
    if (!force_replan_ && path_published_ && !last_planned_path_.empty())
    {
      double deviation = minDistanceToPath(current_x_, current_y_, last_planned_path_);

      if (deviation < replan_distance_threshold_)
      {
        // Robot is still on track, no need to replan
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
          "On track: deviation %.2fm < threshold %.2fm, no replanning needed",
          deviation, replan_distance_threshold_);
        return;
      }
      else
      {
        RCLCPP_INFO(get_logger(),
          "Path deviation detected: %.2fm > threshold %.2fm, replanning...",
          deviation, replan_distance_threshold_);
      }
    }

    // Log reason for replanning
    if (force_replan_)
    {
      RCLCPP_INFO(get_logger(), "🔄 Replanning due to new goal from RViz2");
      force_replan_ = false;  // Reset flag after triggering replan
    }

    // Plan path
    std::vector<Point2D> waypoints;

    if (use_simple_line_planner_ || !map_received_)
    {
      // Simple straight-line planner with intermediate waypoints
      waypoints = planStraightLine(current_x_, current_y_, goal_x_, goal_y_);

      // If straight-line planner detects obstacles and we have a map, try A* instead
      if (waypoints.empty() && map_received_)
      {
        RCLCPP_WARN(get_logger(),
                    "Straight-line planner failed (obstacles detected) - trying A* planner");
        waypoints = planAStar(current_x_, current_y_, goal_x_, goal_y_);
      }

      if (!map_received_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "No map available - using simple line planner without obstacle checking");
      }
    }
    else
    {
      // A* planner on occupancy grid
      waypoints = planAStar(current_x_, current_y_, goal_x_, goal_y_);

      // Fallback to straight line if A* fails (only if there's clear line-of-sight)
      if (waypoints.empty())
      {
        RCLCPP_WARN(get_logger(),
                    "A* planning failed - attempting straight line planner");
        waypoints = planStraightLine(current_x_, current_y_, goal_x_, goal_y_);

        if (waypoints.empty())
        {
          RCLCPP_ERROR(get_logger(),
                       "Both A* and straight-line planning failed - no safe path available");
        }
      }
    }

    if (waypoints.empty())
    {
      RCLCPP_WARN(get_logger(), "Failed to generate path!");
      return;
    }

    // Publish path
    publishPath(waypoints);
    publishVisualization(waypoints);

    // Save path for deviation checking
    last_planned_path_ = waypoints;
    path_published_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Path planned: %zu waypoints from (%.2f, %.2f) to (%.2f, %.2f)",
      waypoints.size(), current_x_, current_y_, goal_x_, goal_y_);
  }

  /**
   * @brief Convert world coordinates to grid indices
   * @param world_x World x coordinate (meters)
   * @param world_y World y coordinate (meters)
   * @param grid_x Output grid x index
   * @param grid_y Output grid y index
   * @return true if conversion successful and indices are within grid bounds
   */
  bool worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const
  {
    if (!map_)
    {
      return false;
    }

    // Convert world coordinates to grid indices using map origin and resolution
    // grid_index = floor((world_coord - origin_coord) / resolution)
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;

    grid_x = static_cast<int>(std::floor((world_x - origin_x) / resolution));
    grid_y = static_cast<int>(std::floor((world_y - origin_y) / resolution));

    // Check bounds
    if (grid_x < 0 || grid_x >= static_cast<int>(map_->info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(map_->info.height))
    {
      return false;
    }

    return true;
  }

  /**
   * @brief Convert grid indices to world coordinates (cell center)
   * @param grid_x Grid x index
   * @param grid_y Grid y index
   * @param world_x Output world x coordinate (meters)
   * @param world_y Output world y coordinate (meters)
   */
  void gridToWorld(int grid_x, int grid_y, double & world_x, double & world_y) const
  {
    // Convert grid indices back to world coordinates (use cell center)
    // world_coord = origin_coord + (grid_index + 0.5) * resolution
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;

    world_x = origin_x + (grid_x + 0.5) * resolution;
    world_y = origin_y + (grid_y + 0.5) * resolution;
  }

  /**
   * @brief Check if a grid cell is occupied
   * @param grid_x Grid x index
   * @param grid_y Grid y index
   * @param use_inflated Use inflated map if true, otherwise use original map
   * @return true if cell is occupied (occupancy >= threshold) or out of bounds
   */
  bool isOccupied(int grid_x, int grid_y, bool use_inflated = false) const
  {
    if (!map_)
    {
      return true; // Conservative: assume occupied if no map
    }

    // Check bounds
    if (grid_x < 0 || grid_x >= static_cast<int>(map_->info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(map_->info.height))
    {
      return true; // Out of bounds = occupied
    }

    // Get occupancy value from 1D array (row-major order)
    int index = grid_y * map_->info.width + grid_x;

    // Use inflated map if requested and available
    int8_t occupancy;
    if (use_inflated && !inflated_map_.empty())
    {
      occupancy = inflated_map_[index];
    }
    else
    {
      occupancy = map_->data[index];
    }

    // Occupancy interpretation:
    // -1 = unknown (configurable: treat as free or occupied)
    // 0 to threshold-1 = free
    // threshold to 100 = occupied
    if (occupancy < 0)
    {
      return !treat_unknown_as_free_; // Return true (occupied) if NOT treating unknown as free
    }

    return occupancy >= occupancy_threshold_;
  }

  /**
   * @brief Inflate obstacles in the occupancy grid by inflation_radius_
   * Creates an inflated map where obstacles are expanded for safer planning
   */
  void inflateMap()
  {
    if (!map_)
    {
      RCLCPP_WARN(get_logger(), "Cannot inflate map: No map available");
      return;
    }

    const int width = map_->info.width;
    const int height = map_->info.height;
    const double resolution = map_->info.resolution;

    // Calculate inflation radius in grid cells
    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution));

    if (inflation_cells == 0)
    {
      RCLCPP_INFO(get_logger(), "Inflation radius too small (%.2fm), no inflation applied",
                  inflation_radius_);
      inflated_map_.clear();
      return;
    }

    // Initialize inflated map as copy of original
    inflated_map_ = map_->data;

    // Count original occupied cells
    int original_occupied = 0;
    for (const auto & cell : map_->data)
    {
      if (cell >= occupancy_threshold_)
      {
        original_occupied++;
      }
    }

    // For each cell in the map
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        int index = y * width + x;

        // If this cell is occupied in the original map
        if (map_->data[index] >= occupancy_threshold_)
        {
          // Inflate by marking all cells within inflation_radius as occupied
          for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
          {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
            {
              int nx = x + dx;
              int ny = y + dy;

              // Check bounds
              if (nx < 0 || nx >= width || ny < 0 || ny >= height)
              {
                continue;
              }

              // Calculate actual distance (Euclidean)
              double dist = std::sqrt(dx * dx + dy * dy) * resolution;

              // If within inflation radius, mark as occupied
              if (dist <= inflation_radius_)
              {
                int nindex = ny * width + nx;

                // Only mark as occupied if it wasn't already occupied
                // Keep the original occupancy value if it was higher
                if (inflated_map_[nindex] < occupancy_threshold_)
                {
                  inflated_map_[nindex] = occupancy_threshold_;
                }
              }
            }
          }
        }
      }
    }

    // Count inflated occupied cells
    int inflated_occupied = 0;
    for (const auto & cell : inflated_map_)
    {
      if (cell >= occupancy_threshold_)
      {
        inflated_occupied++;
      }
    }

    RCLCPP_INFO(get_logger(),
                "Map inflation complete: radius %.2fm (%d cells)\n"
                "  Original occupied: %d cells | Inflated occupied: %d cells (%.1f%% increase)",
                inflation_radius_, inflation_cells, original_occupied, inflated_occupied,
                original_occupied > 0 ?
                  100.0 * (inflated_occupied - original_occupied) / original_occupied : 0.0);
  }

  /**
   * @brief Euclidean distance heuristic for A*
   */
  double euclideanDistance(int x1, int y1, int x2, int y2) const
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief Compute minimum distance from a point to a path
   * @param x Point x coordinate
   * @param y Point y coordinate
   * @param path Path to measure distance to
   * @return Minimum distance from point to any segment on the path
   */
  double minDistanceToPath(double x, double y, const std::vector<Point2D> & path) const
  {
    if (path.empty())
    {
      return std::numeric_limits<double>::max();
    }

    double min_dist = std::numeric_limits<double>::max();

    // Check distance to each path segment
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
      // Compute distance from point (x, y) to line segment [path[i], path[i+1]]
      double x1 = path[i].x;
      double y1 = path[i].y;
      double x2 = path[i + 1].x;
      double y2 = path[i + 1].y;

      // Vector from segment start to point
      double dx = x - x1;
      double dy = y - y1;

      // Segment direction vector
      double seg_dx = x2 - x1;
      double seg_dy = y2 - y1;
      double seg_length_sq = seg_dx * seg_dx + seg_dy * seg_dy;

      // Parametric position along segment (clamped to [0, 1])
      double t = 0.0;
      if (seg_length_sq > 1e-6)  // Avoid division by zero
      {
        t = std::max(0.0, std::min(1.0, (dx * seg_dx + dy * seg_dy) / seg_length_sq));
      }

      // Closest point on segment
      double closest_x = x1 + t * seg_dx;
      double closest_y = y1 + t * seg_dy;

      // Distance to closest point
      double dist = std::hypot(x - closest_x, y - closest_y);
      min_dist = std::min(min_dist, dist);
    }

    return min_dist;
  }

  /**
   * @brief Check if there's a clear line of sight between two world points
   * Uses Bresenham's line algorithm to check all grid cells along the line
   * @param x1 Start world x coordinate
   * @param y1 Start world y coordinate
   * @param x2 End world x coordinate
   * @param y2 End world y coordinate
   * @param use_inflated If true, check against inflated map for safer clearance
   * @return true if path is clear (no obstacles)
   */
  bool hasLineOfSight(double x1, double y1, double x2, double y2, bool use_inflated = false) const
  {
    if (!map_)
    {
      return false;
    }

    // Convert to grid coordinates
    int gx1, gy1, gx2, gy2;
    if (!worldToGrid(x1, y1, gx1, gy1) || !worldToGrid(x2, y2, gx2, gy2))
    {
      return false; // Out of bounds
    }

    // Bresenham's line algorithm to check all cells along the line
    int dx = std::abs(gx2 - gx1);
    int dy = std::abs(gy2 - gy1);
    int sx = (gx1 < gx2) ? 1 : -1;
    int sy = (gy1 < gy2) ? 1 : -1;
    int err = dx - dy;

    int x = gx1;
    int y = gy1;

    while (true)
    {
      // Check if current cell is occupied (use inflated map if requested for safety)
      if (isOccupied(x, y, use_inflated))
      {
        return false;
      }

      // Reached end point
      if (x == gx2 && y == gy2)
      {
        break;
      }

      // Bresenham step
      int e2 = 2 * err;
      if (e2 > -dy)
      {
        err -= dy;
        x += sx;
      }
      if (e2 < dx)
      {
        err += dx;
        y += sy;
      }
    }

    return true; // Clear path
  }

  /**
   * @brief Optimize path by removing unnecessary waypoints using line-of-sight
   * This creates shortcuts by connecting waypoints that have clear line of sight
   * @param path Input path to optimize
   * @return Optimized path with fewer waypoints
   */
  std::vector<Point2D> shortcutPath(const std::vector<Point2D> & path) const
  {
    if (path.size() <= 2)
    {
      return path; // Can't optimize paths with 2 or fewer points
    }

    std::vector<Point2D> optimized;
    optimized.push_back(path[0]); // Always keep start

    size_t current = 0;
    while (current < path.size() - 1)
    {
      // Try to connect to the farthest visible waypoint (use inflated map for safety)
      size_t farthest = current + 1;

      for (size_t i = path.size() - 1; i > current + 1; --i)
      {
        if (hasLineOfSight(path[current].x, path[current].y, path[i].x, path[i].y, true))
        {
          farthest = i;
          break;
        }
      }

      // Add the farthest visible waypoint
      optimized.push_back(path[farthest]);
      current = farthest;
    }

    RCLCPP_DEBUG(get_logger(), "Shortcut optimization: %zu -> %zu waypoints",
                 path.size(), optimized.size());

    return optimized;
  }

  /**
   * @brief Verify and densify path to ensure all consecutive segments are collision-free
   * @param path Input path to verify
   * @param original_path Original detailed path before resampling (for inserting intermediate points)
   * @return Path with additional waypoints inserted where segments cross obstacles
   */
  std::vector<Point2D> verifyAndDensifyPath(
    const std::vector<Point2D> & path,
    const std::vector<Point2D> & original_path) const
  {
    if (path.size() <= 1)
    {
      return path;
    }

    std::vector<Point2D> verified_path;
    verified_path.push_back(path[0]); // Always include start

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
      const Point2D& current = path[i];
      const Point2D& next = path[i + 1];

      // Check if direct line-of-sight exists between consecutive waypoints
      if (hasLineOfSight(current.x, current.y, next.x, next.y, true))
      {
        // Segment is safe, add next waypoint
        verified_path.push_back(next);
      }
      else
      {
        // No line-of-sight! Need to insert intermediate waypoints from original path
        // Find the sequence in original_path between current and next
        size_t start_idx = 0;
        size_t end_idx = 0;
        double min_dist_start = std::numeric_limits<double>::max();
        double min_dist_end = std::numeric_limits<double>::max();

        // Find closest points in original_path to current and next
        for (size_t j = 0; j < original_path.size(); ++j)
        {
          double dist_to_current = std::hypot(original_path[j].x - current.x,
                                              original_path[j].y - current.y);
          double dist_to_next = std::hypot(original_path[j].x - next.x,
                                           original_path[j].y - next.y);

          if (dist_to_current < min_dist_start)
          {
            min_dist_start = dist_to_current;
            start_idx = j;
          }
          if (dist_to_next < min_dist_end)
          {
            min_dist_end = dist_to_next;
            end_idx = j;
          }
        }

        // Insert all intermediate points from original_path
        if (start_idx < end_idx)
        {
          for (size_t j = start_idx + 1; j <= end_idx; ++j)
          {
            verified_path.push_back(original_path[j]);
          }
        }
        else
        {
          // Fallback: just add the next waypoint anyway (shouldn't happen if original path was valid)
          RCLCPP_WARN(get_logger(),
            "Path verification: No clear line-of-sight between waypoints at (%.2f, %.2f) and (%.2f, %.2f), "
            "but cannot find intermediate points. Adding anyway.",
            current.x, current.y, next.x, next.y);
          verified_path.push_back(next);
        }
      }
    }

    return verified_path;
  }

  /**
   * @brief Smooth path using gradient descent optimization
   * Reduces sharp corners while maintaining clearance from obstacles
   * @param path Input path to smooth
   * @return Smoothed path
   */
  std::vector<Point2D> smoothPath(const std::vector<Point2D> & path) const
  {
    if (path.size() <= 2)
    {
      return path; // Can't smooth very short paths
    }

    std::vector<Point2D> smoothed = path; // Copy original path

    // Gradient descent smoothing
    // Balance between staying close to original path and smoothness
    for (int iter = 0; iter < smoothing_iterations_; ++iter)
    {
      std::vector<Point2D> previous = smoothed;

      // Don't move start and end points
      for (size_t i = 1; i < smoothed.size() - 1; ++i)
      {
        // Data term: stay close to original path
        double dx_data = smoothing_weight_data_ * (path[i].x - smoothed[i].x);
        double dy_data = smoothing_weight_data_ * (path[i].y - smoothed[i].y);

        // Smoothness term: move towards average of neighbors
        double avg_x = (previous[i - 1].x + previous[i + 1].x) / 2.0;
        double avg_y = (previous[i - 1].y + previous[i + 1].y) / 2.0;
        double dx_smooth = smoothing_weight_smooth_ * (avg_x - smoothed[i].x);
        double dy_smooth = smoothing_weight_smooth_ * (avg_y - smoothed[i].y);

        // Update position
        Point2D new_point;
        new_point.x = smoothed[i].x + dx_data + dx_smooth;
        new_point.y = smoothed[i].y + dy_data + dy_smooth;

        // Verify that moving to new_point keeps all segments collision-free
        // Check: 1) current -> new_point, 2) prev -> new_point, 3) new_point -> next
        bool move_is_safe = hasLineOfSight(smoothed[i].x, smoothed[i].y, new_point.x, new_point.y, true);

        if (move_is_safe)
        {
          // Also verify segments to neighbors remain clear after the move
          move_is_safe = hasLineOfSight(previous[i - 1].x, previous[i - 1].y, new_point.x, new_point.y, true) &&
                         hasLineOfSight(new_point.x, new_point.y, previous[i + 1].x, previous[i + 1].y, true);
        }

        if (move_is_safe)
        {
          smoothed[i] = new_point;
        }
      }
    }

    RCLCPP_DEBUG(get_logger(), "Path smoothing completed: %d iterations", smoothing_iterations_);

    return smoothed;
  }

  /**
   * @brief A* path planning on occupancy grid
   * @param start_x Start world x coordinate
   * @param start_y Start world y coordinate
   * @param goal_x Goal world x coordinate
   * @param goal_y Goal world y coordinate
   * @return Vector of waypoints in world coordinates (empty if no path found)
   */
  std::vector<Point2D> planAStar(double start_x, double start_y, double goal_x, double goal_y)
  {
    std::vector<Point2D> waypoints;

    if (!map_)
    {
      RCLCPP_ERROR(get_logger(), "A* planning failed: No map available");
      return waypoints;
    }

    // Convert start and goal to grid coordinates
    int start_gx, start_gy, goal_gx, goal_gy;
    if (!worldToGrid(start_x, start_y, start_gx, start_gy))
    {
      RCLCPP_ERROR(get_logger(), "A* planning failed: Start position (%.2f, %.2f) out of map bounds",
                   start_x, start_y);
      return waypoints;
    }

    if (!worldToGrid(goal_x, goal_y, goal_gx, goal_gy))
    {
      RCLCPP_ERROR(get_logger(), "A* planning failed: Goal position (%.2f, %.2f) out of map bounds",
                   goal_x, goal_y);
      return waypoints;
    }

    // Check if start or goal is occupied
    if (isOccupied(start_gx, start_gy))
    {
      RCLCPP_WARN(get_logger(), "A* planning: Start position is in occupied cell, attempting anyway");
      // Don't return - try to plan anyway
    }

    if (isOccupied(goal_gx, goal_gy))
    {
      RCLCPP_ERROR(get_logger(),
        "❌ A* planning FAILED: Goal position (%.2f, %.2f) is inside an obstacle! "
        "Please select a new goal in free space.",
        goal_x, goal_y);
      return waypoints; // Return empty vector - cannot plan to occupied goal
    }

    RCLCPP_DEBUG(get_logger(), "A* planning from grid (%d, %d) to (%d, %d)",
                 start_gx, start_gy, goal_gx, goal_gy);

    // A* search
    // Priority queue (min heap) for open set
    std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;

    // Closed set: stores visited grid cells
    std::set<std::pair<int, int>> closed_set;

    // Parent map for path reconstruction: (x,y) -> (parent_x, parent_y)
    std::unordered_map<int, std::pair<int, int>> parent_map;

    // Helper lambda to encode grid position as single integer
    auto encode = [this](int x, int y) -> int {
      return y * map_->info.width + x;
    };

    // Initialize with start node
    double h_start = euclideanDistance(start_gx, start_gy, goal_gx, goal_gy);
    open_set.push(GridNode(start_gx, start_gy, 0.0, h_start, -1, -1));

    // 8-connected neighbors: N, S, E, W, NE, NW, SE, SW
    const int dx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    const int dy[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    // Costs: orthogonal = 1.0, diagonal = sqrt(2)
    const double costs[8] = {1.0, 1.0, 1.0, 1.0, 1.414213562, 1.414213562, 1.414213562, 1.414213562};

    bool path_found = false;
    int iterations = 0;
    const int max_iterations = map_->info.width * map_->info.height; // Safety limit

    while (!open_set.empty() && iterations < max_iterations)
    {
      iterations++;

      // Get node with lowest f-cost
      GridNode current = open_set.top();
      open_set.pop();

      // Check if already visited
      if (closed_set.count({current.x, current.y}) > 0)
      {
        continue;
      }

      // Mark as visited
      closed_set.insert({current.x, current.y});

      // Check if reached goal
      if (current.x == goal_gx && current.y == goal_gy)
      {
        path_found = true;
        RCLCPP_INFO(get_logger(), "A* found path in %d iterations", iterations);
        break;
      }

      // Explore neighbors
      for (int i = 0; i < 8; ++i)
      {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];

        // Check bounds and obstacles (use inflated map for safer planning)
        if (isOccupied(nx, ny, true))
        {
          continue;
        }

        // Check if already visited
        if (closed_set.count({nx, ny}) > 0)
        {
          continue;
        }

        // Calculate costs
        double new_g = current.g + costs[i];
        double h = euclideanDistance(nx, ny, goal_gx, goal_gy);

        // Add to open set
        open_set.push(GridNode(nx, ny, new_g, h, current.x, current.y));

        // Store parent (overwrite if better path found later)
        parent_map[encode(nx, ny)] = {current.x, current.y};
      }
    }

    if (!path_found)
    {
      RCLCPP_ERROR(get_logger(), "A* planning failed: No path found after %d iterations", iterations);
      return waypoints; // Empty
    }

    // Reconstruct path from goal to start
    std::vector<std::pair<int, int>> grid_path;
    int cx = goal_gx;
    int cy = goal_gy;

    grid_path.push_back({cx, cy});

    while (cx != start_gx || cy != start_gy)
    {
      int key = encode(cx, cy);
      if (parent_map.find(key) == parent_map.end())
      {
        RCLCPP_ERROR(get_logger(), "A* path reconstruction failed: Missing parent for (%d, %d)", cx, cy);
        return waypoints; // Empty
      }

      auto parent = parent_map[key];
      cx = parent.first;
      cy = parent.second;
      grid_path.push_back({cx, cy});
    }

    // Reverse to get start -> goal order
    std::reverse(grid_path.begin(), grid_path.end());

    RCLCPP_INFO(get_logger(), "A* grid path length: %zu cells", grid_path.size());

    // Convert grid path to world coordinates
    std::vector<Point2D> world_path;
    for (const auto & [gx, gy] : grid_path)
    {
      double wx, wy;
      gridToWorld(gx, gy, wx, wy);
      world_path.push_back({wx, wy});
    }

    // Apply path optimizations
    std::vector<Point2D> optimized_path = world_path;

    // 1. Shortcut optimization: Remove unnecessary waypoints using line-of-sight
    if (enable_shortcut_optimization_)
    {
      size_t original_size = optimized_path.size();
      optimized_path = shortcutPath(optimized_path);
      RCLCPP_INFO(get_logger(), "Shortcut optimization: %zu -> %zu waypoints (removed %zu)",
                  original_size, optimized_path.size(), original_size - optimized_path.size());
    }

    // 2. Path smoothing: Reduce sharp corners
    if (enable_path_smoothing_ && optimized_path.size() > 2)
    {
      optimized_path = smoothPath(optimized_path);
      RCLCPP_INFO(get_logger(), "Path smoothing applied");
    }

    // 3. Resample path according to waypoint_spacing_
    // This ensures consistent spacing for the MPC controller
    // Save the optimized path before resampling (needed for verification step)
    std::vector<Point2D> pre_resample_path = optimized_path;

    waypoints.push_back(optimized_path[0]); // Always include start

    for (size_t i = 1; i < optimized_path.size(); ++i)
    {
      double dist = std::hypot(waypoints.back().x - optimized_path[i].x,
                               waypoints.back().y - optimized_path[i].y);

      // Add waypoint if distance exceeds spacing or if it's the last point
      if (dist >= waypoint_spacing_ || i == optimized_path.size() - 1)
      {
        waypoints.push_back(optimized_path[i]);
      }
    }

    RCLCPP_INFO(get_logger(), "Resampled path: %zu waypoints (spacing: %.2fm)",
                waypoints.size(), waypoint_spacing_);

    // 4. Verify and densify: Ensure all consecutive segments are collision-free
    // If resampling created segments that pass through obstacles, insert intermediate points
    size_t before_verification = waypoints.size();
    waypoints = verifyAndDensifyPath(waypoints, pre_resample_path);

    if (waypoints.size() > before_verification)
    {
      RCLCPP_INFO(get_logger(),
        "Path verification: Added %zu intermediate waypoints to ensure obstacle-free segments",
        waypoints.size() - before_verification);
    }

    RCLCPP_INFO(get_logger(), "Final path: %zu waypoints (spacing: %.2fm)",
                waypoints.size(), waypoint_spacing_);

    return waypoints;
  }

  std::vector<Point2D> planStraightLine(double x0, double y0, double xf, double yf)
  {
    std::vector<Point2D> waypoints;

    const double dx = xf - x0;
    const double dy = yf - y0;
    const double distance = std::hypot(dx, dy);

    if (distance < 0.1)
    {
      // Already at goal
      waypoints.push_back({xf, yf});
      return waypoints;
    }

    // Check if goal is in an occupied cell
    if (map_)
    {
      int goal_gx, goal_gy;
      worldToGrid(xf, yf, goal_gx, goal_gy);

      if (isOccupied(goal_gx, goal_gy))
      {
        RCLCPP_ERROR(get_logger(),
          "❌ Straight-line planning FAILED: Goal position (%.2f, %.2f) is inside an obstacle! "
          "Please select a new goal in free space.",
          xf, yf);
        return waypoints; // Return empty vector - cannot plan to occupied goal
      }
    }

    // Check if there's a clear line-of-sight from start to goal
    if (map_ && !hasLineOfSight(x0, y0, xf, yf, true))
    {
      RCLCPP_WARN(get_logger(),
        "Straight-line planner: No clear line-of-sight from (%.2f, %.2f) to (%.2f, %.2f). "
        "Path may be unsafe. Consider using A* planner instead.",
        x0, y0, xf, yf);
      // Return empty to signal failure - caller should try A* instead
      return waypoints;
    }

    // Generate waypoints along the line
    const int num_waypoints = std::max(
      2, static_cast<int>(std::ceil(distance / waypoint_spacing_)));

    for (int i = 1; i <= num_waypoints; ++i)
    {
      const double t = static_cast<double>(i) / num_waypoints;
      Point2D wp;
      wp.x = x0 + t * dx;
      wp.y = y0 + t * dy;
      waypoints.push_back(wp);
    }

    // If we have a map, verify all consecutive segments are collision-free
    if (map_)
    {
      for (size_t i = 0; i < waypoints.size() - 1; ++i)
      {
        if (!hasLineOfSight(waypoints[i].x, waypoints[i].y,
                           waypoints[i + 1].x, waypoints[i + 1].y, true))
        {
          RCLCPP_ERROR(get_logger(),
            "Straight-line planner: Segment between waypoints %zu and %zu crosses obstacle! "
            "This should not happen after initial line-of-sight check.",
            i, i + 1);
        }
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Straight-line planner: No map available - cannot verify path safety!");
    }

    RCLCPP_INFO(get_logger(), "Straight-line planner: Generated %zu waypoints", waypoints.size());

    return waypoints;
  }

  void publishPath(const std::vector<Point2D> & waypoints)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";

    for (const auto & wp : waypoints)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;  // No specific orientation
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
  }

  void publishVisualization(const std::vector<Point2D> & waypoints)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "global_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;  // Line width
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto & wp : waypoints)
    {
      geometry_msgs::msg::Point p;
      p.x = wp.x;
      p.y = wp.y;
      p.z = 0.1;  // Slightly above ground
      marker.points.push_back(p);
    }

    vis_pub_->publish(marker);
  }

  double goal_x_{0.0};
  double goal_y_{0.0};
  bool goal_received_{false};  // Flag to prevent planning until goal is set via RViz2
  bool force_replan_{false};   // Flag to force replanning when new goal received
  double waypoint_spacing_{1.0};
  double replan_distance_threshold_{0.5};
  bool planning_enabled_{true};
  bool use_simple_line_planner_{true};
  int occupancy_threshold_{50};
  bool treat_unknown_as_free_{true};
  double inflation_radius_{0.5};

  // Path smoothing parameters
  bool enable_path_smoothing_{true};
  int smoothing_iterations_{50};
  double smoothing_weight_data_{0.5};
  double smoothing_weight_smooth_{0.3};
  bool enable_shortcut_optimization_{true};

  double current_x_{0.0};
  double current_y_{0.0};
  bool pose_received_{false};
  bool map_received_{false};
  bool path_published_{false};

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<int8_t> inflated_map_;  // Inflated occupancy grid for safer planning
  std::vector<Point2D> last_planned_path_;  // Last successfully planned path for deviation checking

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // TF2 for coordinate frame transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_;
  std::string base_link_frame_;

  std::mutex data_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
