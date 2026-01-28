cmake_minimum_required(VERSION 3.8)
project(dwa_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(social_mpc_nav REQUIRED)
find_package(person_tracker REQUIRED)

# Include directories
include_directories(
  include
  ${social_mpc_nav_INCLUDE_DIRS}
)

# Pure DWA Controller Node (recommended)
add_executable(dwa_controller_node
  src/dwa_controller_pure.cpp
)

# Original DWA with VLM (for comparison)
add_executable(dwa_controller_vlm_node
  src/dwa_controller_node.cpp
)

ament_target_dependencies(dwa_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  social_mpc_nav
  person_tracker
)

ament_target_dependencies(dwa_controller_vlm_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  social_mpc_nav
)

target_link_libraries(dwa_controller_vlm_node
  ${social_mpc_nav_LIBRARIES}
)

# Install targets
install(TARGETS
  dwa_controller_node
  dwa_controller_vlm_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
