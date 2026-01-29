cmake_minimum_required(VERSION 3.8)
project(social_mpc_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(people_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gb_visual_detection_3d_msgs REQUIRED)
find_package(person_tracker REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

# Gazebo Transport for physics-based pose subscription
find_package(gz-transport13 REQUIRED)

set(MSG_FILES
  "msg/Person2D.msg"
  "msg/People2D.msg"
  "msg/VLMParameters.msg"
  "msg/TranslatorStatus.msg"
  "msg/SARLOutput.msg"
  "msg/NavigationExplanation.msg"
)

set(SRV_FILES
  "srv/EvaluateSARLBatch.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
)

add_library(social_contract_helper src/social_contract.cpp)
ament_target_dependencies(social_contract_helper
  rclcpp
  std_msgs
)
rosidl_target_interfaces(social_contract_helper
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(gt_localization_node src/gt_localization_node.cpp)
ament_target_dependencies(gt_localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(gt_localization_node
  gz-transport13::gz-transport13
)
rosidl_target_interfaces(gt_localization_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pedestrian_logger_node src/pedestrian_logger_node.cpp)
ament_target_dependencies(pedestrian_logger_node
  rclcpp
  hunav_msgs
)
rosidl_target_interfaces(pedestrian_logger_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(crowd_state_node src/crowd_state_node.cpp)
ament_target_dependencies(crowd_state_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
  std_msgs
)
rosidl_target_interfaces(crowd_state_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(people_adapter_node src/people_adapter_node.cpp)
ament_target_dependencies(people_adapter_node
  rclcpp
  people_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(people_adapter_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(global_planner_node src/global_planner_node.cpp)
ament_target_dependencies(global_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
rosidl_target_interfaces(global_planner_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(mpc_controller_node src/mpc_controller_node.cpp)
ament_target_dependencies(mpc_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  person_tracker
)
rosidl_target_interfaces(mpc_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mpc_controller_node social_contract_helper)

add_executable(mpc_controller_vlm_node src/mpc_controller_vlm_node.cpp)
ament_target_dependencies(mpc_controller_vlm_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  visualization_msgs
  person_tracker
)
rosidl_target_interfaces(mpc_controller_vlm_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mpc_controller_vlm_node social_contract_helper)

add_executable(mpc_controller_sarl_node src/mpc_controller_sarl_node.cpp)
ament_target_dependencies(mpc_controller_sarl_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  visualization_msgs
  person_tracker
)
rosidl_target_interfaces(mpc_controller_sarl_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mpc_controller_sarl_node social_contract_helper)

# CasADi-based MPC controller (from system install)
set(CASADI_INCLUDE_DIR "/usr/local/include")
set(CASADI_LIBRARY_DIR "/usr/local/lib")

add_executable(mpc_controller_casadi_node src/mpc_controller_casadi_node.cpp)
ament_target_dependencies(mpc_controller_casadi_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  person_tracker
)
target_include_directories(mpc_controller_casadi_node PRIVATE
  ${CASADI_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)
target_link_directories(mpc_controller_casadi_node PRIVATE ${CASADI_LIBRARY_DIR})
target_link_libraries(mpc_controller_casadi_node
  social_contract_helper
  casadi
)
# Set RPATH so the binary can find libcasadi.so at runtime
set_target_properties(mpc_controller_casadi_node PROPERTIES
  INSTALL_RPATH "${CASADI_LIBRARY_DIR}"
  BUILD_RPATH "${CASADI_LIBRARY_DIR}"
)
rosidl_target_interfaces(mpc_controller_casadi_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# CasADi-based MPC controller with SARL attention-weighted social costs
add_executable(mpc_controller_casadi_sarl_node src/mpc_controller_casadi_sarl_node.cpp)
ament_target_dependencies(mpc_controller_casadi_sarl_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  visualization_msgs
  person_tracker
)
target_include_directories(mpc_controller_casadi_sarl_node PRIVATE
  ${CASADI_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)
target_link_directories(mpc_controller_casadi_sarl_node PRIVATE ${CASADI_LIBRARY_DIR})
target_link_libraries(mpc_controller_casadi_sarl_node
  social_contract_helper
  casadi
)
set_target_properties(mpc_controller_casadi_sarl_node PROPERTIES
  INSTALL_RPATH "${CASADI_LIBRARY_DIR}"
  BUILD_RPATH "${CASADI_LIBRARY_DIR}"
)
rosidl_target_interfaces(mpc_controller_casadi_sarl_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(vlm_integration_node src/vlm_integration_node.cpp)
ament_target_dependencies(vlm_integration_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gb_visual_detection_3d_msgs
  person_tracker
  cv_bridge
  OpenCV
)
target_link_libraries(vlm_integration_node ${CURL_LIBRARIES})
target_include_directories(vlm_integration_node PRIVATE ${CURL_INCLUDE_DIRS})
rosidl_target_interfaces(vlm_integration_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(vlm_translator_node src/vlm_translator_node.cpp)
ament_target_dependencies(vlm_translator_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)
rosidl_target_interfaces(vlm_translator_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install shared library to lib/ (for runtime linker)
install(TARGETS social_contract_helper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install executables to lib/${PROJECT_NAME}/
install(TARGETS
  gt_localization_node
  pedestrian_logger_node
  crowd_state_node
  people_adapter_node
  global_planner_node
  mpc_controller_node
  mpc_controller_vlm_node
  mpc_controller_sarl_node
  mpc_controller_casadi_node
  mpc_controller_casadi_sarl_node
  vlm_integration_node
  vlm_translator_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/sarl_bridge_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(social_contract_helper)

ament_package()

