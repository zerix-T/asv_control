cmake_minimum_required(VERSION 3.8)

project(asv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# Dependencies
# =========================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(mavlink REQUIRED)

# =========================================================
# Include directories
# =========================================================

include_directories(
  include
)

# =========================================================
# asv_key_teleop
# =========================================================

add_executable(asv_key_teleop
  src/asv_key_teleop.cpp
)

ament_target_dependencies(asv_key_teleop
  rclcpp
  geometry_msgs
  std_msgs
)

install(
  TARGETS asv_key_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================
# forward_test
# =========================================================

add_executable(forward_test
  src/test_move.cpp
)

ament_target_dependencies(forward_test
  rclcpp
  std_msgs
)

install(
  TARGETS forward_test
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================
# pointcloud_to_laserscan COMPONENT
# =========================================================

add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan.cpp
)

target_compile_definitions(pointcloud_to_laserscan
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL"
)

ament_target_dependencies(pointcloud_to_laserscan
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Register the component
rclcpp_components_register_nodes(
  pointcloud_to_laserscan
  "asv_control::PointcloudToLaserscan"
)

# Install component library
install(
  TARGETS pointcloud_to_laserscan
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =========================================================
# laserscan_to_obstacledistance COMPONENT
# =========================================================

add_library(laserscan_to_obstacledist SHARED
  src/laserscan_to_obstacledist.cpp
)

target_compile_definitions(laserscan_to_obstacledist
  PRIVATE "LASERSCAN_TO_OBSTACLEDIST_BUILDING_DLL"
)

ament_target_dependencies(laserscan_to_obstacledist
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Register the component
rclcpp_components_register_nodes(
  laserscan_to_obstacledist
  "asv_control::LaserscanToObstacleDist"
)

# Install component library
install(
  TARGETS laserscan_to_obstacledist
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =========================================================
# Launch files
# =========================================================

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================================================
# Headers
# =========================================================

# Uncomment if you have headers:
#
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# =========================================================
# Testing
# =========================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

# =========================================================
# Package
# =========================================================

ament_package()
