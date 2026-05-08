cmake_minimum_required(VERSION 3.8)
project(somars_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Nodes
# ---------------------------------------------------------------------------

add_executable(target_localizer src/target_localizer.cpp)
ament_target_dependencies(target_localizer
  rclcpp px4_msgs geometry_msgs std_msgs sensor_msgs)
target_link_libraries(target_localizer Eigen3::Eigen)

add_executable(guidance_node src/guidance_node.cpp)
ament_target_dependencies(guidance_node
  rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(guidance_node Eigen3::Eigen)

add_executable(offboard_manager src/offboard_manager.cpp)
ament_target_dependencies(offboard_manager
  rclcpp px4_msgs std_msgs)

add_executable(mock_nodes test/mock_nodes.cpp)
ament_target_dependencies(mock_nodes
  rclcpp px4_msgs geometry_msgs std_msgs)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS
  target_localizer
  guidance_node
  offboard_manager
  mock_nodes
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
