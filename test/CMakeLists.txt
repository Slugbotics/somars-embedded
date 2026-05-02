# ============================================================================
# Standalone test build – runs on any machine with CMake + Eigen3.
# No ROS2 required.
#
# Usage:
#   cd test && mkdir build && cd build
#   cmake .. && make && ./test_projection_math
# ============================================================================

cmake_minimum_required(VERSION 3.8)
project(somars_controls_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(test_projection_math test_projection_math.cpp)
target_link_libraries(test_projection_math Eigen3::Eigen)
