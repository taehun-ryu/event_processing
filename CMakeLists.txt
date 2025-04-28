################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(event_processing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
################################################################################
# Build
################################################################################
include_directories(
  include
  ${HDF5_INCLUDE_DIRS}
)

# ------- RCLCPP -------
add_executable(event_3d_visualizer_node
  src/event_3d_visualizer.cpp
)
ament_target_dependencies(event_3d_visualizer_node
  rclcpp
  dvs_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
)

# ------- C++ -------
add_executable(event_cpp
  src/main.cpp
  src/H5Reader.cpp
  src/EventArray.cpp
)
target_link_libraries(event_cpp PRIVATE
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Ceres::ceres
  ${HDF5_LIBRARIES}
)
################################################################################
# Install
################################################################################
# lib/<package>: executable binaries
install(
  TARGETS
    event_3d_visualizer_node
    event_cpp
  DESTINATION lib/${PROJECT_NAME}
)
# share/<package>: launch files and other resources
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
################################################################################
# Macro for ament package
################################################################################
ament_package()
