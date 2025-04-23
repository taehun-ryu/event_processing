cmake_minimum_required(VERSION 3.10)
project(event_processing)

add_executable(main
  src/main.cpp
  src/H5Reader.cpp
  src/EventArray.cpp
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Ceres REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

target_include_directories(main PRIVATE
  include
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(main PRIVATE
  ${OpenCV_LIBS}
  Eigen3::Eigen
  #Ceres::ceres
  ${HDF5_LIBRARIES}
)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")