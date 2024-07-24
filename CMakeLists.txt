cmake_minimum_required(VERSION 3.5)
project(manual_lidar_camera_calibration)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(pointcloud_loader SHARED
  src/pointcloud_loader.cpp
)

ament_auto_add_executable(
  pointcloud_loader_node
  src/pointcloud_loader_node.cpp
)

target_link_libraries(
  pointcloud_loader_node
  ${PCL_LIBRARIES}
)

install(PROGRAMS
  scripts/camera_points_publisher.py
  scripts/extrinsic_calibrator.py
DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake_python)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)