cmake_minimum_required(VERSION 3.8)
project(perception_system)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_transformation_node src/pointcloud_transformation.cpp)

ament_target_dependencies(pointcloud_transformation_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS pointcloud_transformation_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(vision_pcl_processing src/vision_based_processing.cpp)

ament_target_dependencies(vision_pcl_processing
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS vision_pcl_processing
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY config launch dl_models 
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
