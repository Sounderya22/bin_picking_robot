cmake_minimum_required(VERSION 3.8)
project(codestub)

add_subdirectory (grasp_planner)
add_subdirectory (motion_planner)
add_subdirectory (object_detection)
add_subdirectory (object_manipulation)
add_subdirectory (perception_system)
add_subdirectory (pose_estimation)
add_subdirectory (robot_motion_planner)
add_subdirectory (sensor_manager)

install(
    DIRECTORY 
    .
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/codestub
)
