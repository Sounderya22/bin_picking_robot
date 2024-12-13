cmake_minimum_required(VERSION 3.8)
project(custom_motions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(approach_retreat
    src/approach_retreat.cpp)
target_include_directories(approach_retreat
    PUBLIC include)
ament_target_dependencies(approach_retreat
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(cartesian_path
    src/cartesian_path.cpp)
target_include_directories(cartesian_path
    PUBLIC include)
ament_target_dependencies(cartesian_path
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(setup_collision_objects
src/setup_collision_objects.cpp)
target_include_directories(setup_collision_objects
PUBLIC include)
ament_target_dependencies(setup_collision_objects
ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs
moveit_visual_tools
)


add_executable(perception_pose
    src/perception_pose.cpp)
target_include_directories(perception_pose
    PUBLIC include)
ament_target_dependencies(perception_pose
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

	add_executable(visualise_pose
    src/visualise_pose.cpp)
target_include_directories(visualise_pose
    PUBLIC include)
ament_target_dependencies(visualise_pose
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	moveit_visual_tools)



# Install the executable
install(TARGETS
	approach_retreat
	cartesian_path
    setup_collision_objects
    perception_pose
	visualise_pose
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
