# Create a library called "myLib1" (in Linux, this library is created
# with the name of either libmyLib1.a or myLib1.so).
add_library (robot
  # list of cpp source files:
  robot.cpp
  )

target_link_libraries(robot PUBLIC motion_planner)

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(robot PRIVATE -static
  # list of directories:
  )

add_library (robot_motion_planner
  # list of cpp source files:
  robot_motion_planner.cpp
  )

target_link_libraries(robot_motion_planner PUBLIC motion_planner)

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(robot_motion_planner PRIVATE -static
  # list of directories:
  ${CMAKE_SOURCE_DIR}/libs
  )
