cmake_minimum_required(VERSION 3.5)
project(robotnik_sensors_gazebo NONE)
find_package(ament_cmake REQUIRED)
ament_package()

## Install some arbitrary executables
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/run_me_for_fun.bash
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})
