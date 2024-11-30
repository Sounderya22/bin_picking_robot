cmake_minimum_required(VERSION 3.8)
project(ros2_template_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(ros2_template_node src/ros2_template_node.cpp)
ament_target_dependencies(ros2_template_node)

add_executable(talker src/talker_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/listener_node.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(timer src/timer_node.cpp)
ament_target_dependencies(timer rclcpp std_msgs)

add_executable(parameters src/parameters_node.cpp)
ament_target_dependencies(parameters rclcpp std_msgs)

target_include_directories(ros2_template_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
$<INSTALL_INTERFACE:include>)

target_compile_features(ros2_template_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


if(BUILD_TESTING)

  set(
    _dirs_to_check
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/launch
  )

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(${_dirs_to_check})

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(${_dirs_to_check} FILTERS -runtime/int)

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(${_dirs_to_check})

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(${_dirs_to_check} ${CMAKE_CURRENT_SOURCE_DIR})

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(${_dirs_to_check} LANGUAGE C++)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME} test/test_ros2_template_node.cpp)

  # find_package(catch_ros2 REQUIRED) # add integration test support
  # add_executable(integration_test
  #   test/integration_test.cpp)
  # ament_target_dependencies(integration_test
  #   rclcpp
  #   std_msgs my_model)
  # target_link_libraries(integration_test
  #   catch_ros2::catch_ros2_with_node_main
  #   my_model)
  # catch_ros2_add_integration_test (Demo_Integration_Test
  #   LAUNCH_FILE integration_test.launch.yaml)

endif()


###################################################
# Create the compilation database for clangd and move it out of the build dir.
###################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

###################################################
# Doxygen Rules
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
###################################################
if (DOXYGEN_FOUND)
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

###################################
# Specify Targets 
###################################
# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test
  ${PROJECT_SOURCE_DIR}/scripts
  )


###################################
# Installation
###################################
# Install executables:

install(TARGETS
ros2_template_node
talker
listener
timer
parameters
# integration_test   ## For integration test if required.
DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME})

## Install some arbitrary executables
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
scripts/run_me_for_fun.bash
scripts/generate_coverage_report.bash
DESTINATION lib/${PROJECT_NAME})

ament_package() 
