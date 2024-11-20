# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vv/bin_picking_robot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vv/bin_picking_robot/build

# Include any dependencies generated for this target.
include libs/robot/CMakeFiles/robot.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include libs/robot/CMakeFiles/robot.dir/compiler_depend.make

# Include the progress variables for this target.
include libs/robot/CMakeFiles/robot.dir/progress.make

# Include the compile flags for this target's objects.
include libs/robot/CMakeFiles/robot.dir/flags.make

libs/robot/CMakeFiles/robot.dir/src.cpp.o: libs/robot/CMakeFiles/robot.dir/flags.make
libs/robot/CMakeFiles/robot.dir/src.cpp.o: ../libs/robot/src.cpp
libs/robot/CMakeFiles/robot.dir/src.cpp.o: libs/robot/CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vv/bin_picking_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libs/robot/CMakeFiles/robot.dir/src.cpp.o"
	cd /home/vv/bin_picking_robot/build/libs/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libs/robot/CMakeFiles/robot.dir/src.cpp.o -MF CMakeFiles/robot.dir/src.cpp.o.d -o CMakeFiles/robot.dir/src.cpp.o -c /home/vv/bin_picking_robot/libs/robot/src.cpp

libs/robot/CMakeFiles/robot.dir/src.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/src.cpp.i"
	cd /home/vv/bin_picking_robot/build/libs/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vv/bin_picking_robot/libs/robot/src.cpp > CMakeFiles/robot.dir/src.cpp.i

libs/robot/CMakeFiles/robot.dir/src.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/src.cpp.s"
	cd /home/vv/bin_picking_robot/build/libs/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vv/bin_picking_robot/libs/robot/src.cpp -o CMakeFiles/robot.dir/src.cpp.s

# Object files for target robot
robot_OBJECTS = \
"CMakeFiles/robot.dir/src.cpp.o"

# External object files for target robot
robot_EXTERNAL_OBJECTS =

libs/robot/librobot.a: libs/robot/CMakeFiles/robot.dir/src.cpp.o
libs/robot/librobot.a: libs/robot/CMakeFiles/robot.dir/build.make
libs/robot/librobot.a: libs/robot/CMakeFiles/robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vv/bin_picking_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library librobot.a"
	cd /home/vv/bin_picking_robot/build/libs/robot && $(CMAKE_COMMAND) -P CMakeFiles/robot.dir/cmake_clean_target.cmake
	cd /home/vv/bin_picking_robot/build/libs/robot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/robot/CMakeFiles/robot.dir/build: libs/robot/librobot.a
.PHONY : libs/robot/CMakeFiles/robot.dir/build

libs/robot/CMakeFiles/robot.dir/clean:
	cd /home/vv/bin_picking_robot/build/libs/robot && $(CMAKE_COMMAND) -P CMakeFiles/robot.dir/cmake_clean.cmake
.PHONY : libs/robot/CMakeFiles/robot.dir/clean

libs/robot/CMakeFiles/robot.dir/depend:
	cd /home/vv/bin_picking_robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vv/bin_picking_robot /home/vv/bin_picking_robot/libs/robot /home/vv/bin_picking_robot/build /home/vv/bin_picking_robot/build/libs/robot /home/vv/bin_picking_robot/build/libs/robot/CMakeFiles/robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/robot/CMakeFiles/robot.dir/depend

