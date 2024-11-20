# Create a library called "myLib1" (in Linux, this library is created
# with the name of either libmyLib1.a or myLib1.so).
add_library (pose_estimation
  # list of cpp source files:
  src.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(pose_estimation PUBLIC
  # list of directories:
  ${CMAKE_SOURCE_DIR}/libs
  )
