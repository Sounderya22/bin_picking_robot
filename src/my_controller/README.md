# `my_controller`

Here's a simple ROS2 package that demonstrates how integration test
(aka level 2 unit test) can be done using Catch2.

This ROS2 package depends on the `my_model` module.  This dependency
is specified in the package's `package.xml` file:

```
  <depend>my_model</depend>
```

Alternatively, we could also specify the dependency by creating a
`colcon.pkg` file with the content listed below.  But since this is a
ROS package, we must use `package.xml` instead of `colcon.pkg`.
Otherwise the package will not show up in the list of ROS2 packages
(ie., output of `ros2 pkg list`).

```
{
    "name": "my_controller",
    "type": "cmake",
    "dependencies" : ["my_model"]
}
```

## How to build as a stand-alone CMake project
First, make sure you have already built the `my_mode` package.  See `../my_model/README.md`.

```bash
# Configure the project and generate a native build system:
  # Re-run this command whenever any CMakeLists.txt file has been changed.
  cmake -S ./ -B build/
# Compile and build the project:
  # rebuild only files that are modified since the last build
  cmake --build build/
  # or rebuild everything from scracth
  cmake --build build/ --clean-first
  # to see verbose output, do:
  cmake --build build/ --verbose
# Run unit tests:
  ctest --test-dir build/
  # or if you have older cmake
  cd build/; ctest; cd -
# Build docs:
  cmake --build build/ --target docs
  # open a web browser to browse the doc
  open docs/html/index.html
# Clean
  cmake --build build/ --target clean
# Clean and start over:
  rm -rf build/
```

## Build and run ROS2 integration test 

The ROS2 integration test executes the `my_controller` ROS2 package.
Therefore, we need to use colcon to build `my_controller` as a ROS2 package.

```bash
cd ../../
colcon build --packages-select my_controller  # will also build the "my_model" dependency
colcon test --event-handlers console_direct+ --packages-select my_controller
```

The integration test uses the Catch2 test framework that is similar to Google Test.  In the particular example, it tests the `talker` node that is provided by the `my_controller` ROS2 package.  See [CMakeLists.txt](CMakeLists.txt), [test/integration_test.cpp](test/integration_test.cpp) and [launch/integration_test.launch.yaml](launch/integration_test.launch.yaml).
