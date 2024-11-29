# Intelligent Bin-Picking Robot

![CICD Workflow status](https://github.com/Sounderya22/bin_picking_robot/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/Sounderya22/bin_picking_robot/branch/main/graph/badge.svg)](https://codecov.io/gh/Sounderya22/bin_picking_robot) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

This repository contains the deliverables for the Final Project of the course **ENPM700: Software Development for Robotics**

## Project Contributor
[Sounderya Varagur Venugopal](https://github.com/Sounderya22) - 121272423
## Project Overview
This project proposes an intelligent bin-picking robot system capable of autonomously identifying, grasping, and sorting objects from mixed bins into designated locations. This system will leverage computer vision techniques for object detection and pose detection to enable flexible and efficient picking of various objects in unstructured environments. The prototype will demonstrate core capabilities needed for applications in e-commerce fulfillment, manufacturing part handling, and recycling sorting.
## UML
The initial diagrams can be found [here](https://github.com/Sounderya22/bin_picking_robot/blob/main/UML/initial)
## Product backlog
The product backlog can be found [here](https://docs.google.com/spreadsheets/d/1dOl7ko8kiRCL01SYXUkV1blOiVYMngp9uYMhj_psIf0/edit?usp=sharing).
## Sprint Planning
The sprint planning sheet can be found [here](https://docs.google.com/document/d/1k97gEPnfccyWxz8z-w4MMVNBKQnhrrpmbxowb54gQMY/edit?usp=sharing).
## Dependencies with license
- [YOLOv8](https://docs.ultralytics.com/models/yolov8/) - GNU Affero General Public License v3.0
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense) - Apache License 2.0
- [Point Cloud Library](https://github.com/PointCloudLibrary/pcl) - BSD-2-Clause License
- [Grasp Pose Detection(GPD)](https://github.com/atenpas/gpd) - MIT License
- [MoveIt2](https://moveit.picknik.ai/main/index.html) - Apache License 2.0

## Build Instructions
```bash
# Download the code:
  git clone https://github.com/Sounderya22/bin_picking_robot
  cd bin_picking_robot/
# if you don't have gcovr or lcov installed, do:
  sudo apt-get install gcovr lcov
# Configure the project and generate a native build system:
  # Must re-run this command whenever any CMakeLists.txt file has been changed.
  # Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Compile and build the project:
  # rebuild only files that are modified since the last build
  cmake --build build/
  # To do a clean compile, run unit test, and generate the covereage report
  cmake --build build/ --clean-first --target all app_coverage test_coverage

This generates a index.html page in the build/test_coverage and build/app_coverage sub-directory that can be viewed locally in a web browser.

# open a web browser to browse the test coverage report
  open build/test_coverage/index.html
# opdn a web browser to browse the app coverage report
  open build/app_coverage/index.html
# Run program:
  ./build/app/bin-picking
# Clean and start over:
  rm -rf build/

  
## How to generate module / package dependency graph

``` bash
colcon graph --dot | dot -Tpng -o depGraph.png
open depGraph.png
```
[<img src=screenshots/depGraph.png
    width="20%" 
    style="display: block; margin: 0 auto"
    />](screenshots/depGraph.png)



## How to build and run demo

First, make sure we install the catch2 ROS2 package.
```bash
$ source /opt/ros/humble/setup.bash  # if needed
$ sudo apt install ros-${ROS_DISTRO}-catch-ros2
```
Now, we can build our system:
```bash
rm -rf build/ install/
colcon build 

```
And finally, run the demo:

```bash
source install/setup.bash
ros2 launch my_controller run_demo.launch.yaml
```
example output:

```
[INFO] [launch]: All log files can be found below /home/tchang/.ros/log/2024-11-14-01-15-43-657639-tchang-IdeaPad-3-17ABA7-2012192
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [2012193]
[INFO] [listener-2]: process started with pid [2012195]
[listener-2] Calling OpenCV function
[listener-2] [INFO] [1731564943.913295525] [my_model]: Calling ROS function
[talker-1] Calling OpenCV function
[talker-1] [INFO] [1731564944.413417817] [my_model]: Calling ROS function
[talker-1] [INFO] [1731564944.413485842] [talker]: Publishing: 101 Hello, world! 0
[listener-2] [INFO] [1731564944.413881423] [listener]: subName=subscription0, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.413998198] [listener]: subName=subscription1, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.414027042] [listener]: subName=subscription2, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.414057074] [listener]: subName=subscription3, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.414086617] [listener]: subName=subscription4, I heard : 'Hello, world! 0'
[talker-1] Calling OpenCV function
[talker-1] [INFO] [1731564944.911201194] [my_model]: Calling ROS function
[talker-1] [INFO] [1731564944.911250432] [talker]: Publishing: 102 Hello, world! 1
[listener-2] [INFO] [1731564944.911475601] [listener]: subName=subscription0, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911539017] [listener]: subName=subscription2, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911562413] [listener]: subName=subscription3, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911591467] [listener]: subName=subscription4, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911633512] [listener]: subName=subscription1, I heard : 'Hello, world! 1'
[talker-1] Calling OpenCV function
[talker-1] [INFO] [1731564945.411270616] [my_model]: Calling ROS function
[talker-1] [INFO] [1731564945.411329702] [talker]: Publishing: 103 Hello, world! 2

```

## How to build tests (unit test and integration test)
We want to run tests with code coverage.  Therefore, we need to enable the code coverage option.

```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
```

## How to generate coverage reports after running colcon test

First make sure we have run the unit test already.

```bash
colcon test
```

### Coverage report for `my_controller`:

``` bash
ros2 run my_controller generate_coverage_report.bash
open build/my_controller/test_coverage/index.html
```

### Coverage report for `my_model`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_model \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RUN=1
open build/my_model/test_coverage/index.html
```

### Automate the previous steps and combine both coverage reports

``` bash
./do-tests-and-coverage.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
```

## How to use GitHub CI to upload coverage report to Codecov

There is already a `.github/workflows/run-unit-test-and-upload-codecov.yml` file provided.  But we still need to create a codecov account.