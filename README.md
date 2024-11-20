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

  
