# Intelligent Bin-Picking Robot

![CICD Workflow status](https://github.com/Sounderya22/bin_picking_robot/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/Sounderya22/bin_picking_robot/branch/main/graph/badge.svg)](https://codecov.io/gh/Sounderya22/bin_picking_robot) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

This repository contains the deliverables for the Final Project of the course **ENPM700: Software Development for Robotics** 

## Project Contributor
[Sounderya Varagur Venugopal](https://github.com/Sounderya22) - 121272423
## Project Overview
This project proposes an intelligent bin-picking robot system capable of autonomously identifying, grasping, and sorting objects from mixed bins into designated locations. This system will leverage computer vision techniques for object detection and pose detection to enable flexible and efficient picking of various objects in unstructured environments. The prototype will demonstrate core capabilities needed for applications in e-commerce fulfillment, manufacturing part handling, and recycling sorting.
## UML
The UML diagrams can be found [here](https://github.com/Sounderya22/bin_picking_robot/blob/main/UML/initial)
## Product backlog
The product backlog can be found [here](https://docs.google.com/spreadsheets/d/1dOl7ko8kiRCL01SYXUkV1blOiVYMngp9uYMhj_psIf0/edit?usp=sharing).
## Sprint Planning
The sprint planning sheet can be found [here](https://docs.google.com/document/d/1k97gEPnfccyWxz8z-w4MMVNBKQnhrrpmbxowb54gQMY/edit?usp=sharing).
## Dependencies with license
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense) - Apache License 2.0
- [Point Cloud Library](https://github.com/PointCloudLibrary/pcl) - BSD-2-Clause License
- [Grasp Pose Detection(GPD)](https://github.com/atenpas/gpd) - MIT License
- [MoveIt2](https://moveit.picknik.ai/main/index.html) - Apache License 2.0

## Building the code
Run the commands listed below to build the package and libraries.
```bash
rm -rf build/ install/
colcon build 
source install/setup.bash
```

### Building for Unit and Integration Tests

Further, run these commands to build for running the unit tests and integration tests.
```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```
### Running the Unit and Integration Tests

Execute the following commands to run the previosuly built unit and integration tests.
```bash
source install/setup.bash
colcon test
```

### Generating Coverage Reports After Running Colcon Test

To obtain the coverage reports (first make sure to have run the unit test already), execute the comands listed below.
```bash
colcon test
```

## How to generate project documentation
``` bash
./do-docs.bash
```

### Static Code Analysis
To check the static code analysis of this project, check the `results/cppcheck_output.txt` file to see the output on using the *cppcheck* tool. You should not be able to see any issues or problems, with all the files checked successfully.

This can be self-verified as well by running the following command in the highest-level directory of the project.
```sh
# Install cppcheck (ignore if already installed):
  sudo apt install cppcheck
# Navigate to the 'src' directory
  cd src/
# Self-check the static code analysis using Cppcheck:
  cppcheck --enable=all --std=c++11 --std=c++17 --enable=information --check-config --suppress=missingInclude --suppress=*:*test*/ --suppress=unmatchedSuppression $( find . -name *.cpp | grep -vE -e "^./build/") > cppcheck_results.txt 2>&1

```

On running the above command, you should see the same output in the `cppcheck_results.txt` file.

### Running the simulation
```bash
ros2 launch ur_simulation_gazebo ur_sim_pick_and_place.launch.py
```


