#ifndef ROBOT_MOTION_PLANNER_HPP
#define ROBOT_MOTION_PLANNER_HPP

#include "robot.hpp"
#include <iostream>


// Define a structure to represent a trajectory
struct Trajectory {
    std::vector<double> positions;  // Joint positions
    std::vector<double> velocities; // Joint velocities
    double duration;                // Time duration for the trajectory
};

// RobotMotionPlanner class
class RobotMotionPlanner {
public:
    // Constructor
    RobotMotionPlanner(Robot& robot);

    // Destructor
    ~RobotMotionPlanner();

    // Method to execute a trajectory
    bool executeTrajectory(const Trajectory& trajectory);

private:
    Robot& robot_;

};

#endif // ROBOT_MOTION_PLANNER_HPP
