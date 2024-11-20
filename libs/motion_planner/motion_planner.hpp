#ifndef MOTION_PLANNER_HPP
#define MOTION_PLANNER_HPP

#include "robot_motion_planner/robot_motion_planner.hpp"

// MotionPlanner class
class MotionPlanner {
public:
    // Constructor
    MotionPlanner(RobotMotionPlanner& robotMotionPlanner);

    // Destructor
    ~MotionPlanner();

    // Method to compute a trajectory
    Trajectory computeTrajectory(const std::vector<double>& start, const std::vector<double>& goal);

private:

    // Configuration parameters
    double max_velocity_;
    double max_acceleration_;
    Trajectory trajectoryPlan;
    RobotMotionPlanner& robotMotionPlanner_;
};

#endif // MOTION_PLANNER_HPP
