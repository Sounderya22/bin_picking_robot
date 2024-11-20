#include "motion_planner.hpp"
#include <cmath>
#include <iostream>

// Constructor
MotionPlanner::MotionPlanner(RobotMotionPlanner& robotMotionPlanner) : robotMotionPlanner_(robotMotionPlanner) { 
    std::cout << "MotionPlanner initialized with default parameters.\n";
}

// Destructor
MotionPlanner::~MotionPlanner() {
    std::cout << "MotionPlanner destroyed.\n";
}

// Method to compute a trajectory
Trajectory MotionPlanner::computeTrajectory(const std::vector<double>& start, const std::vector<double>& goal) {
    
}