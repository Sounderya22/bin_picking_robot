#ifndef POSE_ESTIMATION_HPP
#define POSE_ESTIMATION_HPP

#include "robot_motion_planner/robot.hpp"

class PoseEstimation {
public:
    // Constructor
    PoseEstimation();

    // Destructor
    ~PoseEstimation();

    // Method to estimate pose from 3D data
    Pose estimateFrom3DData();

private:
    // Object pose as a private member
    Pose objectPose;
};

#endif // POSE_ESTIMATION_HPP
