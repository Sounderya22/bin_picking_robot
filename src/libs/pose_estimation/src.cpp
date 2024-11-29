#include "pose_estimation.hpp"

// Constructor
PoseEstimation::PoseEstimation() {
}

// Destructor
PoseEstimation::~PoseEstimation() {
}

// Method to estimate pose from 3D data
Pose PoseEstimation::estimateFrom3DData() {
    return objectPose;
}
