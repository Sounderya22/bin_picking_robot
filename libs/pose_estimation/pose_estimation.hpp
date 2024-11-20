#ifndef POSE_ESTIMATION_HPP
#define POSE_ESTIMATION_HPP


struct Pose {
    double x;
    double y;
    double z;

    double roll;
    double pitch;
    double yaw;
};

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
