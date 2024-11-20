#ifndef PERCEPTION_SYSTEM_HPP
#define PERCEPTION_SYSTEM_HPP

#include "object_detection/object_detection.hpp"
#include "pose_estimation/pose_estimation.hpp"

class PerceptionSystem {
public:
    // Constructor
    PerceptionSystem(ObjectDetection& ObjectDetection, PoseEstimation& PoseEstimation);

    // Destructor
    ~PerceptionSystem();

    // Detect objects using the camera and object detection module
    void detectObjects();

    // Estimate poses of detected objects
    void estimatePose();

private:
    // Camera instance for capturing data
    std::string camera;
    ObjectDetection& objectDetection_;
    PoseEstimation& poseEstimation_;
};

#endif // PERCEPTION_SYSTEM_HPP
