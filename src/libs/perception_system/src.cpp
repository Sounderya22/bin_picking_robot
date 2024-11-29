#include "perception_system.hpp"
#include <iostream>

// Constructor
PerceptionSystem::PerceptionSystem(ObjectDetection& ObjectDetection, PoseEstimation& PoseEstimation) : objectDetection_(ObjectDetection), poseEstimation_(PoseEstimation){
    std::cout << "Perception System initialized\n";
}

// Destructor
PerceptionSystem::~PerceptionSystem() {
}

// Detect objects using the camera and object detection module
void PerceptionSystem::detectObjects() {
    
}

// Estimate poses of detected objects
void PerceptionSystem::estimatePose() {
   
}
