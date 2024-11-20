#ifndef OBJECT_MANIPULATION_HPP
#define OBJECT_MANIPULATION_HPP


#include "perception_system/perception_system.hpp"
#include "motion_planner/motion_planner.hpp"

class ObjectManipulationManager {
public:
    // Constructor
    ObjectManipulationManager(PerceptionSystem& perceptionSystem, MotionPlanner& motionPlanner);

    // Method to manipulate an object (pick and place)
    void manipulateObject();

private:
    PerceptionSystem& perceptionSystem_;
    MotionPlanner& motionPlanner_;
};

#endif