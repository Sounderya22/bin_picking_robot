#include "object_manipulation.hpp"
#include <iostream>

ObjectManipulationManager::ObjectManipulationManager(PerceptionSystem& perceptionSystem, MotionPlanner& motionPlanner)
    : perceptionSystem_(perceptionSystem), motionPlanner_(motionPlanner) {}

void ObjectManipulationManager::manipulateObject() {
}
