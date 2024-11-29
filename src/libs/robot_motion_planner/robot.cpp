#include "robot.hpp"
#include <iostream>

// Constructor
Robot::Robot() {
    std::cout << "Robot initialized.\n";
}


// Destructor
Robot::~Robot() {
    std::cout << "Robot destroyed.\n";
}

// Method to move the robot to a specified position
bool Robot::moveToPosition(const Pose position) {
    return true;
}

// Method to pick an object
bool Robot::pickObject(const Pose position) {
    return true;
}

// Method to place an object
bool Robot::placeObject(const Pose position) {
    return true;
}
