#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <vector>


// Robot class
class Robot {
public:
    // Constructor
    Robot();

    // Destructor
    ~Robot();

    // Method to move the robot to a specified position
    bool moveToPosition(const std::vector<double>& position);

    // Method to pick an object
    bool pickObject();

    // Method to place an object
    bool placeObject(const std::vector<double>& position);

private:
    // Robot components
    std::string robotArm_;      // Type set as string for now
    std::string gripper_;
};



#endif