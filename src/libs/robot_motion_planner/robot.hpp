#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <vector>

struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

// Robot class
class Robot {
public:
    // Constructor
    Robot();

    // Destructor
    ~Robot();

    // Method to move the robot to a specified position
    bool moveToPosition(const Pose position);

    // Method to pick an object
    bool pickObject(const Pose position);

    // Method to place an object
    bool placeObject(const Pose position);

private:
    // Robot components
    std::string robotArm_;      // Type set as string for now
    std::string gripper_;
};



#endif