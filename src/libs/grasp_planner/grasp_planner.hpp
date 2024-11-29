#ifndef GRASP_PLANNER_HPP
#define GRASP_PLANNER_HPP

#include <vector>

struct Plan {
    double x;
    double y;
    double z;
};

// Class definition for GraspPlanner
class GraspPlanner {
public:
    // Constructor
    GraspPlanner();

    // Destructor
    ~GraspPlanner();

    // Sends command for grasping
    void grasp();

private:

    // Plans a grasp for a given object
    std::vector<Plan>* graspPlan;
};

#endif // GRASP_PLANNER_HPP
