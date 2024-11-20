#include <gtest/gtest.h>
#include "motion_planner/motion_planner.hpp"
#include "object_detection/object_detection.hpp"
#include "object_manipulation/object_manipulation.hpp"
#include "perception_system/perception_system.hpp"
#include "pose_estimation/pose_estimation.hpp"
#include "robot_motion_planner/robot_motion_planner.hpp"
#include "sensor_manager/sensor_manager.hpp"


// Define operator== for Trajectory
bool operator==(const Trajectory& lhs, const Trajectory& rhs) {
    return lhs.duration == rhs.duration &&
           lhs.positions == rhs.positions &&
           lhs.velocities == rhs.velocities;
}

bool operator==(const Pose& lhs, const Pose& rhs) {
    return lhs.x == rhs.x &&
           lhs.y == rhs.y &&
           lhs.z == rhs.z &&
           lhs.roll == rhs.roll &&
           lhs.pitch == rhs.pitch &&
           lhs.yaw == rhs.yaw;
}
// Test class for class RobotMotionPlanner and Robot
class RobotMotionPlannerTest : public testing::Test {
  protected:
    Robot robot;
    RobotMotionPlanner test_planner;
    Pose test_pos;
    Trajectory test_trajectory;

    RobotMotionPlannerTest() : test_planner(robot) {};

    void SetUp() override {
      // Initialized to all zeros for now. Will be changed later
      test_pos.x = 0;
      test_pos.y = 0;
      test_pos.z = 0;
      test_pos.roll = 0;
      test_pos.yaw = 0;
      test_pos.pitch = 0;

      test_trajectory.duration = 5;
      test_trajectory.positions = {0.0, 0.0};
      test_trajectory.velocities = {0.0, 0.0};
    }
};

// Test case to check if pickObject works correctly
TEST_F(RobotMotionPlannerTest, PickWorks) {
    // Call pickObject with test position and check if it returns true
    ASSERT_TRUE(robot.pickObject(test_pos));
}

// Test case to check if moveToPosition works correctly
TEST_F(RobotMotionPlannerTest, MoveToPositionWorks) {
    // Call moveToPosition with test position and check if it returns true
    ASSERT_TRUE(robot.moveToPosition(test_pos));
}

// Test case to check if placeObject works correctly
TEST_F(RobotMotionPlannerTest, PlaceObjectWorks) {
    // Call placeObject with test position and check if it returns true
    ASSERT_TRUE(robot.placeObject(test_pos));
}

// Test case to check if ExecuteTrajectory() of RobotMotionPlanner works correctly
TEST_F(RobotMotionPlannerTest, ExecuteTrajectoryWorks) {
  // Call executeTrajectory with test trajectory and check if it returns true
  ASSERT_TRUE(test_planner.executeTrajectory(test_trajectory));
}

// Test class for class MotionPlanner
class MotionPlannerTest : public testing::Test {
  protected:
    RobotMotionPlanner robot_motion_planner;
    Robot robot;
    Trajectory test_trajectory;
    MotionPlanner motion_planner;
    std::vector<double> test_start;
    std::vector<double> test_goal;

    MotionPlannerTest() : robot_motion_planner(robot), motion_planner(robot_motion_planner) {};

    void SetUp() override {
      test_start = {0.0, 0.0};
      test_goal = {0.0, 0.0};
    }
};

// Test case to check if ComputeTrajectory works correctly
TEST_F(MotionPlannerTest, ComputeTrajectoryWorks) {
  EXPECT_EQ(motion_planner.computeTrajectory(test_start, test_goal), test_trajectory);
}

// Test class for SensorManager
class SensorManagerTest : public ::testing::Test {
protected:
    SensorManager test_sensor_manager;
    PerceptionSystem test_perception_system;
    PoseEstimation pose_estimation;
    ObjectDetection object_detection;

    SensorManagerTest() : test_perception_system(object_detection, pose_estimation), test_sensor_manager(test_perception_system) {};
};

// Test case for getSensorData
TEST_F(SensorManagerTest, GetSensorDataWorks) {
    // Expected output
    std::vector<std::string> expectedSensorData = {"Sensor1"};

    // Get actual sensor data
    auto sensorData = test_sensor_manager.getSensorData();

    // Check if the retrieved sensor data matches the expected data
    EXPECT_EQ(sensorData, expectedSensorData);
}

// Test for PoseEstimation class method, estimateFrom3DData()
TEST(PoseEstimationTest, estimateFrom3DDataWorks) {
  PoseEstimation test_estimate;
  Pose test_pose;
  test_pose.x = 0;
  test_pose.y = 0;
  test_pose.z = 0;
  test_pose.pitch = 0;
  test_pose.roll = 0;
  test_pose.yaw = 0;
  EXPECT_EQ(test_estimate.estimateFrom3DData(), test_pose);
}