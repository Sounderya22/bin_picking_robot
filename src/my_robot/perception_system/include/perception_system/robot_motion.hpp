#ifndef UR_ROBOT_MOTION_H
#define UR_ROBOT_MOTION_H

#include <string>
#include <sstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/shape_operations.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class URMotion : public rclcpp::Node
{
public:
  explicit URMotion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ur_motion", options),
      move_group_interface_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "ur_manipulator")),
      visual_tools_("base_link")
  {
    // Initialize MoveIt visual tools
    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();

    // Initialize planning scene and move group interface
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    RCLCPP_INFO(this->get_logger(), "UR Motion Node Initialized");
  }

  ~URMotion() {}

private:
  // MoveIt components
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  // Helper functions for gripper operation
  bool OperateGripper(const bool &close_gripper);

  bool SetupCollisionObject(const std::string &object_id,
                            const std::string &mesh_path,
                            const geometry_msgs::msg::Pose &object_pose,
                            moveit_msgs::msg::CollisionObject &collision_object);

  tf2::Quaternion RPYToQuaternion(float R, float P, float Y);
};

#endif  // UR_ROBOT_MOTION_H