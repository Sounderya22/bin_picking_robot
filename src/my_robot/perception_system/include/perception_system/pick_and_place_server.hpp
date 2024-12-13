#ifndef PERCEPTION_SYSTEM__PICK_AND_PLACE_SERVER_HPP_
#define PERCEPTION_SYSTEM__PICK_AND_PLACE_SERVER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <string>

#include <gazebo_msgs/srv/get_model_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/shape_operations.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class URPickPlace : public rclcpp::Node {
public:
  explicit URPickPlace(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("ur_pick_place_server", options),
        move_group_interface_(
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                this, "ur_manipulator")),
        visual_tools_("base_link") {
    // Initialize MoveIt visual tools
    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();

    // Initialize service clients and other components here
    client_ =
        this->create_client<gazebo_msgs::srv::GetModelState>("get_model_state");

    // Initialize planning scene and move group interface
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    RCLCPP_INFO(this->get_logger(), "UR Pick and Place Server Initialized");
  }

  ~URPickPlace() {}

  bool Routine(const std::shared_ptr<pr2_robot::srv::PickPlace::Request> req,
               std::shared_ptr<pr2_robot::srv::PickPlace::Response> res);

private:
  // Service client for Gazebo model state
  rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedPtr client_;

  // MoveIt components
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  // Helper functions for gripper operation
  bool OperateGripper(const bool &close_gripper);

  bool
  SetupCollisionObject(const std::string &object_id,
                       const std::string &mesh_path,
                       const geometry_msgs::msg::Pose &object_pose,
                       moveit_msgs::msg::CollisionObject &collision_object);

  tf2::Quaternion RPYToQuaternion(float R, float P, float Y);

  bool IsPickPoseWithinLimits(geometry_msgs::msg::Pose &pick_pose,
                              geometry_msgs::msg::Pose &act_obj_pose);
};

#endif // PERCEPTION_SYSTEM__PICK_AND_PLACE_SERVER_HPP_
