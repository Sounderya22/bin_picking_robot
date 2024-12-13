/*******************************************************************************
 * Copyright (C) 2017 Udacity Inc.
 *
 * This file is part of Robotic Arm: Perception project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#include <perception_system/robot_motion.hpp>

URMotion::URMotion(const rclcpp::NodeOptions &options)
    : Node("ur_motion", options),
      move_group_interface_(
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              this, "ur_manipulator")),
      visual_tools_("base_link") {
  // Initialize MoveIt visual tools
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();

  // Initialize planning scene and move group interface
  planning_scene_interface_ =
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  RCLCPP_INFO(this->get_logger(), "UR Motion Node Initialized");

  // Setup collision objects and other initializations
  setupCollisionObjects();
}

URMotion::~URMotion() {}

void URMotion::setupCollisionObjects() {
  // Define collision objects and add them to the scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_object_list;
  moveit_msgs::msg::CollisionObject dropbox_collision_object;

  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = 0;
  mesh_pose.position.y = 0.7;
  mesh_pose.position.z = 0.605;
  mesh_pose.orientation.w = 0.707;
  mesh_pose.orientation.x = 0;
  mesh_pose.orientation.y = 0;
  mesh_pose.orientation.z = 0.707;

  setupCollisionObject("dropbox", DROPBOX_MESH_PATH, mesh_pose,
                       dropbox_collision_object);

  collision_object_list.push_back(dropbox_collision_object);

  planning_scene_interface_->addCollisionObjects(collision_object_list);

  RCLCPP_INFO(this->get_logger(), "Added collision objects to the world");
}

bool URMotion::setupCollisionObject(
    const std::string &object_id, const std::string &mesh_path,
    const geometry_msgs::msg::Pose &object_pose,
    moveit_msgs::msg::CollisionObject &collision_object) {
  collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
  collision_object.id = object_id;

  shapes::Mesh *m = shapes::createMeshFromResource(mesh_path);
  RCLCPP_DEBUG(this->get_logger(), "%s mesh loaded", object_id.c_str());

  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);

  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);

  collision_object.meshes[0] = object_mesh;
  collision_object.mesh_poses[0] = object_pose;

  collision_object.operation = collision_object.ADD;

  return true;
}

bool URMotion::operateGripper(const bool &close_gripper) {
  // RobotState contains the current position/velocity/acceleration data
  auto gripper_current_state = move_group_interface_->getCurrentState();

  std::vector<double> gripper_joint_positions;

  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group_,
                                                 gripper_joint_positions);

  RCLCPP_DEBUG(this->get_logger(), "No. of joints in eef_group: %zd",
               gripper_joint_positions.size());

  if (close_gripper) {
    gripper_joint_positions[0] =
        0.04; // Adjust based on your robot's gripper configuration
    gripper_joint_positions[1] = 0.04;
  } else {
    gripper_joint_positions[0] = 0.0;
    gripper_joint_positions[1] = 0.0;
  }

  move_group_interface_->setJointValueTarget(gripper_joint_positions);

  rclcpp::sleep_for(std::chrono::milliseconds(1500));

  bool success = (move_group_interface_->move() ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return success;
}

tf2::Quaternion URMotion::rpyToQuaternion(float R, float P, float Y) {
  tf2::Matrix3x3 mat;

  mat.setRPY(R, P, Y);

  tf2::Quaternion quat;

  mat.getRotation(quat);

  return quat;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto ur_motion_node = std::make_shared<URMotion>();

  rclcpp::spin(ur_motion_node);

  rclcpp::shutdown();

  return 0;
}
