#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  move_group_arm.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  std::vector<double> joint_group_positions_arm;

  for (int i = 0; i < 6; i++) {
    joint_group_positions_arm.push_back(0.0);
  }

  // Go Home
  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.54; // Shoulder Lift
  joint_group_positions_arm[2] = 0;     // Elbow
  joint_group_positions_arm[3] = 0;     // Wrist 1
  joint_group_positions_arm[4] = 0;     // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  // Define Euler angles in radians
  float roll = 0.0330272;  // Rotation around X-axis
  roll = 3.14 + roll;      // Rotation around X-axis
  float pitch = 0;         // Rotation around Y-axis
  float yaw = 1.40297;     // Rotation around Z-axis
  yaw = -1 * (yaw - 1.54); // Rotation around Z-axis
  float pose_x = 0.611773;
  float pose_y = -0.208666;
  float pose_z = 0.258726;
  // Create a quaternion from Euler angles (roll, pitch, yaw)
  Eigen::Quaternionf quaternion;
  quaternion = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  // Normalize the quaternion (optional but recommended)
  quaternion.normalize();
  // Output the quaternion components
  std::cout << "Quaternion:\n";
  std::cout << "w: " << quaternion.w() << "\n";
  std::cout << "x: " << quaternion.x() << "\n";
  std::cout << "y: " << quaternion.y() << "\n";
  std::cout << "z: " << quaternion.z() << "\n";

  geometry_msgs::msg::Pose target_pose1;

  target_pose1.orientation.x = quaternion.x();
  target_pose1.orientation.y = quaternion.y();
  target_pose1.orientation.z = quaternion.z();
  target_pose1.orientation.w = quaternion.w();
  target_pose1.position.x = pose_x;
  target_pose1.position.y = pose_y;
  target_pose1.position.z = pose_z + 0.3;
  move_group_arm.setPoseTarget(target_pose1);
  move_group_arm.setGoalPositionTolerance(0.01); // Relax position tolerance
  move_group_arm.setGoalOrientationTolerance(
      0.1); // Relax orientation tolerance

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  //* / Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.1;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Retreat

  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 2 seconds

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.1;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.1;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 2 seconds

  //   target_pose1.position.x = 0;
  //   target_pose1.position.y = -0.71;
  //   target_pose1.position.z = 0.605 + 0.2;
  //   target_pose1.orientation.x = 0.0;
  //   target_pose1.orientation.y = 0.0;
  //   target_pose1.orientation.z =  0.710;
  //   target_pose1.orientation.w = 0.71;
  //   move_group_arm.setPoseTarget(target_pose1);
  //   success_arm = (move_group_arm.plan(my_plan_arm) ==
  //                  moveit::core::MoveItErrorCode::SUCCESS);
  //   move_group_arm.execute(my_plan_arm);

  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Going Green");

  joint_group_positions_arm[0] = -1.80; // Shoulder Pan
  joint_group_positions_arm[1] = -1.07; // Shoulder Lift
  joint_group_positions_arm[2] = 0.765; // Elbow
  joint_group_positions_arm[3] = -1.43; // Wrist 1
  joint_group_positions_arm[4] = -1.72; // Wrist 2
  joint_group_positions_arm[5] = -0.30; // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 2 seconds
  // position: {x: 0.0, y: 0.20, z: 1.0},
  // orientation: {x: 0.071, y: 0.0, z: 0.0, w: 0.71}

  //   target_pose1.position.x = 0.0;
  //   target_pose1.position.y = 0.20;
  //   target_pose1.position.z = 0.80;
  //   target_pose1.orientation.x = 0.0;
  //   target_pose1.orientation.y = 0.0;
  //   target_pose1.orientation.z =  0.710;
  //   target_pose1.orientation.w = 0.71;
  //   move_group_arm.setPoseTarget(target_pose1);
  //   success_arm = (move_group_arm.plan(my_plan_arm) ==
  //                  moveit::core::MoveItErrorCode::SUCCESS);
  //   move_group_arm.execute(my_plan_arm);

  // Go Home
  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.54; // Shoulder Lift
  joint_group_positions_arm[2] = 0;     // Elbow
  joint_group_positions_arm[3] = 0;     // Wrist 1
  joint_group_positions_arm[4] = 0;     // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 2 seconds

  // Define Euler angles in radians
  roll = 0.106633;         // Rotation around X-axis
  roll = 3.14 + roll;      // Rotation around X-axis
  pitch = 0;               // Rotation around Y-axis
  yaw = -2.1726;           // Rotation around Z-axis
  yaw = -1 * (yaw - 1.54); // Rotation around Z-axis
  pose_x = 0.637112;
  pose_y = 0.340101;
  pose_z = 0.213261;
  // Create a quaternion from Euler angles (roll, pitch, yaw)
  // Eigen::Quaternionf /* quaternion */;
  quaternion = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  // Normalize the quaternion (optional but recommended)
  quaternion.normalize();
  // Output the quaternion components
  std::cout << "Quaternion:\n";
  std::cout << "w: " << quaternion.w() << "\n";
  std::cout << "x: " << quaternion.x() << "\n";
  std::cout << "y: " << quaternion.y() << "\n";
  std::cout << "z: " << quaternion.z() << "\n";

  geometry_msgs::msg::Pose target_pose2;

  target_pose2.orientation.x = quaternion.x();
  target_pose2.orientation.y = quaternion.y();
  target_pose2.orientation.z = quaternion.z();
  target_pose2.orientation.w = quaternion.w();
  target_pose2.position.x = pose_x;
  target_pose2.position.y = pose_y;
  target_pose2.position.z = pose_z + 0.3;
  move_group_arm.setPoseTarget(target_pose2);
  move_group_arm.setGoalPositionTolerance(0.03); // Relax position tolerance
  move_group_arm.setGoalOrientationTolerance(
      0.1); // Relax orientation tolerance

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  //* / Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  approach_waypoints = {};
  target_pose2.position.z -= 0.1;
  approach_waypoints.push_back(target_pose2);

  target_pose2.position.z -= 0.02;
  approach_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_approach1;

  fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach1);

  move_group_arm.execute(trajectory_approach1);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");
  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 2 seconds

  retreat_waypoints = {};
  target_pose2.position.z += 0.1;
  retreat_waypoints.push_back(target_pose2);

  target_pose2.position.z += 0.07;
  retreat_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat1;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat1);

  move_group_arm.execute(trajectory_retreat1);

  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 2 seconds

  //   target_pose2.position.x = 0;
  //   target_pose2.position.y = 0.71;
  //   target_pose2.position.z = 0.605 + 0.2;
  //   target_pose2.orientation.x = 0.0;
  //   target_pose2.orientation.y = 0.0;
  //   target_pose2.orientation.z = 0.710;
  //   target_pose2.orientation.w = 0.71;
  //   move_group_arm.setPoseTarget(target_pose2);
  //    success_arm = (move_group_arm.plan(my_plan_arm) ==
  //                  moveit::core::MoveItErrorCode::SUCCESS);

  //   move_group_arm.execute(my_plan_arm);

  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Going Red");

  joint_group_positions_arm[0] = 1.49;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.07; // Shoulder Lift
  joint_group_positions_arm[2] = 0.765; // Elbow
  joint_group_positions_arm[3] = -1.43; // Wrist 1
  joint_group_positions_arm[4] = -1.72; // Wrist 2
  joint_group_positions_arm[5] = -0.30; // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 2 seconds
  // position: {x: 0.0, y: 0.20, z: 1.0},
  // orientation: {x: 0.071, y: 0.0, z: 0.0, w: 0.71}

  //   target_pose1.position.x = 0.0;
  //   target_pose1.position.y = 0.20;
  //   target_pose1.position.z = 0.80;
  //   target_pose1.orientation.x = 0.0;
  //   target_pose1.orientation.y = 0.0;
  //   target_pose1.orientation.z =  0.710;
  //   target_pose1.orientation.w = 0.71;
  //   move_group_arm.setPoseTarget(target_pose1);
  //   success_arm = (move_group_arm.plan(my_plan_arm) ==
  //                  moveit::core::MoveItErrorCode::SUCCESS);
  //   move_group_arm.execute(my_plan_arm);

  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.54; // Shoulder Lift
  joint_group_positions_arm[2] = 0;     // Elbow
  joint_group_positions_arm[3] = 0;     // Wrist 1
  joint_group_positions_arm[4] = 0;     // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  rclcpp::shutdown();
  return 0;
}
