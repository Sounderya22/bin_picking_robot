#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

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

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

      // Define Euler angles in radians
    float roll = 0.0330272;  // Rotation around X-axis
    roll = 3.14 +  roll;  // Rotation around X-axis
    float pitch = 0;  // Rotation around Y-axis
    float yaw = 1.40297 ;     // Rotation around Z-axis
     yaw = -1 * (yaw - 1.54);     // Rotation around Z-axis
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
  target_pose1.orientation.z =  quaternion.z();
  target_pose1.orientation.w = quaternion.w();
  target_pose1.position.x = pose_x;
  target_pose1.position.y = pose_y;
  target_pose1.position.z = pose_z + 0.3;
  move_group_arm.setPoseTarget(target_pose1);
  move_group_arm.setGoalPositionTolerance(0.01); // Relax position tolerance
  move_group_arm.setGoalOrientationTolerance(0.1); // Relax orientation tolerance

  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  //* / Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.1;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.07;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Retreat

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





   // Define Euler angles in radians
    roll = 0.106633;  // Rotation around X-axis
    roll = 3.14 +  roll;  // Rotation around X-axis
    pitch = 0;  // Rotation around Y-axis
    yaw = -2.1726 ;     // Rotation around Z-axis
    yaw = -1 * (yaw - 1.54);     // Rotation around Z-axis
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
  target_pose2.orientation.z =  quaternion.z();
  target_pose2.orientation.w = quaternion.w();
  target_pose2.position.x = pose_x;
  target_pose2.position.y = pose_y;
  target_pose2.position.z = pose_z + 0.3;
  move_group_arm.setPoseTarget(target_pose2);
  move_group_arm.setGoalPositionTolerance(0.03); // Relax position tolerance
  move_group_arm.setGoalOrientationTolerance(0.1); // Relax orientation tolerance

   success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  //* / Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  approach_waypoints = {};
  target_pose2.position.z -= 0.1;
  approach_waypoints.push_back(target_pose2);

  target_pose2.position.z -= 0.07;
  approach_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_approach1;


   fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach1);

  move_group_arm.execute(trajectory_approach1);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  retreat_waypoints = {};
  target_pose2.position.z += 0.1;
  retreat_waypoints.push_back(target_pose2);

  target_pose2.position.z += 0.07;
  retreat_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat1;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat1);

  move_group_arm.execute(trajectory_retreat1); 

  rclcpp::shutdown();
  return 0;
}