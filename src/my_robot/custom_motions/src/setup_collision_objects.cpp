
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

/**
 * @brief The main function that starts our program.
 *
 * This function sets up our ROS 2 environment and prepares it for robot
 * control.
 *
 * @param argc The number of input arguments our program receives.
 * @param argv The list of input arguments our program receives.
 * @return int A number indicating if our program finished successfully (0) or
 * not.
 */
int main(int argc, char *argv[]) {
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Creates a node named "plan_around_objects". The node is set up to
  // automatically handle any settings (parameters) we might want to change
  // later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
      "plan_around_objects",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Creates a "logger" that we can use to print out information or error
  // messages as our program runs.
  auto const logger = rclcpp::get_logger("plan_around_objects");

  // This code creates a separate thread, which is like a mini-program running
  // alongside our main program. This thread uses a ROS 2 "executor" to
  // continuously process information about the robot's state. By running this
  // in its own thread, our ROS 2 node can keep getting updates about the robot
  // without interrupting the main flow of our program.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interfaces
  // These interfaces are used to plan and execute movements, set target poses,
  // and perform other motion-related tasks for each respective part of the
  // robot. The use of auto allows the compiler to automatically deduce the type
  // of variable. Source:
  // https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Specify a planning pipeline to be used for further planning
  arm_group_interface.setPlanningPipelineId("ompl");

  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");

  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(1.0);

  // Set a scaling factor for optionally reducing the maximum joint velocity.
  // Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(1.0);

  //  Set a scaling factor for optionally reducing the maximum joint
  //  acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s",
              arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s",
              arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f",
              arm_group_interface.getPlanningTime());

  // Create collision object for the robot to avoid
  auto const collision_object =
      [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Initialize a CollisionObject message
        moveit_msgs::msg::CollisionObject collision_object;

        // Set the frame ID for the collision object
        collision_object.header.frame_id = frame_id;

        // Set the timestamp to the current time
        collision_object.header.stamp = node->now();

        // Assign a unique ID to the collision object
        collision_object.id = "box1";

        // Define the shape of the collision object as a box
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 1.0;  // Width
        primitive.dimensions[primitive.BOX_Y] = 1.50; // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.05; // Height

        // Define the pose (position and orientation) of the box
        geometry_msgs::msg::Pose box_pose;

        // Set the position of the box center
        box_pose.position.x = 0.8;  // meters in x-direction
        box_pose.position.y = 0.0;  // Centered in y-direction
        box_pose.position.z = 0.10; // meters in z-direction

        // Set the orientation of the box (no rotation in this case)
        box_pose.orientation.x = 0.0;
        box_pose.orientation.y = 0.0;
        box_pose.orientation.z = 0.0;
        box_pose.orientation.w = 1.0;

        // Add the shape and pose to the collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);

        // Set the operation to add the object to the planning scene
        collision_object.operation = collision_object.ADD;

        // Log information about the created collision object
        RCLCPP_INFO(logger, "Created collision object: %s",
                    collision_object.id.c_str());

        RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f",
                    primitive.dimensions[primitive.BOX_X],
                    primitive.dimensions[primitive.BOX_Y],
                    primitive.dimensions[primitive.BOX_Z]);

        RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
                    box_pose.position.x, box_pose.position.y,
                    box_pose.position.z);

        // Return the fully defined collision object
        return collision_object;
      }();

  // Create collision object for the robot to avoid
  auto const collision_object_2 =
      [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Initialize a CollisionObject message
        moveit_msgs::msg::CollisionObject collision_object;

        // Set the frame ID for the collision object
        collision_object.header.frame_id = frame_id;

        // Set the timestamp to the current time
        collision_object.header.stamp = node->now();

        // Assign a unique ID to the collision object
        collision_object.id = "box2";

        // Define the shape of the collision object as a box
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 1.0;  // Width
        primitive.dimensions[primitive.BOX_Y] = 1.0;  // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.05; // Height

        // Define the pose (position and orientation) of the box
        geometry_msgs::msg::Pose box_pose;

        // Set the position of the box center
        box_pose.position.x = 0.0;  // meters in x-direction
        box_pose.position.y = 0.9;  // Centered in y-direction
        box_pose.position.z = 0.10; // meters in z-direction

        // Set the orientation of the box (no rotation in this case)
        box_pose.orientation.x = 0.0;
        box_pose.orientation.y = 0.0;
        box_pose.orientation.z = 0.0;
        box_pose.orientation.w = 1.0;

        // Add the shape and pose to the collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);

        // Set the operation to add the object to the planning scene
        collision_object.operation = collision_object.ADD;

        // Log information about the created collision object
        RCLCPP_INFO(logger, "Created collision object: %s",
                    collision_object.id.c_str());

        RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f",
                    primitive.dimensions[primitive.BOX_X],
                    primitive.dimensions[primitive.BOX_Y],
                    primitive.dimensions[primitive.BOX_Z]);

        RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
                    box_pose.position.x, box_pose.position.y,
                    box_pose.position.z);

        // Return the fully defined collision object
        return collision_object;
      }();

  // Create collision object for the robot to avoid
  auto const collision_object_3 =
      [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Initialize a CollisionObject message
        moveit_msgs::msg::CollisionObject collision_object;

        // Set the frame ID for the collision object
        collision_object.header.frame_id = frame_id;

        // Set the timestamp to the current time
        collision_object.header.stamp = node->now();

        // Assign a unique ID to the collision object
        collision_object.id = "box3";

        // Define the shape of the collision object as a box
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 1.0;  // Width
        primitive.dimensions[primitive.BOX_Y] = 1.0;  // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.05; // Height

        // Define the pose (position and orientation) of the box
        geometry_msgs::msg::Pose box_pose;

        // Set the position of the box center
        box_pose.position.x = 0.0;  // meters in x-direction
        box_pose.position.y = -0.9; // Centered in y-direction
        box_pose.position.z = 0.10; // meters in z-direction

        // Set the orientation of the box (no rotation in this case)
        box_pose.orientation.x = 0.0;
        box_pose.orientation.y = 0.0;
        box_pose.orientation.z = 0.0;
        box_pose.orientation.w = 1.0;

        // Add the shape and pose to the collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);

        // Set the operation to add the object to the planning scene
        collision_object.operation = collision_object.ADD;

        // Log information about the created collision object
        RCLCPP_INFO(logger, "Created collision object: %s",
                    collision_object.id.c_str());

        RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f",
                    primitive.dimensions[primitive.BOX_X],
                    primitive.dimensions[primitive.BOX_Y],
                    primitive.dimensions[primitive.BOX_Z]);

        RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
                    box_pose.position.x, box_pose.position.y,
                    box_pose.position.z);

        // Return the fully defined collision object
        return collision_object;
      }();

  // Create collision object for the robot to avoid
  auto const collision_object_4 =
      [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Initialize a CollisionObject message
        moveit_msgs::msg::CollisionObject collision_object;

        // Set the frame ID for the collision object
        collision_object.header.frame_id = frame_id;

        // Set the timestamp to the current time
        collision_object.header.stamp = node->now();

        // Assign a unique ID to the collision object
        collision_object.id = "groundPlane";

        // Define the shape of the collision object as a box
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 2.0;   // Width
        primitive.dimensions[primitive.BOX_Y] = 2.0;   // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.005; // Height

        // Define the pose (position and orientation) of the box
        geometry_msgs::msg::Pose box_pose;

        // Set the position of the box center
        box_pose.position.x = 0.0;   // meters in x-direction
        box_pose.position.y = 0.0;   // Centered in y-direction
        box_pose.position.z = -0.50; // meters in z-direction

        // Set the orientation of the box (no rotation in this case)
        box_pose.orientation.x = 0.0;
        box_pose.orientation.y = 0.0;
        box_pose.orientation.z = 0.0;
        box_pose.orientation.w = 1.0;

        // Add the shape and pose to the collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);

        // Set the operation to add the object to the planning scene
        collision_object.operation = collision_object.ADD;

        // Log information about the created collision object
        RCLCPP_INFO(logger, "Created collision object: %s",
                    collision_object.id.c_str());

        RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f",
                    primitive.dimensions[primitive.BOX_X],
                    primitive.dimensions[primitive.BOX_Y],
                    primitive.dimensions[primitive.BOX_Z]);

        RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
                    box_pose.position.x, box_pose.position.y,
                    box_pose.position.z);

        // Return the fully defined collision object
        return collision_object;
      }();

  // Set up a virtual representation of the robot's environment
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Add an object to this virtual environment that the robot needs to avoid
  // colliding with
  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(collision_object_2);
  planning_scene_interface.applyCollisionObject(collision_object_3);
  planning_scene_interface.applyCollisionObject(collision_object_4);

  // Clean up and shut down the ROS 2 node
  rclcpp::shutdown();

  // Wait for the spinner thread to finish
  spinner.join();

  // Exit the program
  return 0;
}
