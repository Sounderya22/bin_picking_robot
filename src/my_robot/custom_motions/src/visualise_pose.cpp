#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

class PoseVisualizer : public rclcpp::Node
{
public:
    PoseVisualizer() : Node("pose_visualizer")
    {
        // Initialize MoveIt Visual Tools
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link", "/moveit_visual_tools");
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(this->shared_from_this(), "base_link", "/moveit_visual_tools");
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();

        // Set up Move Group Interface
        // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "manipulator");
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "ur_manipulator", tf_buffer_);

        // Visualize Target Pose
        visualizePose();
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    void visualizePose()
    {

                // Define Euler angles in radians
        float roll = 0.0369361;  // Rotation around X-axis
        float pitch = 0.204994;  // Rotation around Y-axis
        float yaw = 1.40297;     // Rotation around Z-axis
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
        target_pose1.orientation.x = 0.56339;
        target_pose1.orientation.y = -0.257342;
        target_pose1.orientation.z =  0.657969;
        // 0.56339 -0.257342  0.257969
        target_pose1.orientation.w = quaternion.w();
        target_pose1.position.x = quaternion.x();
        target_pose1.position.y = quaternion.y();
        target_pose1.position.z = quaternion.z();
        // // Define a target pose
        // geometry_msgs::msg::Pose target_pose;
        // target_pose.orientation.w = 1.0;
        // target_pose.position.x = 0.5;  // Adjust these values based on your robot's workspace
        // target_pose.position.y = 0.2;
        // target_pose.position.z = 0.3;

        // Publish the pose as an axis marker
        visual_tools_->publishAxisLabeled(target_pose1, "Target Pose");
        visual_tools_->trigger();

        // Set the target pose for MoveIt
        move_group_->setPoseTarget(target_pose1);

        // Plan to the target pose
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful!");
            // Visualize the trajectory
            visual_tools_->publishTrajectoryLine(my_plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup("manipulator"));
            visual_tools_->trigger();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed!");
            visual_tools_->publishText(Eigen::Isometry3d::Identity(), "Planning Failed", rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
            visual_tools_->trigger();
        }
    }

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
