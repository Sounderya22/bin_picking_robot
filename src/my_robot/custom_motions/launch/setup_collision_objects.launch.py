import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit2_config").to_moveit_configs()
    # moveit_config = MoveItConfigsBuilder("custom").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="setup_collision_objects",
        package="custom_motions",
        executable="setup_collision_objects",
        output="screen",
        # parameters=[
        #     moveit_config.robot_description,
        #     moveit_config.robot_description_semantic,
        #     moveit_config.robot_description_kinematics,
        #     {'use_sim_time': True},
        # ],
    )

    return LaunchDescription([moveit_cpp_node])
