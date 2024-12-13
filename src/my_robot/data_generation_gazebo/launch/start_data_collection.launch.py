from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # AppendEnvironmentVariable(
            #     name='GAZEBO_MODEL_PATH',
            #     value=PathJoinSubstitution([FindPackageShare('data_generation_gazebo'), 'models'])
            # ),
            Node(
                package="data_generation_gazebo",
                executable="model_spawner_image.py",
                output="screen",
            ),
        ]
    )
