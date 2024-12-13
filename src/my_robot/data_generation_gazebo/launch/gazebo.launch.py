import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments for the robot name and initial positions file
    # Set GAZEBO_MODEL_PATH environment variable dynamically
    # model_path = os.path.join(FindPackageShare('data_generation_gazebo'))
    # model_path_2 = os.path.join(FindPackageShare('data_generation_gazebo'), "models")
    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     os.environ['GAZEBO_MODEL_PATH'] += f":{model_path}:{model_path_2}"
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] = f"{model_path}:{model_path_2}"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name", default_value="my_robot", description="Name of the robot"
            ),
            # General arguments
            DeclareLaunchArgument(
                "description_package",
                default_value="data_generation_gazebo",
                description="Description package with robot URDF/XACRO files. Usually the argument "
                "is not set, it enables use of a custom description.",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value="sensor.urdf.xacro",
                description="URDF/XACRO description file with the robot.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
            ),
            # AppendEnvironmentVariable(
            # 'GAZEBO_MODEL_PATH',
            # PathJoinSubstitution([FindPackageShare(LaunchConfiguration('description_package')), "models"])),
            # AppendEnvironmentVariable(
            # 'GAZEBO_MODEL_PATH',
            # PathJoinSubstitution([FindPackageShare(LaunchConfiguration('description_package')), "meshes"])),
            # Process the xacro file to generate URDF
            ExecuteProcess(
                cmd=[
                    "xacro",
                    PathJoinSubstitution(
                        [
                            FindPackageShare(LaunchConfiguration("description_package")),
                            "urdf",
                            LaunchConfiguration("description_file"),
                        ]
                    ),
                ],
                output="screen",
            ),
            # Publish robot state to TF
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare(
                                            LaunchConfiguration("description_package")
                                        ),
                                        "urdf",
                                        LaunchConfiguration("description_file"),
                                    ]
                                ),
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_sensor",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    LaunchConfiguration("robot_name"),
                ],
                output="screen",
            ),
            # Gazebo nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gazebo_gui"),
                    "verbose": "true",
                }.items(),
            ),
            # Spawn robot
        ]
    )
