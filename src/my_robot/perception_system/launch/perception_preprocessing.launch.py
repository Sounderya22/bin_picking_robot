import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction


def generate_launch_description():

    # Node 1: pointcloud_transformation_node
    pointcloud_transformation_node = Node(
        name="pointcloud_transformation_node",
        package="perception_system",
        executable="pointcloud_transformation_node",
        output="screen",
    )

    # Node 2: vision_pcl_processing_cpp_node (with delay)
    vision_pcl_processing_cpp_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            Node(
                name="vision_pcl_processing",
                package="perception_system",
                executable="vision_pcl_processing",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [pointcloud_transformation_node , 
         vision_pcl_processing_cpp_node
         ]
    )