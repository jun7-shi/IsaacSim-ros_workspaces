import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("humanoid_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    package_dir, "rviz", "humanoid_navigation.rviz"
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
