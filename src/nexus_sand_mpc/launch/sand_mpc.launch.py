import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nexus_sand_mpc")
    return LaunchDescription(
        [
            Node(
                package="nexus_sand_mpc",
                executable="sand_mpc_compensator",
                name="sand_mpc_compensator",
                output="screen",
                parameters=[os.path.join(pkg_share, "config", "sand_mpc.yaml")],
            )
        ]
    )
