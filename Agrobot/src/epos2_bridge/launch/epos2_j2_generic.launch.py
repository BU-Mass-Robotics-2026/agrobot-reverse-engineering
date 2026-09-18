from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("epos2_bridge")
    joint_config = os.path.join(pkg_share, "config", "joints", "j2.yaml")

    return LaunchDescription([
        Node(
            package="epos2_bridge",
            executable="epos2_joint_bridge",
            name="epos2_j2_bridge",
            output="screen",
            parameters=[joint_config],
        )
    ])
