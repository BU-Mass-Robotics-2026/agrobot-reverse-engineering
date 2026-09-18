from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("epos2_bridge")
    fanout_config = os.path.join(pkg_share, "config", "epos2_fjt_fanout.yaml")

    return LaunchDescription([
        Node(
            package="epos2_bridge",
            executable="epos2_fjt_fanout",
            name="epos2_fjt_fanout",
            output="screen",
            parameters=[fanout_config],
        )
    ])
