from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def include_joint(pkg_share, jid, enabled_cfg):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", f"epos2_{jid}_generic.launch.py")
        ),
        condition=IfCondition(enabled_cfg),
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("epos2_bridge")

    enable_j2 = LaunchConfiguration("enable_j2")
    enable_j3 = LaunchConfiguration("enable_j3")
    enable_j4 = LaunchConfiguration("enable_j4")
    enable_j5 = LaunchConfiguration("enable_j5")
    enable_j6 = LaunchConfiguration("enable_j6")
    enable_fanout = LaunchConfiguration("enable_fanout")

    return LaunchDescription([
        DeclareLaunchArgument("enable_j2", default_value="false"),
        DeclareLaunchArgument("enable_j3", default_value="false"),
        DeclareLaunchArgument("enable_j4", default_value="false"),
        DeclareLaunchArgument("enable_j5", default_value="false"),
        DeclareLaunchArgument("enable_j6", default_value="false"),
        DeclareLaunchArgument("enable_fanout", default_value="false"),

        include_joint(pkg_share, "j2", enable_j2),
        include_joint(pkg_share, "j3", enable_j3),
        include_joint(pkg_share, "j4", enable_j4),
        include_joint(pkg_share, "j5", enable_j5),
        include_joint(pkg_share, "j6", enable_j6),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "epos2_fjt_fanout.launch.py")
            ),
            condition=IfCondition(enable_fanout),
        ),
    ])
