from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    epos2_share = get_package_share_directory("epos2_bridge")
    moveit_config_share = get_package_share_directory("moveit_config")

    enable_j1 = LaunchConfiguration("enable_j1")
    enable_j2 = LaunchConfiguration("enable_j2")
    enable_j3 = LaunchConfiguration("enable_j3")
    enable_j4 = LaunchConfiguration("enable_j4")
    enable_j5 = LaunchConfiguration("enable_j5")
    enable_j6 = LaunchConfiguration("enable_j6")
    apply_live_remap = LaunchConfiguration("apply_live_remap")
    remap_node_ids = LaunchConfiguration("remap_node_ids")

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "j3_canopen_driver_only_ipm.launch.py")
        )
    )

    remap_proc = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            ["source /opt/ros/jazzy/setup.bash && "
             "source ~/agrobot_ws/install/setup.bash && "
             "~/agrobot_ws/scripts/apply_ipm_pdo_remap_all.sh ", remap_node_ids]
        ],
        output="screen",
        condition=IfCondition(apply_live_remap),
    )

    def inc(name, cond):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(epos2_share, "launch", f"epos2_{name}_bridge.launch.py")),
            condition=IfCondition(cond),
        )

    return LaunchDescription([
        DeclareLaunchArgument("enable_j1", default_value="false"),
        DeclareLaunchArgument("enable_j2", default_value="false"),
        DeclareLaunchArgument("enable_j3", default_value="true"),
        DeclareLaunchArgument("enable_j4", default_value="false"),
        DeclareLaunchArgument("enable_j5", default_value="false"),
        DeclareLaunchArgument("enable_j6", default_value="false"),
        DeclareLaunchArgument("apply_live_remap", default_value="true"),
        DeclareLaunchArgument("remap_node_ids", default_value="1"),

        driver_launch,
        TimerAction(period=3.0, actions=[remap_proc]),
        TimerAction(period=6.0, actions=[inc("j1", enable_j1)]),
        TimerAction(period=6.5, actions=[inc("j2", enable_j2)]),
        TimerAction(period=7.0, actions=[inc("j3", enable_j3)]),
        TimerAction(period=7.5, actions=[inc("j4", enable_j4)]),
        TimerAction(period=8.0, actions=[inc("j5", enable_j5)]),
        TimerAction(period=8.5, actions=[inc("j6", enable_j6)]),
    ])
