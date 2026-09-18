from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config_share = get_package_share_directory("moveit_config")

    apply_live_remap = LaunchConfiguration("apply_live_remap")

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_config_share,
                "launch",
                "j3_canopen_driver_only_ipm.launch.py",
            )
        )
    )

    remap_proc = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && "
            "source ~/agrobot_ws/install/setup.bash && "
            "~/agrobot_ws/scripts/apply_ipm_pdo_remap.sh"
        ],
        output="screen",
        condition=IfCondition(apply_live_remap),
    )

    bridge_node = Node(
        package="epos2_bridge",
        executable="epos2_j3_bridge",
        name="epos2_j3_bridge",
        output="screen",
        parameters=[
            {"force_ipm_on_startup": False},
            {"enable_on_startup": False},
            {"fault_clear_on_startup": False},
            {"encoder_qc_per_motor_rev": 25600.0},
            {"gear_ratio_motor_per_joint_rev": 1.0},
            {"sign": 1.0},
            {"ipm_default_segment_ms": 100},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "apply_live_remap",
            default_value="true",
            description="Apply the working live PDO remap before starting the bridge",
        ),
        driver_launch,
        TimerAction(period=3.0, actions=[remap_proc]),
        TimerAction(period=6.0, actions=[bridge_node]),
    ])
