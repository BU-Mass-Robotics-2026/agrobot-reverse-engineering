from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("epos2_bridge")
    joint_config = os.path.join(pkg_share, "config", "joints", "j5.yaml")

    return LaunchDescription([
        Node(
            package="epos2_bridge",
            executable="epos2_joint_bridge",
            name="epos2_j5_bridge",
            output="screen",
            parameters=[joint_config],
            remappings=[
                ("/epos2/j3/clear_fault", "/epos2/j5/clear_fault"),
                ("/epos2/j3/arm_ipm", "/epos2/j5/arm_ipm"),
                ("/epos2/j3/disarm_ipm", "/epos2/j5/disarm_ipm"),
                ("/epos2/j3/move_delta", "/epos2/j5/move_delta"),
                ("/epos2/j3/move_absolute", "/epos2/j5/move_absolute"),
                ("/epos2/j3/move_absolute_timed", "/epos2/j5/move_absolute_timed"),
                ("/epos2/j3/arm_ipm_now", "/epos2/j5/arm_ipm_now"),
                ("/epos2/j3/disarm_ipm_now", "/epos2/j5/disarm_ipm_now"),
                ("/epos2/j3/test_move_rad", "/epos2/j5/test_move_rad"),
                ("/epos2/j3/joint_target", "/epos2/j5/joint_target"),
                ("/epos2/j3/reduced_traj", "/epos2/j5/reduced_traj"),
                ("/epos2/j3/adaptive_reduced_traj", "/epos2/j5/adaptive_reduced_traj"),
                ("/epos2/j3/native_pvt", "/epos2/j5/native_pvt"),
                ("/epos2/j3/native_pvt_traj", "/epos2/j5/native_pvt_traj"),
                ("/epos2/j3/state_summary", "/epos2/j5/state_summary"),
                ("/epos2/j3/state_raw", "/epos2/j5/state_raw"),
                ("/epos2/j3/state_engineering", "/epos2/j5/state_engineering"),
                ("/epos2/j3/fault", "/epos2/j5/fault"),
                ("/epos2/j3/follow_joint_trajectory", "/epos2/j5/follow_joint_trajectory"),
                ("/epos2/j3/follow_joint_trajectory/_action/send_goal", "/epos2/j5/follow_joint_trajectory/_action/send_goal"),
                ("/epos2/j3/follow_joint_trajectory/_action/get_result", "/epos2/j5/follow_joint_trajectory/_action/get_result"),
                ("/epos2/j3/follow_joint_trajectory/_action/cancel_goal", "/epos2/j5/follow_joint_trajectory/_action/cancel_goal"),
                ("/epos2/j3/follow_joint_trajectory/_action/feedback", "/epos2/j5/follow_joint_trajectory/_action/feedback"),
                ("/epos2/j3/follow_joint_trajectory/_action/status", "/epos2/j5/follow_joint_trajectory/_action/status"),
            ],
        )
    ])
