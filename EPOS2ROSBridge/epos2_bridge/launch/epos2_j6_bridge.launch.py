from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="epos2_bridge",
            executable="epos2_j3_bridge",
            name="epos2_j6_bridge",
            output="screen",
            parameters=[
                {"drive_node_id": 6},
                {"joint_name": "joint_6"},
                {"encoder_qc_per_motor_rev": 25600.0},
                {"gear_ratio_motor_per_joint_rev": 100.0},
                {"sign": 1.0},
                {"zero_offset_qc": 0.0},
                {"force_ipm_on_startup": False},
                {"enable_on_startup": False},
                {"fault_clear_on_startup": False},
                {"ipm_default_segment_ms": 100},
            ],
            remappings=[
                ("/epos2/j3/clear_fault", "/epos2/j6/clear_fault"),
                ("/epos2/j3/arm_ipm", "/epos2/j6/arm_ipm"),
                ("/epos2/j3/disarm_ipm", "/epos2/j6/disarm_ipm"),
                ("/epos2/j3/move_delta", "/epos2/j6/move_delta"),
                ("/epos2/j3/arm_ipm_now", "/epos2/j6/arm_ipm_now"),
                ("/epos2/j3/disarm_ipm_now", "/epos2/j6/disarm_ipm_now"),
                ("/epos2/j3/test_move_rad", "/epos2/j6/test_move_rad"),
                ("/epos2/j3/joint_target", "/epos2/j6/joint_target"),
                ("/epos2/j3/state_summary", "/epos2/j6/state_summary"),
                ("/epos2/j3/state_raw", "/epos2/j6/state_raw"),
                ("/epos2/j3/state_engineering", "/epos2/j6/state_engineering"),
                ("/epos2/j3/fault", "/epos2/j6/fault"),
            ],
        )
    ])
