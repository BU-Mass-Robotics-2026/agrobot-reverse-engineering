from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="epos2_bridge",
            executable="epos2_arm_controller",
            name="epos2_arm_controller",
            output="screen",
            parameters=[
                "/home/robotics-club/agrobot_ws/src/epos2_bridge/config/epos2_arm_controller.yaml"
            ],
        )
    ])
