from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="canopen_core",
            executable="device_container_node",
            name="device_container_node",
            output="screen",
            parameters=[
                {
                    "bus_config": "/home/robotics-club/agrobot_ws/src/moveit_config/config/bus_j3_ipm.yml",
                    "master_config": "/home/robotics-club/agrobot_ws/src/moveit_config/config/master.dcf",
                    "can_interface_name": "can0",
                }
            ],
        )
    ])