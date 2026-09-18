from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg = FindPackageShare("moveit_config")

    urdf_file = PathJoinSubstitution([pkg, "config", "my_robot_j3_debug.urdf.xacro"])
    initial_positions = PathJoinSubstitution([pkg, "config", "initial_positions.yaml"])
    controllers_yaml = PathJoinSubstitution([pkg, "config", "ros2_controllers_j3_forward.yaml"])

    robot_description_content = Command([
        "xacro ",
        urdf_file,
        " initial_positions_file:=",
        initial_positions
    ])

    robot_description = {"robot_description": robot_description_content}

    forced_ld_library_path = "/opt/ros/jazzy/lib:" + os.environ.get("LD_LIBRARY_PATH", "")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml],
        additional_env={
            "LD_LIBRARY_PATH": forced_ld_library_path,
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )

    j3_forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "j3_forward_position_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
    ])