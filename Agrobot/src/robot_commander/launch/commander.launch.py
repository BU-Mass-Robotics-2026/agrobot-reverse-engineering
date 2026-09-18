from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot", package_name="moveit_config").to_moveit_configs()

    commander_node = Node(
        package="robot_commander",
        executable="commander",
        parameters=[moveit_config.to_dict()],
    )
    
    tomato_picker_node = Node(
        package="robot_commander",
        executable="tomato_picker.py",
    )

    # Placeholder camera transform until the camera is calibrated.
    # Frame layout: x = along rail, y = forward (toward wall), z = up.
    # Camera sits at y = 0.20 m from linear_rail_link origin, pointing in +y.
    # roll = -pi/2 rotates camera +z (depth) onto linear_rail_link +y.
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0",
            "--y", "0.2",
            "--z", "0.0",
            "--roll", "-1.5708",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "linear_rail_link",
            "--child-frame-id", "camera_color_optical_frame",
        ],
    )

    return LaunchDescription([
        commander_node,
        tomato_picker_node,
        camera_tf_node,
    ])
