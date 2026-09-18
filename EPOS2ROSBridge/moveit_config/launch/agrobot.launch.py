import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config_dir = get_package_share_directory('moveit_config')

    # 1. Boot the core MoveIt stack (RViz, Robot State Publisher, Controller Manager)
    # We disable the flaky auto-spawners so we can handle them manually below.
    moveit_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'demo.launch.py')
        ),
        launch_arguments={'spawn_controllers': 'false'}.items()
    )

    # 2. Spawner: Joint State Broadcaster
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 3. Spawner: Arm Controller
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["armjoints_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 4. The Delay Injection (Wait 4 seconds for CAN bus to initialize)
    delayed_spawners = TimerAction(
        period=4.0,
        actions=[spawn_broadcaster, spawn_arm_controller]
    )

    return LaunchDescription([
        moveit_core,
        delayed_spawners
    ])