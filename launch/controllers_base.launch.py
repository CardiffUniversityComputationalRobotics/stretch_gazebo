import os
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description

def generate_launch_description():
    ld = LaunchDescription()

    pkg_share_folder = get_package_share_directory("stretch_gazebo")
    config_file_path = os.path.join(pkg_share_folder, 'config', 'stretch.yaml')

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file_path],
        output='screen'
    )
    ld.add_action(controller_manager)

    # Timer delays to ensure the controller manager is up before spawning controllers
    joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[GroupAction(
            [
                generate_load_controller_launch_description(
                    controller_name="joint_state_broadcaster",
                    controller_type="joint_state_broadcaster/JointStateBroadcaster",
                    controller_params_file=config_file_path,
                )
            ]
        )]
    )
    ld.add_action(joint_state_broadcaster)

    base_controller = TimerAction(
        period=4.0,
        actions=[GroupAction(
            [
                generate_load_controller_launch_description(
                    controller_name="stretch_mobile_base_controller",
                    controller_type="diff_drive_controller/DiffDriveController",
                    controller_params_file=config_file_path,
                )
            ]
        )]
    )
    ld.add_action(base_controller)

    return ld