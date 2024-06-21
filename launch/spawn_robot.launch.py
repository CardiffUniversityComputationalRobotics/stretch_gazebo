from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    robot_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "stretch",
        ],
        output="screen",
    )

    ld.add_action(robot_entity)

    return ld