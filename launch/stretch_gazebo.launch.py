import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    # Set Gazebo path enviroment
    robot_path = os.path.join(
        get_package_share_directory('stretch_gazebo'))

    install_dir = get_package_prefix("stretch_gazebo")

    gazebo_models_path = os.path.join(robot_path, 'urdf')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share' + ':' + gazebo_models_path

    # Start Gazebo Classic Simulation 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch/gazebo.launch.py')
        )
    )

    ld.add_action(gazebo)

    #Spawn Robot
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('stretch_gazebo'),
                         'launch/spawn_robot.launch.py')
        )
    )

    ld.add_action(robot_spawn)

    #Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('stretch_gazebo'),
                         'launch/robot_state_publisher.launch.py')
        )
    )

    ld.add_action(robot_state_publisher)

    # Launch Stretch controllers
    stretch_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('stretch_gazebo'),
                         'launch/controllers.launch.py')
        )
    )

    ld.add_action(stretch_controller)

    return ld
