# Stretch Gazebo
This package allows migrates the stretch_gazebo package found in (https://github.com/hello-robot/stretch_ros.git) to ROS2. All xacro and urdf files have been obtained from the 'stretch_ros' package and modified. This package has been created for simplification of the meta-package `stretch_ros2`, once the code is functional it will be incorporated to the meta-package found in (https://github.com/hello-robot/stretch_ros2.git)

> [NOTE]: The robot is partially functional excluding the arm and gripper. The current urdf incorporates the mobile base, head, speakers and sensors.

> [UPDATE]: New branch has been created called `dev` for debugging purposes. Currently the migration of the arm and gripper is under development.
# Install Package
To have functional the following simulation first install the create a catkin workspace and install all dependecies mentioned below.

## APT Dependecies
Dependecies you can install from terminal
```
sudo apt-get install ros-humble-launch-param-builder
sudo apt-get install ros-humble-gazebo-ros
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-controller-manager
sudo apt-get install ros-humble-ament-cmake
sudo apt-get install ros-humble-control-msgs
sudo apt-get install ros-humble-diagnostic-updater
sudo apt-get install ros-humble-control-toolbox
sudo apt install python3-pip
pip install setuptools==58.2.0
sudo apt-get install ros-humble-joint-state-broadcaster
sudo apt-get install ros-humble-diff-drive-controller
sudo apt-get install ros-humble-joint-trajectory-controller
```
## Dependecies from Source
Dependecies which should be in the `src` folder of your catkin workspace.
```
git clone -b humble git@github.com:ros-controls/ros2_control.git
git clone -b humble git@github.com:ros-controls/gazebo_ros2_control.git
git clone -b foxy-devel git@github.com:pal-robotics/realsense_gazebo_plugin.git
```

# Start Simulation
After you have built your catkin workspace make sure to add the following lines to the `bashrc` file.
```
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/samuel/ros2_ws/install/stretch_gazebo/share/stretch_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib
```
Then, source your bashrc and your catkin workspace. Once you have followed all previous steps mentioned above you will be ready to launch the simulation by running the following command:
```
ros2 run stretch_gazebo stretch_gazebo.launch.py
```