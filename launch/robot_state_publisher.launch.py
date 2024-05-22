# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''
Modified: Samuel Millan_norman, millan-normans@cardiff.ac.uk
'''

import os
from pathlib import Path
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import load_xacro



def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(
        OpaqueFunction(function=create_robot_description_param)
    )

    # Using ParameterValue is needed so ROS knows the parameter type
    # Otherwise https://github.com/ros2/launch_ros/issues/136
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": ParameterValue(
                    LaunchConfiguration("robot_description"), value_type=str
                ),
            }
        ],
    )

    ld.add_action(rsp)

    return ld


def create_robot_description_param(context, *args, **kwargs):

    xacro_file_path = Path(
        os.path.join(
            get_package_share_directory("stretch_gazebo"),
            "urdf",
            "gazebo.urdf.xacro",
        )
    )

    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc)

    robot_description = doc.toxml()

    return [SetLaunchConfiguration("robot_description", robot_description)]