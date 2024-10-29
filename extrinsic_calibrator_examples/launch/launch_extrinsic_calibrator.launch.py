# ------------------------------------------------------------------------------
# Copyright 2024 IKERLAN. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author Information:
# Author: Josep Rueda Collell
# Created: October 2024
# Email: rueda_999@hotmail.com
# Affiliation: IKERLAN (https://www.ikerlan.es)
# ------------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        # Launch the extrinsic calibrator node. The config file is in the config folder and is passed to the node using the generate_parameter_library
        Node(
            package='extrinsic_calibrator_core',
            executable='extrinsic_calibrator_node.py',
            name='extrinsic_calibrator_node',
            output='screen',
        ), 
        
        # Laucnh the set of usb-cameras with their own config_files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("extrinsic_calibrator_examples"),
                "launch",
                "launch_usb_cameras.launch.py"]))
        ),
        
        # Laucnh the rviz visualizer with the TF of the map and the cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("extrinsic_calibrator_examples"),
                "launch",
                "launch_rviz.launch.py"]))
        ),
    ])
