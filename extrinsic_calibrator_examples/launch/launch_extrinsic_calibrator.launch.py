# ------------------------------------------------------------------------------
# This file is part of **extrinsic-calibrator:
# October 2024
# Copyright 2024 IKERLAN. All Rights Reserved.
#
#
# LICENSE NOTICE
#
# This software is available under a dual license system. Choose between:
# - GNU Affero General Public License v3.0 for open-source usage, or
# - A commercial license for proprietary development.
# For commercial license details, contact us at info@ikerlan.es.
#
# GNU Affero General Public License v3.0
# Version 3, 19 November 2007
# © 2007 Free Software Foundation, Inc. <https://fsf.org/>
#
# Licensed under a dual license system:
# 1. Open-source usage under the GNU Affero General Public License v3.0
#    (AGPL-3.0), allowing you to freely use, modify, and distribute the
#    software for open-source projects. You can find a copy of the AGPL-3.0
#    license at https://www.gnu.org/licenses/agpl-3.0.html.
# 2. For commercial/proprietary use, a separate commercial license is required.
#    Please contact us at info@ikerlan.es for inquiries about our commercial
#    licensing options.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Author Information:
# Author: Josep Rueda Collell
# Created: October 2024
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
