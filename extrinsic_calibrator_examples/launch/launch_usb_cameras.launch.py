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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'extrinsic_calibrator_examples'
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    d435_config = os.path.join(config_dir, 'd435.yaml')
    l515_config = os.path.join(config_dir, 'l515.yaml')

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='d435_camera',
            namespace='camera_1',
            parameters=[d435_config],
            output='screen'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='l515_camera',
            namespace='camera_2',
            parameters=[l515_config],
            output='screen'
        )
    ])
