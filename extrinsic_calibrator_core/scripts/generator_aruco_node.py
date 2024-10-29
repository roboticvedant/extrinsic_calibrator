#!/usr/bin/env python3

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

# ROS-specific imports
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Custom parameters
from extrinsic_calibrator_core.src.aruco_generator_class import ArucoMarkerGenerator


def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    generator_node = ArucoMarkerGenerator()

    # Create executor and add nodes
    executor = MultiThreadedExecutor()
    executor.add_node(generator_node)

    try:
        # Run the nodes within the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        generator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()