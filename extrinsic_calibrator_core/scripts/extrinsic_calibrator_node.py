#!/usr/bin/env python3

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

# ROS-specific imports
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Custom parameters
from extrinsic_calibrator_core.src.extrinsic_calibrator_class import ExtrinsicCalibrator


def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    extrinsic_calibrator_node = ExtrinsicCalibrator()

    # Create executor and add nodes
    executor = MultiThreadedExecutor()
    executor.add_node(extrinsic_calibrator_node)

    try:
        # Run the nodes within the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        extrinsic_calibrator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()