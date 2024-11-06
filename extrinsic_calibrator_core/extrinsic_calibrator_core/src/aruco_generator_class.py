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

# Standard libraries
import os

# Well-known libraries
import cv2

# ROS-specific imports
import rclpy
from rclpy.node import Node

# Custom parameters
from extrinsic_calibrator_core.python_aruco_parameters import aruco_params
from extrinsic_calibrator_core.src.extrinsic_calibrator_class import ArucoParams



class ArucoMarkerGenerator(Node):
    def __init__(self):
        super().__init__('aruco_marker_generator')
        self.get_logger().info("Aruco Marker Generator Node has started.")
        
        aruco_params_listener = aruco_params.ParamListener(self)
        imported_aruco_params = aruco_params_listener.get_params()
        self.real_aruco_params = ArucoParams(self,imported_aruco_params)
        
        # Parameters for marker generation
        self.declare_parameter('marker_size', 200)
        self.declare_parameter('output_directory', '~/markers')

        marker_size = self.get_parameter('marker_size').value
        output_directory = os.path.expanduser(self.get_parameter('output_directory').value)

        # Check if output directory exists, create if it doesn't
        if not os.path.exists(output_directory):
            os.makedirs(output_directory)
            self.get_logger().info(f"Created output directory: {output_directory}")
        else:
            self.get_logger().info(f"Output directory: {output_directory} already exists")
        
        # Generate all markers from aruco_dict
        num_markers = len(self.real_aruco_params.aruco_dict.bytesList)
        for marker_id in range(0, num_markers):
            if not self.generate_marker(marker_id, marker_size, output_directory, self.real_aruco_params):
                self.get_logger().info("ArUco generation failed")
                return False
        
        self.get_logger().info("ArUco generation finished successfully. Hit Ctrl+C to exit")
            
        while(1):
            pass    
            
        self.destroy_node()
        rclpy.shutdown()
           
            
    def generate_marker(self, marker_id, marker_size, output_directory, aruco_params:ArucoParams):
        # Generate the marker image
        marker_image = cv2.aruco.generateImageMarker(aruco_params.aruco_dict, marker_id, marker_size)

        # Rotate the image 180 degrees
        rotated_marker = cv2.rotate(marker_image, cv2.ROTATE_180)

        # Save the rotated marker image to the specified directory
        output_path = os.path.join(output_directory, f'aruco_marker_{marker_id}.png')
        cv2.imwrite(output_path, rotated_marker)
        # self.get_logger().info(f'Marker with ID {marker_id} generated and saved to {output_path}')
        return True

        
