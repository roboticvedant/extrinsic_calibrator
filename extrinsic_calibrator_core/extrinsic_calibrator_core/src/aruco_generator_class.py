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

        
