#!/usr/bin/env python3

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