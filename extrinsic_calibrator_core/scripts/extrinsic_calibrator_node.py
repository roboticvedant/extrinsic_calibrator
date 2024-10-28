#!/usr/bin/env python3

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