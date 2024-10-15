# extrinsic_calibrator
`extrinsic_calibrator` is a ROS2 package designed to extrinsically calibrate a set of cameras distributed throughout a room. The calibration is performed using ArUco markers "randomly" scattered through the environment. Each camera detects one or several ArUco markers within its field of view and the algorithm reconstructs the positions of the markers to create a global map. The positions of the cameras are then computed and incorporated into this aforementioned map map.

This package is composed of two submodules:
1. [extrinsic_calibrator_core](https://github.com/pep248/extrinsic_calibrator_core) with the extrinsic calibrator itself and an ArUco generator to simplify the usage.
2. [extrinsic_calibrator_examples](https://github.com/pep248/extrinsic_calibrator_examples) with some useful launch files to easily set up your cameras as well as a rviz visualizer node.

## Author Information

**Author:** Josep Rueda Collell  
**Created:** October 2024  
**Email:** [rueda_999@hotmail.com](mailto:rueda_999@hotmail.com)  

### Citation
If you use this code, please cite:  
**Josep Rueda Collell**. "ROS2 Extrinsic Camera Calibrator using ArUco Markers". (2024).

This work is licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).  
See the [LICENSE](https://github.com/pep248/extrinsic_calibrator/blob/main/LICENSE) file for more details.
