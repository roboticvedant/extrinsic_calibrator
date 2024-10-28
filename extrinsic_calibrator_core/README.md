# extrinsic_calibrator_core

## Overview

`extrinsic_calibrator_core` is a ROS2 package designed to calibrate a set of cameras distributed throughout a room. The calibration is performed using ArUco markers scattered randomly in the environment. Each camera detects one or several ArUco markers within its field of view, and the algorithm reconstructs the positions of the markers to create a global map. The positions of the cameras are then computed and incorporated into this aforementioned map map.

The algorithm utilizes the OpenCV library to detect the markers and performs matrix transformations to compute the positions of both markers and cameras relative to each other.

## Features

- Extrinsically calibrate any number of cameras simultaneously.
- Automatically build a global ArUco map using marker detection from multiple viewpoints.
- Configurable ArUco marker properties and camera topics.
- Includes an utility for generating printable ArUco markers.

## Configuration

The package provides configuration options through YAML files.

### ArUco Marker Parameters

You can customize the ArUco markers used in the calibration process by modifying the `aruco_parameters.yaml` file.

```yaml
aruco_params:
  aruco_dict: # OpenCV marker dictionary
    type: string
    default_value: "DICT_6X6_250"
  marker_length: # Length of the marker side in meters
    type: double
    default_value: 0.26
```

### Camera Topics Parameters

You can specify the topics for each camera in the `camera_topics_parameters.yaml` file. This setup is scalable to handle as many cameras as needed.
```yaml
cameras_params:
  cam1:
    image_topic:
      type: string
      default_value: "/camera_1/image_raw"
    camera_info_topic:
      type: string
      default_value: "/camera_1/camera_info"
  cam2:
    image_topic:
      type: string
      default_value: "/camera_2/image_raw"
    camera_info_topic:
      type: string
      default_value: "/camera_2/camera_info"
  # cam3:
  #   image_topic:
  #     type: string
  #     default_value: "/camera_3/image_raw"
  #   camera_info_topic:
  #     type: string
  #     default_value: "/camera_3/camera_info"
```

## Usage

1. Place the ArUco marker with ID 0 where any camera can see it. This marker will serve as the reference point. The system will consider this marker's position as the origin (0,0,0) of the global coordinate system, called `map`.

<img src="https://github.com/user-attachments/assets/45816ef9-deda-4200-a8b1-42fd3999c760" alt="setup_paint" width="100%"/>

2. Distribute the remaining ArUco markers around the room, ensuring they're visible to different cameras.

  For best results:

  - Try to have each camera see multiple markers.
  - Aim for overlap, where multiple cameras can see the same markers.
  - The more markers a camera can detect, and the more cameras that can see the same markers, the more accurate your calibration will be.

  Remember, the system first builds a map of marker positions, then determines camera positions based on this map. So, having markers visible to multiple cameras helps create a more accurate and interconnected calibration.

  <img src="https://github.com/user-attachments/assets/3bc13824-a241-475a-bf4e-2e4b1650691a" alt="cameras_paint" width="100%"/>

3. Initiate the calibration process by running the `extrinsic_calibrator_node`.

<img src="https://github.com/user-attachments/assets/8da30c20-f7fb-448c-9c24-5851c9d8aef3" alt="calibration_cameras_paint" width="100%"/>

4. Wait for the algorithm to gather the transform of each marker from each camera. The algorithm will iteratively tell the user which marker transforms are finally reliable and which ones are still being verified.

<img src="https://github.com/user-attachments/assets/31c7d46d-9936-454f-ab96-4d5bf8123879" alt="calibration_debug" width="100%"/>

5. The calibrator will provide tables with useful information, while the calibration is taking place.

<img src="https://github.com/user-attachments/assets/62f81fe5-fa43-4034-8e87-5d7e3ad86d14" alt="markers_per_cam" width="60%"/>

<img src="https://github.com/user-attachments/assets/da7421f7-c5cd-4c7e-b214-76f5ffd2e994" alt="marker_per_marker" width="60%"/>

6. Finally, once the calibration is done, the frame of each marker and camera will be published in tf2.

<img src="https://github.com/user-attachments/assets/21d1daf6-b1f4-4eea-ac81-5ba6d5fa5aae" alt="tf_map" width="100%"/>


### Launching the Calibrator

To run the extrinsic calibrator node, use the following command:
```sh
ros2 run extrinsic_calibrator_core extrinsic_calibrator_node.py
```

### ArUco Marker Generator

This package also provides a utility to generate printable ArUco markers. To generate a set of markers, run:
``` sh
ros2 run extrinsic_calibrator_core generator_aruco_node.py
```
Make sure to manually adjust the size of the printed ArUco markers before printing them. The length of the side of each ArUco marker has to match the marker_length parameter defined in aruco_parameters.yaml.

## Dependencies

The package relies on the following libraries and ROS2 packages:

- numpy for numerical operations
- OpenCV for ArUco marker detection
- PrettyTable to easily print the calibration status
- tf2_ros for handling transformations
- tf_transformations for matrix transformations

To install the necessary dependencies, ensure you run:
```sh
# update libraries
sudo apt-get update
# install ros dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# install python dependencies
sudo pip install -r src/extrinsic_calibrator_core/dependencies.txt
```

## Author Information

**Author:** Josep Rueda Collell  
**Created:** October 2024  
**Email:** [rueda_999@hotmail.com](mailto:rueda_999@hotmail.com)
**Affiliation:** [IKERLAN](https://www.ikerlan.es)

### Citation
If you use this code, please cite:  
**Josep Rueda Collell**. "ROS2 Extrinsic Camera Calibrator using ArUco Markers". (2024).


Developed as part of **AI-PRISM** project.

<a href="https://aiprism.eu/">
<img src="https://aiprism.eu/wp-content/uploads/2022/11/Ai-Prism_Logo_Horizontal-e1669543082668-1024x221.png" height="48px" />
</a>

*AI Powered human-centred Robot Interactions for Smart Manufacturing*

<a href="https://aiprism.eu/">
<img src="https://aiprism.eu/wp-content/uploads/2022/11/eu_funded_en-1024x215.jpg" height="48px" />
</a>

Horizon Europe – Grant Agreement number [101058589](https://cordis.europa.eu/project/id/101058589)

*Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union. The European Union cannot be held responsible for them. Neither the European Union nor the granting authority can be held responsible for them.*

## License

This work is licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).  
See the [LICENSE](https://github.com/pep248/extrinsic_calibrator/blob/main/LICENSE) file for more details.
