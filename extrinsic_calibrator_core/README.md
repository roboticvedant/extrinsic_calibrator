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

<img src="https://github.com/user-attachments/assets/fdf38088-eee8-4648-a192-d9df6b2c2c37" alt="setup_paint" width="100%"/>

2. Distribute the remaining ArUco markers around the room, ensuring they're visible to different cameras.

  For best results:

  - Try to have each camera see multiple markers.
  - Aim for overlap, where multiple cameras can see the same markers.
  - The more markers a camera can detect, and the more cameras that can see the same markers, the more accurate your calibration will be.

  Remember, the system first builds a map of marker positions, then determines camera positions based on this map. So, having markers visible to multiple cameras helps create a more accurate and interconnected calibration.

  <img src="![cameras_paint](https://github.com/user-attachments/assets/4ad78803-f451-4e2d-bd2b-a39e2df49f9b)
" alt="cameras_paint" width="100%"/>

3. Initiate the calibration process by running the `extrinsic_calibrator_node`.

<img src="https://github.com/user-attachments/assets/737566b8-69ee-424e-ad97-f0cddda23562" alt="calibration_cameras_paint" width="100%"/>

4. Wait for the algorithm to gather the transform of each marker from each camera. The algorithm will iteratively tell the user which marker transforms are finally reliable and which ones are still being verified.

<img src="https://github.com/user-attachments/assets/70207ffc-39a4-4aa7-86bf-2cc6ce940463" alt="calibration_debug" width="100%"/>

5. The calibrator will provide tables with useful information, while the calibration is taking place.

<img src="https://github.com/user-attachments/assets/3d92d2de-0152-4fd3-89d3-3d56632eb82c" alt="markers_per_cam" width="60%"/>

<img src="https://github.com/user-attachments/assets/4d8e2b21-d324-4d98-92f9-c2bb9a1f3e8e" alt="marker_per_marker" width="60%"/>

6. Finally, once the calibration is done, the frame of each marker and camera will be published in tf2.

<img src="https://github.com/user-attachments/assets/9b113b6d-e571-4c7e-81ca-356cdfe71376" alt="tf_map" width="100%"/>


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
```

## Author Information

**Authors:**
- [Josep Rueda Collell](mailto:rueda_999@hotmail.com)  
- [Ander Gonzalez](mailto:ander.gonzalez@ikelan.es)

**Created:** October 2024

**Affiliation:** [IKERLAN](https://www.ikerlan.es)

<img src="https://github.com/user-attachments/assets/41cb9091-52c5-4f90-bbc9-ec02814dee49" alt="setup_paint" width="40%"/>

### Citation
If you use this code, please cite:  
**Josep Rueda Collell**. "ROS2 Extrinsic Camera Calibrator using ArUco Markers". (2024).

---

Developed as part of **AI-PRISM** project.

<a href="https://aiprism.eu/">
<img src="https://aiprism.eu/wp-content/uploads/2022/11/Ai-Prism_Logo_Horizontal-e1669543082668-1024x221.png" height="96px" />
</a>

*AI Powered human-centred Robot Interactions for Smart Manufacturing*

<a href="https://aiprism.eu/">
<img src="https://aiprism.eu/wp-content/uploads/2022/11/eu_funded_en-1024x215.jpg" height="48px" />
</a>

Horizon Europe – Grant Agreement number [101058589](https://cordis.europa.eu/project/id/101058589)

*Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union. The European Union cannot be held responsible for them. Neither the European Union nor the granting authority can be held responsible for them.*

## License

This software is provided under a dual license system. You may choose between:

- **GNU Affero General Public License v3**: For open-source development, subject to the conditions of this license.
- **Commercial License**: For proprietary use. For more details on the commercial license, please contact us at [info@ikerlan.es](mailto:info@ikerlan.es).

Please see the [LICENSE](./license.md) file for the complete terms and conditions of each license option.
