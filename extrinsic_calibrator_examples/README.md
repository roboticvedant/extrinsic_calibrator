# extrinsic_calibrator_examples

## Overview

`extrinsic_calibrator_examples` is a ROS2 package designed to provide examples on how to use the `extrinsic_calibrator_core` package as well as useful ros2 launch files to launch the cameras, the calibrator, as well as a demonstration rviz file.

## Features

- Launch file to launch a set of cameras using the `usb_camera` package as well as the corresponding set of config files to configure the cameras.
- Laucnh file to launch the rviz file to visualize the markers and the camera frames as well as the rviz file to configure it.
- Launch file to launch all the previous, as well as the calibrator, them being the set of cameras, the rviz visualizer and the calibrator itself.

## Configuration

The package provides configuration options through YAML files.

### Camera configuration

Here you have an example configuration file `l515.yaml` file to configure the camera according to the `usb_camera` package, as well as the corresponding intrinsic calibration file.

```yaml
/**:
    ros__parameters:
      video_device: "/dev/video12" # "ffplay /dev/video12" to test
      framerate: 6.0
      io_method: "mmap"
      frame_id: "cam2_frame"
      pixel_format: "yuyv"  # see usb_cam/supported_formats for list of supported formats
      av_device_format: "YUV422P"
      image_width: 640
      image_height: 480
      camera_name: "cam2"
      camera_info_url: "package://extrinsic_calibrator_examples/config/l515_intrinsics.yaml"
      brightness: -1
      contrast: -1
      saturation: -1
      sharpness: -1
      gain: -1
      auto_white_balance: true
      white_balance: 4000
      autoexposure: true
      exposure: 100
      autofocus: false
      focus: -1
```

Don't forget to modify the parameter `camera_info_url` to properly link the camera configuration to the intrinsic calibration file.

```yaml
image_width: 640
image_height: 480
camera_name: "cam2"
camera_matrix:
  rows: 3
  cols: 3
  data: [607.4058837890625, 0.0, 325.59991455078125, 0.0, 607.5341186523438, 247.25904846191406, 0.0, 0.0, 1.0]
distortion_model: "plumb_bob"
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.19551624357700348, -0.5865326523780823, -0.002620677463710308, 0.0008374004391953349, 0.5133219957351685]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [607.4058837890625, 0.0, 325.59991455078125, 0.0, 0.0, 607.5341186523438, 247.25904846191406, 0.0, 0.0, 0.0, 1.0, 0.0]

```

In case you want to launch more cameras with the same launch file, simply add them as additional nodes in the launch file `launch_usb_cameras.launch.py`:

```py
d435_config = os.path.join(config_dir, 'd435.yaml')
l515_config = os.path.join(config_dir, 'l515.yaml')
# d457_config = os.path.join(config_dir, 'd457.yaml')

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
    ),
    # Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='d457_camera',
    #     namespace='camera_3',
    #     parameters=[d457_config],
    #     output='screen'
    # ),
])

```

## Usage

### Launching the Cameras

Using the `usb_cam` package for your camera streams,you can launch the set of all cameras using:
```sh
ros2 launch extrinsic_calibrator_examples launch_usb_cameras.launch.py
```

### Launching the rviz visualizer

An example rviz config file is provided which includes displays for the `/camera_1/image_raw/detected_markers` topic and the  `/camera_2/image_raw/detected_markers` topic as well as the tf2 display of the found markers and cameras. To launch it, use:
```sh
ros2 launch extrinsic_calibrator_examples launch_rviz.launch.py
```

### Launching Both Cameras and the Calibrator

To simultaneously launch the cameras, the rviz visualizer and the extrinsic calibrator, use:
```sh
ros2 launch extrinsic_calibrator_examples launch_extrinsic_calibrator.launch.py
```


## Dependencies

The package relies on the following libraries and ROS2 packages:

- `extrinsic_calibrator_core` for the core functionality
- `usb_cam` package for camera streaming
- `rviz2` for visualization


To install the necessary dependencies, ensure you run:
```sh
# update libraries
sudo apt-get update
# install ros dependencies
rosdep update
```
## Author Information

**Author:** Josep Rueda Collell  
**Created:** October 2024  
**Email:** [rueda_999@hotmail.com](mailto:rueda_999@hotmail.com)  
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

This work is licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).  
See the [LICENSE](https://github.com/pep248/extrinsic_calibrator/blob/main/LICENSE) file for more details.

