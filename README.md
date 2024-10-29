# extrinsic_calibrator

<div align="center">
    <img src="https://github.com/user-attachments/assets/5358e238-ceb2-4bd3-b44d-7d1464012325" alt="setup_paint" width="70%"/>
</div>



`extrinsic_calibrator` is a ROS2 package designed to extrinsically calibrate a set of cameras distributed throughout a room. The calibration is performed using ArUco markers "randomly" scattered through the environment. Each camera detects one or several ArUco markers within its field of view and the algorithm reconstructs the positions of the markers to create a global map. The positions of the cameras are then computed and incorporated into this aforementioned map map.

This package is composed of two submodules:
1. [extrinsic_calibrator_core](./extrinsic_calibrator_core) with the extrinsic calibrator itself and an ArUco generator to simplify the usage.
2. [extrinsic_calibrator_examples](./extrinsic_calibrator_examples) with some useful launch files to easily set up your cameras as well as a rviz visualizer node.

## Installation


To clone and install the package, navigate into your workspace and run the following commands:
```sh
# clone the repository
cd ./src
git clone https://github.com/Ikerlan-KER/extrinsic_calibrator.git --branch humble
cd ..
# update libraries
sudo apt-get update
# install ros dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# build de workspace
colcon build
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