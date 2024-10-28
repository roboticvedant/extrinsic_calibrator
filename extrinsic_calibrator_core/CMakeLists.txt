cmake_minimum_required(VERSION 3.8)
project(extrinsic_calibrator_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# DEPENDENCIES

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find_package(generate_parameter_library REQUIRED)
# generate_parameter_library(
#   cpp_camera_topics_parameters # cmake target name for the parameter library
#   config/camera_topics_parameters.yaml # path to input yaml file
# )

generate_parameter_module( python_aruco_parameters # cmake target name for the parameter library
  config/aruco_parameters.yaml # path to input yaml file
)

generate_parameter_module( python_camera_topics_parameters # cmake target name for the parameter library
  config/camera_topics_parameters.yaml # path to input yaml file
)

# PYTHON
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/extrinsic_calibrator_node.py
  scripts/generator_aruco_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# COPY A PARTICULAR FOLDER TO THE INSTALL DIRECTORY
# Install config dependencies
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)


# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)