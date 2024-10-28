from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        # Launch the extrinsic calibrator node. The config file is in the config folder and is passed to the node using the generate_parameter_library
        Node(
            package='extrinsic_calibrator_core',
            executable='extrinsic_calibrator_node.py',
            name='extrinsic_calibrator_node',
            output='screen',
        ), 
        
        # Laucnh the set of usb-cameras with their own config_files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("extrinsic_calibrator_examples"),
                "launch",
                "launch_usb_cameras.launch.py"]))
        ),
        
        # Laucnh the rviz visualizer with the TF of the map and the cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("extrinsic_calibrator_examples"),
                "launch",
                "launch_rviz.launch.py"]))
        ),
    ])
