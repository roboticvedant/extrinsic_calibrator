import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'extrinsic_calibrator_examples'
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    d435_config = os.path.join(config_dir, 'd435.yaml')
    l515_config = os.path.join(config_dir, 'l515.yaml')

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
        )
    ])
