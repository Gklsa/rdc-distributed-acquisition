"""
Local test launch -- mocked_sensors_cpp

Starts only the C++ mock camera publisher, for testing this package
in isolation without going through rdc_bringup.

Usage:
  ros2 launch mocked_sensors_cpp test_mock_camera.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mocked_sensors_cpp',
            executable='mock_camera_publisher',
            name='mock_camera_publisher',
            output='screen',
        ),
    ])
