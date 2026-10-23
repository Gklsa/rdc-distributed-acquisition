"""
Sensor Mock Publishers Launch File -- simulation only

Launches mock sensor publishers for PC-side simulation and testing.
In real hardware deployment, each mock node is replaced by the
corresponding hardware driver node running on the actual Pi.

Use this file when running the full simulated environment (no real hardware).
Do NOT run this on a real Pi.

Simulation status:
  - mock_temperature_publisher: active
  - mock_imu_publisher:         active
  - mock_camera_publisher:      active

Usage:
  ros2 launch rdc_bringup sensors.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mocked_sensors_py',
            executable='mock_imu_publisher',
            name='mock_imu_publisher',
            output='screen',
        ),

        Node(
            package='mocked_sensors_py',
            executable='mock_temperature_publisher',
            name='mock_temperature_publisher',
            output='screen',
        ),

        Node(
            package='mocked_sensors_py',
            executable='mock_camera_publisher',
            name='mock_camera_publisher',
            output='screen',
        ),
    ])
