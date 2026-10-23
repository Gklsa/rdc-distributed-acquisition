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
        Node(
            package='rdc_core',
            executable='experiment_manager',
            name='experiment_manager',
            output='screen',
        ),
        Node(
            package='rdc_core',
            executable='runtime_status',
            name='runtime_status',
            output='screen',
        ),
    ])
