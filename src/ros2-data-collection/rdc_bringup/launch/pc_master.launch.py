"""
PC master launch file.

The executable remains rdc_core/pc_master. The ROS node name is derived from
master_device_id so multiple users/devices in one ROS_DOMAIN_ID are traceable.
"""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node_suffix(value):
    safe = re.sub(r'[^A-Za-z0-9_]+', '_', value or 'master').strip('_')
    return safe or 'master'


def _launch_nodes(context):
    master_device_id = LaunchConfiguration('master_device_id').perform(context)
    recordings_dir = LaunchConfiguration('recordings_dir').perform(context)
    suffix = _node_suffix(master_device_id)

    return [
        Node(
            package='rdc_core',
            executable='pc_master',
            name=f'pc_master_{suffix}',
            output='screen',
            parameters=[{
                'recordings_dir': recordings_dir,
                'master_device_id': master_device_id,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'recordings_dir',
            default_value='/ros2_ws/recordings',
            description='Used as the root directory for experiment outputs',
        ),
        DeclareLaunchArgument(
            'master_device_id',
            default_value='review_pc',
            description='Stable ID of the only node allowed to originate START/STOP commands',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
