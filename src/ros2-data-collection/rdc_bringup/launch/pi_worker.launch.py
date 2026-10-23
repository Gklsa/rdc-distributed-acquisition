"""
Per-device worker launch file.

Starts the sensor node, experiment manager, and runtime status node for one Pi.
Prefer passing config:=src/devices_config/<device>.yaml so the Makefile does
not need to know individual packages or node parameters.
"""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _csv(value):
    if isinstance(value, list):
        return ','.join(str(item) for item in value)
    return str(value or '')


def _node_suffix(device_name):
    safe = re.sub(r'[^A-Za-z0-9_]+', '_', device_name or 'device').strip('_')
    return safe or 'device'


def _load_config(path):
    if not path:
        return {}
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            'pyyaml is required when using pi_worker.launch.py config:=...'
        ) from exc
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _launch_nodes(context):
    config = _load_config(LaunchConfiguration('config').perform(context))

    def arg_or_config(arg_name, key, default=''):
        value = LaunchConfiguration(arg_name).perform(context)
        return value if value else config.get(key, default)

    device_name = arg_or_config('device_name', 'device_name', '')
    topics = arg_or_config('topics', 'topics', '/imu/data,/temperature,/camera/image_raw')
    recordings_dir = arg_or_config('recordings_dir', 'recordings_dir', '/ros2_ws/recordings')
    mqtt_broker = arg_or_config('mqtt_broker_host', 'mqtt_broker', 'localhost')
    recording_mode = arg_or_config('recording_mode', 'recording_mode', 'python_subprocess')
    sensor_package = arg_or_config('sensor_package', 'sensor_package', '')
    accepted_master = arg_or_config(
        'accepted_master_device_id',
        'accepted_master_device_id',
        '',
    )

    topics_csv = _csv(topics)
    suffix = _node_suffix(device_name)
    manager_executable = (
        'experiment_manager_cpp' if recording_mode == 'cpp_writer' else 'experiment_manager'
    )

    nodes = []
    if sensor_package:
        nodes.append(Node(
            package=sensor_package,
            executable='tmp116_node',
            name=f'tmp116_node_{suffix}',
            output='screen',
        ))

    nodes.append(Node(
        package='rdc_core',
        executable=manager_executable,
        name=f'experiment_manager_{suffix}',
        output='screen',
        parameters=[{
            'recordings_dir': recordings_dir,
            'topics': topics_csv,
            'device_name': device_name,
            'accepted_master_device_id': accepted_master,
        }],
    ))

    nodes.append(Node(
        package='rdc_core',
        executable='runtime_status',
        name=f'runtime_status_{suffix}',
        output='screen',
        parameters=[{
            'topics': topics_csv,
            'device_name': device_name,
            'mqtt_broker_host': mqtt_broker,
        }],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='Optional per-device YAML config',
        ),
        DeclareLaunchArgument(
            'recordings_dir',
            default_value='',
            description='Override recordings directory',
        ),
        DeclareLaunchArgument(
            'topics',
            default_value='',
            description='Override comma-separated topic list',
        ),
        DeclareLaunchArgument(
            'device_name',
            default_value='',
            description='Override device identifier',
        ),
        DeclareLaunchArgument(
            'mqtt_broker_host',
            default_value='',
            description='Override MQTT broker host',
        ),
        DeclareLaunchArgument(
            'recording_mode',
            default_value='',
            description='python_subprocess or cpp_writer',
        ),
        DeclareLaunchArgument(
            'sensor_package',
            default_value='',
            description='Package exporting tmp116_node, e.g. ti_tmp116',
        ),
        DeclareLaunchArgument(
            'accepted_master_device_id',
            default_value='',
            description='Only accept START/STOP commands from this master_device_id',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
