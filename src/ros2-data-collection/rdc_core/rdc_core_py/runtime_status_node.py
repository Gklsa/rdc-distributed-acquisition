"""
Runtime Status Node -- Task 3

Runs alongside sensor publishers and experiment_manager. Monitors the health
of configured ROS2 topics (message count, observed Hz, last message age) and
periodically publishes a JSON status report to:
  - ROS2 topic : /runtime/status  (std_msgs/String, JSON)
  - MQTT topic : runtime/<device_id>/status  (JSON, same payload)

The MQTT broker address is configurable via the mqtt_broker_host parameter.
If the paho-mqtt library is not installed, MQTT publishing is skipped with
a warning and the node continues to publish on the ROS2 topic only.

Parameters:
  topics               comma-separated list of topics to monitor
                       (default: /imu/data,/temperature,/camera/image_raw)
  expected_hz          comma-separated expected Hz per topic, same order
                       (default: 10,1,10)
  device_name          device identifier override (same as experiment_manager)
  mqtt_broker_host     MQTT broker hostname or IP  (default: localhost)
  mqtt_broker_port     MQTT broker port             (default: 1883)
  publish_interval     status publish period in seconds (default: 5.0)
"""

import glob
import json
import os
import re
import socket
import time
from collections import deque
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, Temperature, Image
from std_msgs.msg import String

TOPIC_TYPE_MAP = {
    '/imu/data':          Imu,
    '/temperature':       Temperature,
    '/camera/image_raw':  Image,
}

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

DEFAULT_TOPICS = '/imu/data,/temperature,/camera/image_raw'
DEFAULT_HZ     = '10,1,10'
DEFAULT_HZ_BY_TOPIC = {
    '/imu/data':          10.0,
    '/temperature':       1.0,
    '/camera/image_raw':  10.0,
}


class RuntimeStatusNode(Node):
    def __init__(self):
        super().__init__('runtime_status')

        self.declare_parameter('topics',            DEFAULT_TOPICS)
        self.declare_parameter('expected_hz',       DEFAULT_HZ)
        self.declare_parameter('device_name',       '')
        self.declare_parameter('mqtt_broker_host',  'localhost')
        self.declare_parameter('mqtt_broker_port',  1883)
        self.declare_parameter('publish_interval',  5.0)

        topics_str   = self.get_parameter('topics').get_parameter_value().string_value
        hz_str       = self.get_parameter('expected_hz').get_parameter_value().string_value
        device_name  = self.get_parameter('device_name').get_parameter_value().string_value.strip()
        broker_host  = self.get_parameter('mqtt_broker_host').get_parameter_value().string_value
        broker_port  = self.get_parameter('mqtt_broker_port').get_parameter_value().integer_value
        interval     = self.get_parameter('publish_interval').get_parameter_value().double_value

        self._topics = [t.strip() for t in topics_str.split(',') if t.strip()]
        hz_list      = [h.strip() for h in hz_str.split(',') if h.strip()]
        # DEFAULT_HZ is positional against DEFAULT_TOPICS' order ('/imu/data,/temperature,
        # /camera/image_raw'). If `topics` ends up being some other subset/order of that
        # (e.g. just '/temperature' alone, as device1.yaml does) and `expected_hz` was never
        # explicitly overridden, positional matching grabs the wrong value (index 0 -> 10,
        # meant for IMU). Fall back to a name-keyed lookup in that case; an explicitly
        # passed expected_hz (aligned with the caller's own topics order) still matches
        # positionally as before.
        using_default_hz = (hz_str == DEFAULT_HZ)
        self._expected_hz = {}
        for i, topic in enumerate(self._topics):
            if using_default_hz:
                self._expected_hz[topic] = DEFAULT_HZ_BY_TOPIC.get(topic, 10.0)
            else:
                self._expected_hz[topic] = float(hz_list[i]) if i < len(hz_list) else 10.0

        # Keep the ID safe for ROS names and MQTT topic paths, including when
        # falling back to a hostname that contains characters such as '-'.
        if device_name:
            raw_device_id = device_name
        else:
            hostname = socket.gethostname()
            local_ip = self._get_local_ip()
            raw_device_id = f'{hostname}_{local_ip.replace(".", "_")}'
        self._device_id = re.sub(r'[^A-Za-z0-9_]+', '_', raw_device_id).strip('_') or 'device'

        # Per-topic tracking: sliding window of recent message timestamps (last 30s)
        self._msg_times  = {t: deque() for t in self._topics}
        self._msg_counts = {t: 0 for t in self._topics}

        # Experiment state received from experiment_manager
        self._experiment_id  = None
        self._source_master_device_id = None
        self._exp_status     = 'idle'
        self._exp_start_time = None
        self._bag_directory  = None
        self._join_mode      = None

        # ROS2 publisher and subscriptions
        self.status_pub = self.create_publisher(String, '/runtime/status', 10)

        # Use depth=1 BEST_EFFORT so large messages (e.g. camera images) are
        # dropped rather than queued — we only need the arrival timestamp.
        monitor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for topic in self._topics:
            msg_type = TOPIC_TYPE_MAP.get(topic, String)
            self.create_subscription(msg_type, topic, self._make_callback(topic), monitor_qos)

        self.create_subscription(String, '/experiment/state', self._state_callback, 10)

        self.create_timer(interval, self._publish_status)

        # MQTT client
        self._mqtt_client = None
        self._mqtt_topic  = f'runtime/{self._device_id}/status'
        if MQTT_AVAILABLE:
            self._connect_mqtt(broker_host, broker_port)
        else:
            self.get_logger().warn(
                'paho-mqtt not installed — MQTT publishing disabled. '
                'Install with: pip install paho-mqtt'
            )

        self.get_logger().info(f'RuntimeStatusNode ready. device_id={self._device_id}')
        self.get_logger().info(f'  monitoring : {self._topics}')
        self.get_logger().info(f'  mqtt topic : {self._mqtt_topic}')
        self.get_logger().info(f'  interval   : {interval}s')

    # ------------------------------------------------------------------
    def _make_callback(self, topic):
        def _cb(msg):
            now = time.monotonic()
            self._msg_times[topic].append(now)
            self._msg_counts[topic] += 1
            # drop timestamps older than 30 seconds
            cutoff = now - 30.0
            while self._msg_times[topic] and self._msg_times[topic][0] < cutoff:
                self._msg_times[topic].popleft()
        return _cb

    def _get_bag_size_mb(self):
        if not self._bag_directory:
            return None
        try:
            mcap_files = glob.glob(os.path.join(self._bag_directory, '*.mcap'))
            if not mcap_files:
                return None
            total_bytes = sum(os.path.getsize(f) for f in mcap_files)
            return round(total_bytes / (1024 * 1024), 1)
        except OSError:
            return None

    def _state_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._experiment_id  = data.get('experiment_id')
        self._source_master_device_id = data.get('source_master_device_id')
        self._exp_status     = data.get('status', 'idle')
        self._exp_start_time = data.get('pi_start_time')
        self._bag_directory  = data.get('bag_directory')
        self._join_mode      = data.get('join_mode')

    # ------------------------------------------------------------------
    def _publish_status(self):
        now_mono = time.monotonic()
        now_iso  = datetime.now(timezone.utc).isoformat()

        topic_stats = {}
        for topic in self._topics:
            times  = self._msg_times[topic]
            count  = self._msg_counts[topic]
            exp_hz = self._expected_hz[topic]

            # observed Hz from messages in the last 10 seconds
            window = 10.0
            cutoff = now_mono - window
            recent = [t for t in times if t >= cutoff]
            obs_hz = round(len(recent) / window, 2) if recent else 0.0

            last_age = round(now_mono - times[-1], 3) if times else None

            if obs_hz >= exp_hz * 0.8:
                state = 'ok'
            elif obs_hz >= exp_hz * 0.3:
                state = 'warning'
            else:
                state = 'error' if count > 0 else 'inactive'

            topic_stats[topic] = {
                'expected_hz':       exp_hz,
                'observed_hz':       obs_hz,
                'message_count':     count,
                'last_msg_age_s':    last_age,
                'state':             state,
            }

        runtime_s = None
        if self._exp_start_time and self._exp_status == 'running':
            try:
                start = datetime.fromisoformat(self._exp_start_time)
                runtime_s = round(
                    (datetime.now(timezone.utc) - start).total_seconds(), 1
                )
            except ValueError:
                pass

        payload = {
            'schema_version':  '1.0',
            'device_id':       self._device_id,
            'experiment_id':   self._experiment_id,
            'source_master_device_id': self._source_master_device_id,
            'status':          self._exp_status,
            'recording_active': self._exp_status == 'running',
            'recording_path':  self._bag_directory,
            'bag_size_mb':     self._get_bag_size_mb(),
            'join_mode':       self._join_mode,
            'pi_start_time':   self._exp_start_time,
            'runtime_seconds': runtime_s,
            'last_update':     now_iso,
            'configured_topics': self._topics,
            'topic_stats':     topic_stats,
        }

        json_str = json.dumps(payload)

        out = String()
        out.data = json_str
        self.status_pub.publish(out)

        if self._mqtt_client:
            try:
                self._mqtt_client.publish(self._mqtt_topic, json_str, qos=0, retain=False)
            except Exception as e:
                self.get_logger().warn(f'MQTT publish failed: {e}')

    # ------------------------------------------------------------------
    def _connect_mqtt(self, host, port):
        try:
            client = mqtt.Client(client_id=f'ros2_{self._device_id}')
            client.connect(host, port, keepalive=60)
            client.loop_start()
            self._mqtt_client = client
            self.get_logger().info(f'MQTT connected to {host}:{port}')
        except Exception as e:
            self.get_logger().warn(f'MQTT connection failed ({host}:{port}): {e}')
            self.get_logger().warn('Continuing without MQTT — start the broker and restart.')

    def _get_local_ip(self):
        host_ip = os.environ.get('HOST_IP', '').strip()
        if host_ip:
            return host_ip
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return '127.0.0.1'

    def destroy_node(self):
        if self._mqtt_client:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
