"""
PC Master Node -- Task 2 / Task 3

Responsibilities:
  1. User calls /start_experiment service -> generate shared experiment_id
     -> broadcast START command to all Pi workers
  2. User calls /stop_experiment service  -> broadcast STOP to all Pi workers

Each Pi runs ExperimentManagerNode which subscribes to /experiment/command.
All Pis use the same experiment_id, allowing data to be aligned in post-processing.

Broadcast message format (JSON string on /experiment/command topic):
  START: {"command": "START", "experiment_id": "exp_001_20260515T151145Z",
          "pc_timestamp": "2026-05-15T15:11:45.123456+00:00"}
  STOP:  {"command": "STOP",  "experiment_id": "exp_001_20260515T151145Z"}
"""

import json
import os
import re
import socket
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEFAULT_RECORDINGS_DIR = '/ros2_ws/recordings'


class PcMasterNode(Node):
    def __init__(self):
        super().__init__('pc_master')

        self.declare_parameter('recordings_dir', DEFAULT_RECORDINGS_DIR)
        self.declare_parameter('master_device_id', '')
        self.recordings_dir = (
            self.get_parameter('recordings_dir').get_parameter_value().string_value
        )
        configured_master = (
            self.get_parameter('master_device_id').get_parameter_value().string_value.strip()
        )
        self.master_device_id = configured_master or self._default_master_id()

        self.cmd_pub = self.create_publisher(String, '/experiment/command', 10)

        self.start_srv = self.create_service(
            Trigger, '/start_experiment', self.start_callback
        )
        self.stop_srv = self.create_service(
            Trigger, '/stop_experiment', self.stop_callback
        )

        self._current_exp_id   = None
        self._current_pc_ts    = None

        # Heartbeat: re-broadcast current experiment state every 10s so Pis
        # that reconnect after a drop can automatically rejoin.
        self.create_timer(10.0, self._heartbeat)

        self.get_logger().info(f'PC Master ready. master_device_id={self.master_device_id}')
        self.get_logger().info('Call /start_experiment to broadcast START to all Pis.')
        self.get_logger().info('Call /stop_experiment  to broadcast STOP  to all Pis.')

    # ------------------------------------------------------------------
    def start_callback(self, request, response):
        if self._current_exp_id is not None:
            response.success = False
            response.message = f'Already running: {self._current_exp_id}'
            return response

        start_time = datetime.now(timezone.utc)
        ts = start_time.strftime('%Y%m%dT%H%M%SZ')
        suffix = uuid.uuid4().hex[:8]
        safe_master = re.sub(r'[^A-Za-z0-9_]+', '_', self.master_device_id).strip('_')
        self._current_exp_id = f'exp_{ts}_{safe_master}_{suffix}'
        self._current_pc_ts  = start_time.isoformat()

        self._create_documentation_template(self._current_exp_id, start_time)
        self._publish_command('START', self._current_exp_id, self._current_pc_ts)

        response.success = True
        response.message = f'Broadcast START | experiment_id={self._current_exp_id}'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    def stop_callback(self, request, response):
        if self._current_exp_id is None:
            response.success = False
            response.message = 'No experiment is currently running'
            return response

        exp_id = self._current_exp_id
        self._publish_command('STOP', exp_id)
        self._current_exp_id = None
        self._current_pc_ts  = None

        response.success = True
        response.message = f'Broadcast STOP | experiment_id={exp_id}'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    def _heartbeat(self):
        # Re-broadcast current experiment state so Pis that reconnected after
        # a drop can automatically pick up the experiment_id and rejoin.
        if self._current_exp_id is not None:
            self._publish_command('START', self._current_exp_id, self._current_pc_ts)
            self.get_logger().debug(f'Heartbeat: re-broadcast {self._current_exp_id}')

    def _publish_command(self, command, experiment_id, pc_timestamp=None):
        payload = {
            'command': command,
            'experiment_id': experiment_id,
            'source_device_id': self.master_device_id,
        }
        if pc_timestamp:
            payload['pc_timestamp'] = pc_timestamp
        msg = String()
        msg.data = json.dumps(payload)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def _default_master_id(self):
        hostname = socket.gethostname().replace('-', '_')
        return re.sub(r'[^A-Za-z0-9_]+', '_', hostname).strip('_') or 'pc_master'

    def _create_documentation_template(self, experiment_id, start_time):
        exp_dir = os.path.join(self.recordings_dir, experiment_id)
        os.makedirs(exp_dir, exist_ok=True)
        path = os.path.join(exp_dir, 'experiment_documentation.yaml')
        if os.path.exists(path):
            self.get_logger().warn(f'Documentation template already exists: {path}')
            return

        template = f"""# Fill this file during or immediately after the experiment.
# The analysis notebook checks the mandatory fields before accepting the recording.
experiment_id: {experiment_id}
created_at_utc: {start_time.isoformat()}
master_device_id: {self.master_device_id}

mandatory:
  operator: ""
  goal: ""
  material_type: ""
  measurement_series: ""
  setup_notes: ""

optional:
  special_events: ""
  environmental_conditions: ""
  device_notes: ""
  external_references: ""
"""
        with open(path, 'w', encoding='utf-8') as f:
            f.write(template)
        self.get_logger().info(f'Created experiment documentation template: {path}')

    def _next_exp_number(self):
        pattern = re.compile(r'^exp_(\d+)_')
        max_n = 0
        try:
            for name in os.listdir(self.recordings_dir):
                m = pattern.match(name)
                if m:
                    max_n = max(max_n, int(m.group(1)))
        except FileNotFoundError:
            pass
        return max_n + 1


def main(args=None):
    rclpy.init(args=args)
    node = PcMasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
