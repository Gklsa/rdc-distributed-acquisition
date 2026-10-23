import json
import os
import re
import socket
import subprocess
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEFAULT_RECORDINGS_DIR = '/ros2_ws/recordings'
DEFAULT_TOPICS = '/imu/data,/temperature,/camera/image_raw'

# Written by `make pi-sync-code` at packaging time (PC-side, real .git). Real Pis have no
# .git of their own (excluded from the sync tarball), so this file is their only source of
# truth for which commit/clean-state was actually deployed; fall back to `git` directly for
# setups where /ros2_ws is a real repo checkout (e.g. the docker-compose simulation).
GIT_SYNC_INFO_PATH = '/ros2_ws/.git_sync_info.json'
REPO_DIR = '/ros2_ws'


class ExperimentManagerNode(Node):
    def __init__(self):
        super().__init__('experiment_manager')

        self.declare_parameter('recordings_dir', DEFAULT_RECORDINGS_DIR)
        self.declare_parameter('topics', DEFAULT_TOPICS)
        self.declare_parameter('device_name', '')
        self.declare_parameter('accepted_master_device_id', '')

        self.recordings_dir = (
            self.get_parameter('recordings_dir').get_parameter_value().string_value
        )
        topics_str = self.get_parameter('topics').get_parameter_value().string_value
        self.topics = [t.strip() for t in topics_str.split(',') if t.strip()]

        device_name_override = (
            self.get_parameter('device_name').get_parameter_value().string_value.strip()
        )
        self.accepted_master_device_id = (
            self.get_parameter('accepted_master_device_id').get_parameter_value().string_value.strip()
        )

        os.makedirs(self.recordings_dir, exist_ok=True)

        # ROS names may only contain alphanumerics and underscores. Sanitize both
        # explicit IDs and hostname-based fallbacks before using them in service names.
        if device_name_override:
            raw_device_id = device_name_override
        else:
            hostname = socket.gethostname()
            local_ip = self._get_local_ip()
            raw_device_id = f'{hostname}_{local_ip.replace(".", "_")}'
        self._device_id = re.sub(r'[^A-Za-z0-9_]+', '_', raw_device_id).strip('_') or 'device'

        # Device-specific services — each Pi exposes its own uniquely named service
        # so individual Pis can be controlled independently (exp-rejoin, pi-down)
        # e.g. /device1/start_experiment, /device1/stop_experiment
        self.start_srv = self.create_service(
            Trigger, f'/{self._device_id}/start_experiment', self.start_callback
        )
        self.stop_srv = self.create_service(
            Trigger, f'/{self._device_id}/stop_experiment', self.stop_callback
        )

        # Broadcast subscription for PC master commands (Task 2+)
        self.cmd_sub = self.create_subscription(
            String, '/experiment/command', self.command_callback, 10
        )

        # Publishes experiment state so runtime_status_node can track it
        self.state_pub = self.create_publisher(String, '/experiment/state', 10)

        self._proc = None
        self._log_fh = None
        self._exp_dir = None
        self._exp_id = None
        self._bag_path = None
        self._start_time = None
        self._pc_start_time = None
        self._source_master_device_id = None
        self._join_mode = None
        self._code_commit_hash = None
        self._code_is_clean = None

        self.get_logger().info(f'ExperimentManager ready. device_id={self._device_id}')
        self.get_logger().info(f'  recordings_dir : {self.recordings_dir}')
        self.get_logger().info(f'  topics         : {self.topics}')
        self.get_logger().info(
            f'  accepted_master: {self.accepted_master_device_id or "any"}'
        )

    # ------------------------------------------------------------------
    # Local service callbacks (Task 1)
    # ------------------------------------------------------------------
    def start_callback(self, request, response):
        ok, msg = self._do_start(experiment_id=None, pc_timestamp=None)
        response.success = ok
        response.message = msg
        return response

    def stop_callback(self, request, response):
        ok, msg = self._do_stop()
        response.success = ok
        response.message = msg
        return response

    # ------------------------------------------------------------------
    # Broadcast topic callback (Task 2+)
    # ------------------------------------------------------------------
    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON on /experiment/command: {msg.data}')
            return

        cmd = data.get('command', '').upper()
        ext_id = data.get('experiment_id') or None
        pc_ts = data.get('pc_timestamp') or None
        source_master = data.get('source_device_id') or ''

        if (
            self.accepted_master_device_id
            and source_master
            and source_master != self.accepted_master_device_id
        ):
            self.get_logger().warn(
                f'Ignoring {cmd} for {ext_id}: source_device_id={source_master} '
                f'does not match accepted_master_device_id={self.accepted_master_device_id}'
            )
            return
        if self.accepted_master_device_id and not source_master:
            self.get_logger().warn(
                f'Ignoring {cmd} for {ext_id}: command has no source_device_id '
                f'but accepted_master_device_id={self.accepted_master_device_id} is required'
            )
            return

        if cmd == 'START':
            if self._proc is not None and self._exp_id == ext_id:
                # Heartbeat from pc_master while already recording this experiment — ignore
                return
            ok, text = self._do_start(
                experiment_id=ext_id,
                pc_timestamp=pc_ts,
                source_master_device_id=source_master or None,
            )
            self.get_logger().info(f'[remote] START -> {text}')
        elif cmd == 'STOP':
            ok, text = self._do_stop()
            self.get_logger().info(f'[remote] STOP  -> {text}')
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    # ------------------------------------------------------------------
    # Core logic
    # ------------------------------------------------------------------
    def _do_start(self, experiment_id=None, pc_timestamp=None, source_master_device_id=None):
        if self._proc is not None:
            return False, f'Already recording: {self._exp_id}'

        self._start_time = datetime.now(timezone.utc)
        self._pc_start_time = pc_timestamp
        self._source_master_device_id = source_master_device_id
        self._code_commit_hash, self._code_is_clean = self._get_code_version()

        # Use shared experiment_id from PC master (Task 2+) or generate locally (Task 1)
        if experiment_id:
            self._exp_id = experiment_id
        else:
            n = self._next_exp_number()
            ts = self._start_time.strftime('%Y%m%dT%H%M%SZ')
            self._exp_id = f'exp_{n:03d}_{ts}'

        # Two-level directory: recordings/<experiment_id>/<device_id>_<pi_timestamp>/
        pi_ts = self._start_time.strftime('%Y%m%dT%H%M%SZ')
        device_dir = f'{self._device_id}_{pi_ts}'
        self._exp_dir = os.path.join(self.recordings_dir, self._exp_id, device_dir)
        os.makedirs(self._exp_dir, exist_ok=True)
        self._bag_path = os.path.join(self._exp_dir, 'bag')

        # join_mode: "normal" if started with the experiment,
        #            "late_join" if Pi reconnected after the experiment began
        self._join_mode = 'normal'
        join_mode = self._join_mode
        if experiment_id and self._pc_start_time:
            try:
                pc_dt = datetime.fromisoformat(self._pc_start_time)
                delay = (self._start_time - pc_dt).total_seconds()
                if delay > 15:
                    join_mode = 'late_join'
                    self._join_mode = join_mode
            except ValueError:
                pass

        exp_info = {
            'experiment_id': self._exp_id,
            'device_id': self._device_id,
            'pc_start_time': self._pc_start_time,
            'source_master_device_id': self._source_master_device_id,
            'pi_start_time': self._start_time.isoformat(),
            'topics': self.topics,
            'storage_format': 'mcap',
            'bag_directory': self._bag_path,
            'trigger_mode': 'remote' if experiment_id else 'local',
            'join_mode': join_mode,
            'code_commit_hash': self._code_commit_hash,
            'code_is_clean': self._code_is_clean,
        }
        self._write_json('experiment_info.json', exp_info)

        self._log_fh = open(os.path.join(self._exp_dir, 'logs.txt'), 'w')
        cmd = ['ros2', 'bag', 'record', '-o', self._bag_path, '--storage', 'mcap'] + self.topics
        self.get_logger().info(f'Running: {" ".join(cmd)}')
        self._proc = subprocess.Popen(cmd, stdout=self._log_fh, stderr=subprocess.STDOUT)

        msg = f'Started {self._exp_id} | device={self._device_id} | dir={self._exp_dir}'
        self.get_logger().info(msg)
        self._publish_state()
        return True, msg

    def _do_stop(self):
        if self._proc is None:
            return False, 'No experiment is currently running'

        self._proc.terminate()
        try:
            self._proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('rosbag2 did not stop in time, sending SIGKILL')
            self._proc.kill()
            self._proc.wait()

        self._proc = None
        if self._log_fh:
            self._log_fh.close()
            self._log_fh = None

        end_time = datetime.now(timezone.utc)
        duration = (end_time - self._start_time).total_seconds()

        metadata = {
            'experiment_id': self._exp_id,
            'device_id': self._device_id,
            'pc_start_time': self._pc_start_time,
            'source_master_device_id': self._source_master_device_id,
            'pi_start_time': self._start_time.isoformat(),
            'pi_end_time': end_time.isoformat(),
            'duration_seconds': round(duration, 3),
            'status': 'completed',
            'topics_recorded': self.topics,
            'bag_directory': self._bag_path,
            'code_commit_hash': self._code_commit_hash,
            'code_is_clean': self._code_is_clean,
        }
        self._write_json('metadata.json', metadata)

        exp_id = self._exp_id
        self._exp_id = None
        self._exp_dir = None
        self._bag_path = None
        self._start_time = None
        self._pc_start_time = None
        self._source_master_device_id = None
        self._code_commit_hash = None
        self._code_is_clean = None

        msg = f'Stopped {exp_id} | duration={duration:.1f}s'
        self.get_logger().info(msg)
        self._publish_state()
        return True, msg

    # ------------------------------------------------------------------
    def _publish_state(self):
        state = {
            'device_id': self._device_id,
            'experiment_id': self._exp_id,
            'source_master_device_id': self._source_master_device_id,
            'status': 'running' if self._proc is not None else 'idle',
            'pi_start_time': self._start_time.isoformat() if self._start_time else None,
            'topics': self.topics,
            'bag_directory': self._bag_path,
            'join_mode': self._join_mode,
        }
        out = String()
        out.data = json.dumps(state)
        self.state_pub.publish(out)

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

    def _get_code_version(self):
        if os.path.exists(GIT_SYNC_INFO_PATH):
            try:
                with open(GIT_SYNC_INFO_PATH) as f:
                    info = json.load(f)
                return info.get('commit_hash', 'unknown'), info.get('is_clean')
            except (OSError, json.JSONDecodeError) as exc:
                self.get_logger().warn(f'Failed to read {GIT_SYNC_INFO_PATH}: {exc}')

        # -c safe.directory=... bypasses git's "dubious ownership" check, which otherwise
        # blocks these commands whenever the bind-mounted repo is owned by a different UID
        # than the container's (e.g. host user vs. root inside the container).
        git_prefix = ['git', '-c', f'safe.directory={REPO_DIR}', '-C', REPO_DIR]
        try:
            commit_hash = subprocess.check_output(
                git_prefix + ['rev-parse', 'HEAD'],
                stderr=subprocess.DEVNULL, text=True,
            ).strip()
            status = subprocess.check_output(
                git_prefix + ['status', '--porcelain'],
                stderr=subprocess.DEVNULL, text=True,
            )
            return commit_hash, status.strip() == ''
        except (subprocess.CalledProcessError, FileNotFoundError):
            return 'unknown', None

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

    def _write_json(self, filename, data):
        path = os.path.join(self._exp_dir, filename)
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'Written: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
