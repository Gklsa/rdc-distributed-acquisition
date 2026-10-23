#!/usr/bin/env python3
"""
MQTT Runtime Monitor -- PC-side dashboard

Runs on PC only. Subscribes to the MQTT broker on Pi0 and displays
real-time sensor health and experiment status for all connected Pis.

Message source:
    runtime_status_node.py (running on each Pi) publishes JSON every 5s
    to MQTT topic: runtime/<device_id>/status
    via Pi0's mosquitto broker → WiFi → this script on PC

This script subscribes to: runtime/+/status  (+ matches all device IDs)

Features:
    - Per-device block: status, runtime, bag size, sensor Hz
    - Offline detection (> 15s no message): OFFLINE + gap duration + drop count
    - Auto-reconnect display when Pi rejoins via pc_master heartbeat

Usage:
    python3 src/tools/mqtt_monitor.py
    python3 src/tools/mqtt_monitor.py --broker localhost   # Pi0 IP
    python3 src/tools/mqtt_monitor.py --timeout 15

Requirements:
    pip install paho-mqtt
"""

import argparse
import json
import os
import re
import socket
import sys
import threading
import time
import uuid

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print('ERROR: paho-mqtt not installed. Run: pip install paho-mqtt')
    sys.exit(1)

_devices   = {}   # device_id -> latest payload
_last_seen = {}   # device_id -> monotonic of last message
_offline_at = {}  # device_id -> (monotonic, wall_time_str)
_connected = False
_broker_label = ''

class DeviceHistory:
    def __init__(self):
        self.drop_count     = 0
        self.last_reconnect = None   # "HH:MM:SS"
        self.last_gap_s     = None   # float
        self.reconnect_mono = None   # monotonic (for 60s display window)
        self.offline_since  = None   # "HH:MM:SS"
        self.offline_mono   = None   # monotonic

_history = {}
_TIMEOUT          = 15
_RECONNECT_SHOW_S = 60
W                 = 78   # device block width (must be <= terminal width)

GREEN  = '\033[32m'
YELLOW = '\033[33m'
RED    = '\033[31m'
GRAY   = '\033[90m'
CYAN   = '\033[36m'
BOLD   = '\033[1m'
RESET  = '\033[0m'

_ANSI_RE = re.compile(r'\033\[[0-9;]*m')

def _vis(s):
    """Visible length of a string (ignores ANSI colour codes)."""
    return len(_ANSI_RE.sub('', s))

def _box_top(label_display):
    """┌─ LABEL ──────────────────────────────────────┐"""
    inner = W - 2
    part  = f'─ {label_display} '
    dashes = '─' * (inner - _vis(part))
    print(f'┌{part}{dashes}┐')

def _box_mid():
    """├──────────────────────────────────────────────┤"""
    print('├' + '─' * (W - 2) + '┤')

def _box_bot():
    """└──────────────────────────────────────────────┘"""
    print('└' + '─' * (W - 2) + '┘')

def _box_row(text=''):
    """│ text                                          │"""
    pad = W - 4 - _vis(text)
    print(f'│ {text}{" " * max(0, pad)} │')

def _ts():
    return time.strftime('%H:%M:%S')

def _client_id():
    hostname = re.sub(r'[^A-Za-z0-9_-]+', '_', socket.gethostname()).strip('_')
    suffix = uuid.uuid4().hex[:6]
    return f'mqtt_monitor_{hostname or "host"}_{os.getpid()}_{suffix}'

def _color(state):
    return {
        'ok':       GREEN  + 'ok'       + RESET,
        'warning':  YELLOW + 'warning'  + RESET,
        'error':    RED    + 'error'    + RESET,
        'inactive': GRAY   + 'inactive' + RESET,
    }.get(state, state)

def _on_connect(client, userdata, flags, rc):
    global _connected
    if rc == 0:
        _connected = True
        client.subscribe('runtime/+/status')
        _print_dashboard()
    else:
        print(f'Connection failed (rc={rc})')

def _on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        return

    dev_id   = payload.get('device_id', msg.topic)
    now_mono = time.monotonic()

    if dev_id not in _history:
        _history[dev_id] = DeviceHistory()
    hist = _history[dev_id]

    # Reconnect detected
    if dev_id in _offline_at:
        offline_mono, _ = _offline_at.pop(dev_id)
        gap = round(now_mono - offline_mono, 1)
        hist.drop_count    += 1
        hist.last_reconnect = _ts()
        hist.last_gap_s     = gap
        hist.reconnect_mono = now_mono
        hist.offline_since  = None
        hist.offline_mono   = None

    _devices[dev_id]   = payload
    _last_seen[dev_id] = now_mono
    _print_dashboard()

def _check_timeouts():
    now = time.monotonic()
    for dev_id, last in list(_last_seen.items()):
        if now - last > _TIMEOUT and dev_id not in _offline_at:
            ts = time.strftime('%H:%M:%S',
                               time.localtime(time.time() - (now - last)))
            _offline_at[dev_id] = (last, ts)
            if dev_id in _history:
                _history[dev_id].offline_since = ts
                _history[dev_id].offline_mono  = last

def _is_offline(dev_id):
    return dev_id in _offline_at

def _hline(char='─'):
    print(char * W)

def _print_device_block(dev_id, now_mono):
    d       = _devices[dev_id]
    hist    = _history.get(dev_id, DeviceHistory())
    offline = _is_offline(dev_id)

    # ── Box top with device name ─────────────────────────────────────
    if offline:
        label = f'{BOLD}{RED}{dev_id}  [OFFLINE]{RESET}'
    else:
        label = f'{BOLD}{CYAN}{dev_id}{RESET}'
    _box_top(label)

    # ── Status row ───────────────────────────────────────────────────
    if offline:
        _, offline_ts = _offline_at[dev_id]
        gap_s  = round(now_mono - _last_seen.get(dev_id, now_mono), 1)
        status = f'{RED}OFFLINE{RESET}  since {offline_ts}  ({gap_s:.0f}s)'
        if hist.drop_count > 0:
            status += f'  {GRAY}[total drops: {hist.drop_count + 1}]{RESET}'
    else:
        s      = (d.get('status') or 'idle').capitalize()
        color  = GREEN if s == 'Running' else GRAY
        status = f'{color}{s}{RESET}'

        rt = d.get('runtime_seconds')
        if rt:
            status += f'  runtime {rt:.0f}s'

        bag_mb = d.get('bag_size_mb')
        if bag_mb is not None:
            status += f'  {GRAY}bag {bag_mb:.1f} MB{RESET}'

        if d.get('join_mode') == 'late_join':
            status += f'  {YELLOW}[late join]{RESET}'

        if (hist.reconnect_mono and
                now_mono - hist.reconnect_mono < _RECONNECT_SHOW_S):
            status += (f'  {YELLOW}reconnected {hist.last_reconnect}'
                       f'  gap {hist.last_gap_s}s{RESET}')
            if hist.drop_count > 0:
                status += f'  {GRAY}[drops: {hist.drop_count}]{RESET}'
        elif hist.drop_count > 0:
            status += (f'  {GRAY}drops: {hist.drop_count}'
                       f'  last gap {hist.last_gap_s}s{RESET}')

    last_upd   = (d.get('last_update') or '')[11:19]
    exp_display = d.get('experiment_id') or '—'
    master_display = d.get('source_master_device_id') or '—'
    _box_row(f'Status    : {status}')
    _box_row(f'Last seen : {last_upd}   Exp: {exp_display}')
    _box_row(f'Master    : {master_display}')

    # ── Sensor topic table ───────────────────────────────────────────
    _box_mid()
    stats = d.get('topic_stats', {})
    if stats:
        _box_row(f'{"Topic":<26}  {"ExpHz":>5}  {"ObsHz":>6}  {"Count":>9}  '
                 f'{"Age(s)":>6}  State')
        _box_row(f'{"─"*26}  {"─"*5}  {"─"*6}  {"─"*9}  {"─"*6}  {"─"*7}')
        for topic, s in sorted(stats.items()):
            age       = f'{s["last_msg_age_s"]:.1f}' if s['last_msg_age_s'] is not None else '—'
            state_str = (RED + 'offline' + RESET) if offline else _color(s['state'])
            _box_row(f'{topic:<26}  {s["expected_hz"]:>5.0f}  '
                     f'{s["observed_hz"]:>6.1f}  {s["message_count"]:>9}  '
                     f'{age:>6}  {state_str}')
    else:
        _box_row(f'{GRAY}no topic data yet{RESET}')

    _box_bot()

def _print_dashboard():
    _check_timeouts()
    now_mono = time.monotonic()
    now_str  = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    online_n  = sum(1 for k in _devices if not _is_offline(k))
    offline_n = sum(1 for k in _devices if _is_offline(k))

    running   = [d for d in _devices.values()
                 if not _is_offline(d.get('device_id',''))
                 and d.get('status') == 'running']
    exp_id    = running[0].get('experiment_id', '—') if running else '—'
    status    = 'Running' if running else 'Idle'
    rt        = max((d.get('runtime_seconds') or 0 for d in running), default=None)
    rt_str    = f'{rt:.0f}s' if rt else '—'

    os.system('clear')

    # ── Global experiment header ─────────────────────────────────────
    _hline('═')
    print(f'  ROS2 Multi-Pi Runtime Monitor')
    _hline('─')
    conn = f'{GREEN}connected{RESET}' if _connected else f'{YELLOW}connecting{RESET}'
    print(f'  MQTT       : {_broker_label}   {conn}')
    print(f'  Experiment : {exp_id}   Status: {status}   Runtime: {rt_str}')
    print(f'  Devices    : {GREEN}{online_n} online{RESET} / '
          f'{RED}{offline_n} offline{RESET} / {len(_devices)} total')
    _hline('─')
    print(f'  Updated: {now_str}   Timeout: {_TIMEOUT}s   (Ctrl+C to quit)')
    if not _devices:
        _hline('─')
        print('  Connected. Waiting for runtime/+/status data...')

    # ── One block per device ─────────────────────────────────────────
    for dev_id in sorted(_devices):
        _print_device_block(dev_id, now_mono)

    # ── Global sensor health summary (all devices, all topics) ───────
    _hline('═')
    print(f'  Sensor Health Summary  (all devices)')
    _hline('─')
    print(f'  {"Host":<26}  {"Topic":<24}  {"ObsHz":>6}  {"Count":>9}  {"Age(s)":>7}  State')
    print(f'  {"─"*26}  {"─"*24}  {"─"*6}  {"─"*9}  {"─"*7}  ─────')
    for dev_id in sorted(_devices):
        d       = _devices[dev_id]
        offline = _is_offline(dev_id)
        for topic, s in sorted(d.get('topic_stats', {}).items()):
            age       = f'{s["last_msg_age_s"]:.1f}' if s['last_msg_age_s'] is not None else '—'
            state_str = (RED + 'offline' + RESET) if offline else _color(s['state'])
            print(f'  {dev_id:<26}  {topic:<24}  '
                  f'{s["observed_hz"]:>6.1f}  {s["message_count"]:>9}  '
                  f'{age:>7}  {state_str}')
    _hline('═')

def _refresh_loop():
    while True:
        time.sleep(5)
        if _devices:
            _print_dashboard()

def main():
    global _TIMEOUT, _broker_label

    parser = argparse.ArgumentParser(description='MQTT runtime monitor')
    parser.add_argument('--broker',  default='localhost')
    parser.add_argument('--port',    default=1883, type=int)
    parser.add_argument('--timeout', default=15, type=int,
                        help='Seconds without message before OFFLINE (default: 15)')
    args     = parser.parse_args()
    _TIMEOUT = args.timeout
    _broker_label = f'{args.broker}:{args.port}'

    print(f'Connecting to {args.broker}:{args.port} ...')
    client = mqtt.Client(client_id=_client_id())
    client.on_connect = _on_connect
    client.on_message = _on_message

    try:
        client.connect(args.broker, args.port, keepalive=60)
    except Exception as e:
        print(f'ERROR: cannot connect — {e}')
        print('Make sure mosquitto is running.')
        sys.exit(1)

    threading.Thread(target=_refresh_loop, daemon=True).start()

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print('\nMonitor stopped.')
        client.disconnect()

if __name__ == '__main__':
    main()
