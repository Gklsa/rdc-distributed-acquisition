# Local review helper; run in a sourced Linux ROS 2 workspace.
# Hardware SSH, rsync destinations, site firewall rules and private network setup
# are intentionally excluded from this export.
SHELL := /bin/bash
CONFIG ?= src/devices_config/device2.example.yaml
MASTER_DEVICE ?= review_pc
RECORDINGS_DIR ?= $(CURDIR)/recordings
MQTT_HOST ?= localhost
ANALYSIS_MODE ?= mock
COLLECT_SOURCE ?=
.PHONY: help build sensors master worker start stop dashboard mock-up mock-down analysis collect
help:
	@echo "build | sensors | master | worker CONFIG=... | start | stop | dashboard | analysis"
	@echo "Mock containers: mock-up / mock-down (prepared local images required)"
build:
	colcon build
sensors:
	ros2 launch rdc_bringup sensors.launch.py
master:
	ros2 launch rdc_bringup pc_master.launch.py master_device_id:=$(MASTER_DEVICE) "recordings_dir:=$(RECORDINGS_DIR)"
worker:
	ros2 launch rdc_bringup pi_worker.launch.py "config:=$(CONFIG)" mqtt_broker_host:=$(MQTT_HOST) "recordings_dir:=$(RECORDINGS_DIR)"
start:
	ros2 service call /start_experiment std_srvs/srv/Trigger
stop:
	ros2 service call /stop_experiment std_srvs/srv/Trigger
dashboard:
	python3 src/tools/mqtt_monitor.py --broker $(MQTT_HOST)
mock-up:
	docker compose up -d
mock-down:
	docker compose down
analysis:
	ANALYSIS_MODE=$(ANALYSIS_MODE) HEADLESS_ANALYSIS=1 jupyter nbconvert --execute --to notebook --output-dir analysis_output --output recording_analysis_output src/tools/recording_analysis.ipynb
collect:
	@test -n "$(COLLECT_SOURCE)" || (echo "Set COLLECT_SOURCE=user@host:/path/to/recordings/" && exit 1)
	mkdir -p recordings
	rsync -av "$(COLLECT_SOURCE)" recordings/
