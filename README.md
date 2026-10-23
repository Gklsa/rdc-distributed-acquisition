# RDC · Distributed Sensor Acquisition

ROS 2 acquisition across independently configured devices, with local MCAP recording, shared experiment control, MQTT status monitoring and Jupyter analysis. The workspace includes Python/C++ recorder alternatives, TMP116 I²C drivers and mock temperature, IMU and camera publishers.

```text
PC coordinator -> START/STOP + shared experiment ID -> device workers
Sensor topics -> per-device Python or C++ recorder -> MCAP + JSON metadata
Device status -> MQTT broker -> terminal dashboard
Collected recordings -> Jupyter analysis
```

Recordings include command/recording timestamps, segments and code-version metadata for traceability. These timestamps alone do not establish clock-synchronization precision.

## Structure

| Path | Purpose |
|---|---|
| `src/ros2-data-collection/rdc_core/` | Coordinator, recorder alternatives and runtime status |
| `src/ros2-data-collection/rdc_bringup/launch/` | Master, worker and mock-sensor launch files |
| `src/ros2-data-collection/rdc_hw/` | TMP116 drivers and simulated sensor publishers |
| `src/devices_config/` | Generic per-device configuration templates |
| `src/tools/` | MQTT dashboard and analysis notebook |
| `Makefile` / `docker-compose.yml` | Local launch, analysis and container helpers |

Existing package metadata and in-file attribution are retained; see [source and license notes](SOURCE_NOTICE.md).

## Notebook-only demo

This path generates synthetic arrays and needs no ROS 2, robot hardware or recordings. Use Python with the notebook dependencies in `requirements.txt`. From a Linux shell in the repository root:

```bash
git clone https://github.com/Gklsa/rdc-distributed-acquisition.git
cd rdc-distributed-acquisition
python -m pip install -r requirements.txt
make analysis ANALYSIS_MODE=mock
```

The executed notebook is saved under `analysis_output/`, with plots under `recording_analysis_output/`. You can also open `src/tools/recording_analysis.ipynb` in Jupyter and run its default `mock` mode. Real-recording modes require your own bag files and matching ROS 2 Python modules.

## ROS 2 mock recording

Use a prepared Linux ROS 2 Jazzy environment with `colcon`, a C++17 compiler, the dependencies declared in the package manifests, MCAP storage support, PyYAML and `paho-mqtt`. Pip notebook requirements do not install ROS 2.

```bash
colcon build
source install/setup.bash
```

In separate terminals with the same workspace sourced:

```bash
make sensors
make master
make worker
```

The helper defaults to the mock `device2.example.yaml` and writes recordings to the workspace's `recordings/` directory. Start and stop an experiment from another sourced terminal:

```bash
make start
make stop
```

For live status, run a local MQTT broker and `make dashboard`. The Compose example provides two mock workers on one Linux host and requires an already prepared ROS 2 image and built workspace.

## Hardware and checks

Hardware deployment uses a reviewed local copy of `device1.example.yaml`, a Raspberry Pi with TMP116 I²C access and your own device endpoints. The Python TMP116 driver additionally requires `smbus2`. Keep operational configuration in ignored local YAML files. Original network settings, raw recordings and generated reports are excluded.

The included ROS test files are ament lint checks. Run them in the prepared ROS environment:

```bash
colcon test
colcon test-result --verbose
```

The notebook's synthetic mode provides an offline analysis check. A ROS 2 build, distributed recording and hardware validation require the corresponding runtime and devices.
