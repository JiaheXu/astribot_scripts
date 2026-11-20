# Astribot SDK and scripts

This repository contains the complete Astribot S1 ROS 2 SDK workspace: SDK
libraries, robot models, messages, bundled dependencies, examples, and the
observation, recording, visualization, startup, and control tools.

The checkout root is the SDK root. The Python package `astribot_scripts/` stays
one level below it so existing imports, launch files, and commands retain their
relative paths.

## Setup

The robot runtime requires Ubuntu 22.04, ROS 2 Humble, and Python 3.10. The
bundled native libraries target Linux; macOS cannot run the robot stack.

```bash
git clone git@github.com:JiaheXu/astribot_scripts.git astribot_sdk_ros2
cd astribot_sdk_ros2
./install.sh
source /opt/ros/humble/setup.bash
source ./env.sh
```

`install.sh` installs system and Python dependencies and configures the local
SDK directories. Source ROS and `env.sh` in each new terminal. `env.sh` derives
paths from its own location and configures ROS domain 25. The DDS interface
profile is generated locally from `config/fastdds_whitelist_192.xml.template`.

For the consolidated local folder, the checkout is
`astribot_sdk_ros2/astribot_scripts/`: change into that directory before sourcing
`env.sh`. For a fresh clone, using `astribot_sdk_ros2` as the destination above
matches the paths in the deployment guides. An existing checkout should also
be treated as the SDK root after pulling this update; update any desktop
shortcuts or services that used its former parent as the SDK root.

## Layout

```text
<checkout>/
├── env.sh, install.sh       # Environment and installation
├── astribot_sdk/            # SDK Python code and native libraries
├── astribot_msgs/           # ROS 2 interfaces and bundled message support
├── astribot_config/         # Robot model and calibration assets
├── third_party/             # Bundled middleware, Drake, and dependencies
├── examples/                # Original SDK examples
├── config/, tools/          # DDS configuration and SDK utilities
├── main.py                  # SDK application
├── camera_raw_node.py       # Camera streaming
├── lidar_node.py            # LiDAR streaming
├── lidar_head_stream.py     # Head LiDAR streaming
└── astribot_scripts/        # Commands, common code, launch/config/docs/tests
```

The repository includes native libraries and model assets, so the initial
clone is larger than the former scripts-only checkout. Python bytecode, local
caches, generated DDS configuration, recordings, and build output are ignored.

## Commands and documentation

Run commands from the checkout root after loading the environment:

```bash
python3 -m astribot_scripts.publish_robot_data --help
./astribot_scripts/launch_all_tmux.sh check
```

The launch check also requires the patched FAST-LIO workspace described in the
scripts guide. Observation tools do not issue motion commands; startup and
power commands can move the robot, as documented in that guide.

- [Scripts guide](astribot_scripts/README.md): command reference and operating procedures.
- [SDK guide](README_sdk.md) / [中文 SDK 指南](README_zh.md).
- [Sensor streaming](SENSOR_STREAMING.md).
- [FAST-LIO and RViz](astribot_scripts/docs/fast_lio.md).
- [Navigation](astribot_scripts/docs/nav2.md).
- [RGB-D point clouds](astribot_scripts/docs/rgbd_pointcloud.md).
- [FoundationPose integration](astribot_scripts/integrations/foundationpose/README.md).

## Tests

With the ROS 2 and SDK environments loaded, run the existing tests from the
checkout root. SDK connections are mocked; these tests do not require a robot.
Some tests use ROS message packages and Linux utilities such as `flock`.

```bash
python3 -m unittest discover -s astribot_scripts/tests -p 'test_*.py'
```

The SDK license is preserved in [LICENSE](LICENSE). Bundled dependencies retain
their own license files under `third_party/`.
