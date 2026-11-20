# Astribot scripts

Command-line tools for S1 observation, sensor/state publishing, recording, RViz,
startup, and ROS command reception. Observation tools issue no joint/motion
commands. `startup.py` commands home motion; `robot_power.sh` separately sequences power, activation
and home. Recording scripts write ROS bags without controlling the robot.

The [repository setup guide](../README.md) covers the bundled SDK and clone layout.
Run the commands below from the checkout root containing `env.sh`.

## Layout

```text
astribot_scripts/
├── *.py                 # Existing executable commands (including numbered examples)
├── record_robot_data.sh # ROS bag recorder
├── record_raw_images.sh # Four RGB and four depth topics at 3 Hz, MCAP with 4 GB file splitting
├── robot_power.sh       # Separate power/activation/home startup and shutdown
├── common/
│   ├── cli.py           # Shared argument validation
│   ├── control.py       # Shared command receiver loop and SDK connection
│   ├── sensors.py       # Image/depth conversion and sensor publishing
│   └── state.py         # S1 model, joint mapping and pose markers
├── launch/              # ROS 2 launch files
├── config/              # RViz settings, recording QoS and calibration template
├── docs/                # Camera and IMU mounting references
└── tests/               # Offline unit tests; SDK connections are mocked
```

Executable filenames and flags are preserved. The old root-level launch,
RViz and YAML paths are relative symlinks to their canonical locations, so
existing commands and calibration-file edits still work. Preserve symlinks
when copying this folder. The `examples/` originals and SDK live alongside this
package at the checkout root.

Import reusable code from `astribot_scripts.common`; executable scripts retain
previously exposed helper names for compatibility. Source `env.sh` before
running commands so the repository root is on `PYTHONPATH`. No package install
or colcon build is required. Named commands also support module execution,
for example `python3 -m astribot_scripts.publish_robot_data --help`.

## Receive mobile-base and body-joint commands

`control_mobile_base.py` follows `examples/202-chassis_joy_control_local.py`:
it receives `geometry_msgs/msg/Twist` on `/cmd_vel` and integrates local chassis
velocities into SDK position targets. `linear.x/y` use m/s and `angular.z` uses
rad/s; other components must be zero. Planar speed magnitude is limited to
0.3 m/s and yaw rate to 0.6 rad/s by default. After 0.5 seconds without a valid
command, it stops advancing the base target and continues holding that target.
Invalid commands also cancel the previous velocity. Change these settings with
`--max-linear-speed`, `--max-angular-speed`, and `--timeout`.

`control_joints.py` follows `examples/104-set_joint_position.py`: it receives
`sensor_msgs/msg/JointState` on `/astribot/joint_cmd`, using `name` and `position`
in radians. Leave `velocity` and `effort` empty. Supported names are:

| Body part | Joint names |
| --- | --- |
| Torso | `astribot_torso_joint_1` through `astribot_torso_joint_4` |
| Left arm | `astribot_arm_left_joint_1` through `astribot_arm_left_joint_7` |
| Right arm | `astribot_arm_right_joint_1` through `astribot_arm_right_joint_7` |
| Head | `astribot_head_joint_1`, `astribot_head_joint_2` |

Names can be in any order and messages can address a subset. When a body part
is first commanded, its unspecified joints keep their SDK desired positions;
later messages update only the named joints. Position targets remain active
until replaced, so a single message is sufficient. These are immediate position
targets, not timed trajectories; `header` is not used for scheduling. Non-finite
values, duplicate/unknown names, mismatched arrays, and targets outside the SDK
joint limits reject the entire message. Chassis and gripper commands are not
accepted by the joint receiver.

On the receiving PC, with the robot already powered and activated, source ROS
and `env.sh` in each terminal. Run **one** of these commands from the SDK root:

```bash
source /opt/ros/humble/setup.bash
source ./env.sh

# Base only:
python3 -m astribot_scripts.control_mobile_base

# Body joints only:
python3 -m astribot_scripts.control_joints

# Both receivers through one SDK control owner:
python3 -m astribot_scripts.control_mobile_base --with-joints
```

The SDK permits only one control owner, so use `--with-joints` when controlling
both; do not run the two standalone receivers concurrently. Normal SDK control
acquisition applies, including its prompt if another controller owns the robot.
If stdin is closed or unavailable, the receivers decline that takeover prompt.
Acquisition itself restarts SDK control and resets desired positions to current
positions. These scripts do not power/activate the robot or command a home move.
They use filtered position control at 250 Hz with WBC disabled and automatic torso
insertion disabled, so base-only and head-only messages do not add a torso target.
Commands are dispatched on one loop, with a queue of one per topic; no targets
are sent until the first valid message. Ctrl-C/SIGTERM requests SDK `stop_robot()`
if motion has been commanded and this process still owns control. Loss of control
rights or robot feedback exits the loop. Base timeout is a software target hold,
not an independent hardware emergency stop.

Example publishers in another sourced terminal (these commands move the robot):

```bash
# Stream a slow forward base command; stopping the publisher triggers the timeout.
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Set the two head joint targets once.
ros2 topic pub --once /astribot/joint_cmd sensor_msgs/msg/JointState \
  '{name: [astribot_head_joint_1, astribot_head_joint_2], position: [0.1, 0.0]}'
```

Use `--topic` to change either receiver's input topic, `--joint-topic` to change
the joint topic in combined mode, and `--rate` to change the SDK loop frequency.
Both scripts also support `python3 astribot_scripts/<script>.py` after sourcing
`env.sh`. Offline receiver tests require only Python's standard library:

```bash
python3 -m unittest astribot_scripts.tests.test_command_receivers -v
```

## Start all sensors, rear FAST-LIO and RViz in tmux

For Nav2 base navigation alongside this observation stack, see
[`docs/nav2.md`](docs/nav2.md). `start_nav2.sh` launches a Humble holonomic Nav2
profile, rear-LIO navigation feedback/TF bridge, freshness gate, and the existing
SDK base receiver. Preview with `enable_control:=false`; `with_joints:=true`
shares one SDK owner with the body-joint receiver. Navigation has its own TF
topics and does not use the display-time transform described below.

On the receiving PC, with the robot's normal controller already running, open a
desktop terminal and run:

```bash
cd /home/fourier/astribot_sdk_ros2
tmuxp load -L astribot-lio ./astribot_scripts/config/astribot.tmux.yaml
```

The **tmuxp YAML** defines all startup commands directly. Each pane sources ROS
Humble, this SDK's `env.sh` and `$HOME/ros2_ws/install/local_setup.bash`
automatically. Paths resolve relative to the YAML file. The dedicated tmux server
and session are both named `astribot-lio`, with three panes:

1. **Sensors:** one observation-only SDK connection for head, torso, left-wrist
   and right-wrist color images, plus lidars, IMUs, joints and poses. The YAML
   selects `--cameras head_rgbd torso_rgbd left_wrist_rgbd right_wrist_rgbd
   --no-depth --color-encoding rgb8 --image-rate 3 --image-size 640 480`: four
   uncompressed `sensor_msgs/msg/Image` streams in RGB order, each resized to
   640×480 and published at up to 3 Hz, without depth or stereo republishing.
2. **Rear FAST-LIO:** the existing rear profile using `/livox/lidar_back` and
   `/livox/imu_back`, with `rviz:=false`.
3. **One combined RViz:** `launch/astribot_lio_rviz.launch.py` loads
   `config/astribot_fast_lio.rviz`: FAST-LIO odometry, trajectory and map view,
   the articulated robot, current/desired pose markers, and the four RGB images.
   The map view accumulates 60 seconds of registered scans, so it remains bounded;
   it is not a saved/global map. No second SDK state publisher is started.

The combined view also starts a head/torso RGB-D adapter and includes two RGB
point-cloud displays. It samples native depth separately from the RGB sensor
pane. **Depth calibration/units/registration must be completed before clouds
can appear**; the supplied RGB intrinsics are calibrated, but the depth fields
remain unset pending driver verification. See [RGB-D point clouds](docs/rgbd_pointcloud.md)
for calibration, topics, timing, and adding clouds to an already-running view.

This launcher never runs `startup.py` or home/motion commands. Its observation-only
connection bypasses the SDK's automatic control acquisition, even when no owner
exists, so it does not trigger the control-acquisition restart/desired-position
reset. Sensor activation services still run; the normal robot controller remains
responsible for the robot. The SDK's generic initialization logs may say
"acquired control rights"; the publisher then explicitly reports that acquisition
is disabled. SDK source files are unchanged.

The robot base is **`astribot_viz_chassis`**. The shared visualization tree has
`map -> astribot_viz_chassis`, with fixed children `body` (rear IMU) and
`astribot_torso_base` (parent of articulated links), carried on `/astribot_viz/tf`
and `/astribot_viz/tf_static`. The fixed chassis-to-IMU link
uses nominal robot-description mounts and the SDK's 0.097 m torso weld offset;
it is for visualization, not measured chassis calibration or navigation fusion.
`fast_lio_display_tf.py` publishes chassis-centered odometry and trajectory on
`/fast_lio/chassis/odometry` and `/fast_lio/chassis/path`, preserving source
timestamps and the `map` world origin. FAST-LIO uses `common.world_frame: map`
for world clouds, odometry, path and TF. RViz's fixed frame, grid and orbit target
are all `map`, so the view stays in the environment as the robot moves. The map
origin is established when FAST-LIO initializes and resets when mapping restarts.
The node broadcasts the latest chassis pose at current display
time because the live sensor timestamps lag SDK joint polling. It stops updating
after two seconds without new odometry. Original odometry, cloud and sensor-TF
timestamps remain unchanged; this is not temporal sensor fusion.
The model follows FAST-LIO without driving the robot. See
[`docs/fast_lio.md`](docs/fast_lio.md#one-rviz-on-the-receiving-pc).

RViz shows **Chassis odometry history (1000 poses)** as small axes, adding poses
after at least 5 cm of translation or 0.1 rad of rotation. The white **Robot
chassis trajectory (session history)** line shows the cumulative chassis path
from the current FAST-LIO run. Reloading the visualization retains that path
because the mapper republishes its history; restarting FAST-LIO resets it.

With `start_nav2.sh` running, the RViz **2D Goal Pose** tool (**G**) sends a
click-and-drag map target and heading to Nav2. **Publish Point** (**U**) selects
X/Y from the point cloud and preserves the current heading. The **Click to
navigate** display group shows the target, result/status label, and planned path.
Reload `config/astribot_fast_lio.rviz` through **File → Open Config** if the view
was already open. Keep Fixed Frame `map`; coordinate conversion is automatic.
See [goal interaction and cancellation](docs/nav2.md#click-a-target-in-the-existing-combined-rviz).

The RViz **Robot joint axes** group shows axes at the chassis, torso joints 1–3,
chest (torso joint 4), head pan/tilt joints, and left/right wrist joint 7. Each
uses the corresponding URDF child-link frame and can be toggled independently;
the existing current/desired Cartesian pose markers remain separate displays.

The **Camera optical frames** group adds head, torso and left/right wrist RGB
camera axes. The generated model attaches `astribot_viz_<camera>_optical` to each
moving link using `config/camera_frames.yaml`. These are nominal robot-description
mounts with ROS optical axes: X right, Y down, Z forward. They do not change image
headers or supply measured camera calibration.

The **AprilTag detections** group draws RGB axes at the center of **tag36h11,
ID 1, side 60 mm**, as configured in the detector's `config/tags.yaml`. The
combined launch starts detection automatically when available (`apriltags:=auto`).
For an already running view, use `launch/astribot_apriltag.launch.py`, then reload
the RViz config. Detector and axes use the same tag configuration. See
[AprilTag setup, timing, and axis display](docs/apriltags.md).

### Desktop shortcuts

For the operator walkthrough, open the desktop **Astribot Button Guide** shortcut
or [read the offline HTML guide](docs/desktop-buttons.html). It covers all four
action buttons, startup/recording order, MCAP topics and output files, graceful
recording stop, separate robot shutdown, and troubleshooting logs.

Install the shortcuts on the receiving PC (also replaces the old Restart icon):

```bash
python3 astribot_scripts/install_desktop_launcher.py
```

Click **Start Astribot** (green play icon) to start the stack. If `astribot-lio`
already exists, Start stops it gracefully and reloads `config/astribot.tmux.yaml`.
This retains the old Restart behavior and begins a fresh map and trajectory.
Click **Stop Astribot** (red square icon) to stop the sensor publisher, FAST-LIO,
robot visualization and RViz, then remove this tmux session. Stop also succeeds
if the stack is already stopped. It leaves the robot controller, camera/lidar
drivers, unrelated tmux sessions and OBS alone; it sends no robot motion commands.
Repeated Start clicks during an action are ignored. If Stop is clicked during
startup, it queues behind that action and then shuts the stack down.
**Record RGB-D MCAP** (red circle icon) opens a separate recording terminal for
the four RGB and four depth topics at 3 Hz each. Press Ctrl-C there to finalize the recording.
The Start action checks dependencies and the desktop first, then waits for the three
panes and their sensor, mapping and RViz processes. It reports success only after
receiving fresh joint states, chassis odometry, map clouds, all four RGB images
and the robot/camera transforms rooted in `map`. Sensor activation allows 30
seconds for each control service to be discovered, using a persistent client.
This starts a fresh FAST-LIO map. Missing data produces a failure notification
instead of reporting an empty RViz as ready. Details are in
`~/.local/state/astribot/start.log` and `~/.local/state/astribot/stop.log`.

The same actions are available from a terminal or SSH:

```bash
python3 astribot_scripts/start_astribot.py
python3 astribot_scripts/stop_astribot.py
```

`restart_astribot.py` remains available as a compatibility entry point for Start.

```bash
tmuxp load -L astribot-lio -d ./astribot_scripts/config/astribot.tmux.yaml  # detached
tmux -L astribot-lio attach -t astribot-lio

# The earlier shell helper can also inspect/stop a session loaded from YAML:
./astribot_scripts/launch_all_tmux.sh check                 # no robot connection
./astribot_scripts/launch_all_tmux.sh attach
./astribot_scripts/launch_all_tmux.sh status
./astribot_scripts/launch_all_tmux.sh stop
```

For SSH/headless operation, set `environment.ASTRIBOT_RVIZ` to `"false"` in the
YAML before loading it. Install the YAML loader with `sudo apt install tmuxp`
(already installed on the receiving PC). If the named session already exists,
tmuxp offers to attach to it; it does not rebuild its panes. To apply YAML edits,
stop the old session first, then load the YAML again. Do not launch the same stack
under a different socket/session while the existing one is active.

Use **Ctrl-b, then d** to detach while processes keep running. Re-running `start`
reuses the existing session. Failed/exited panes remain visible for inspection;
other panes continue running. Use `stop`, then start again to restart the stack
or change GUI mode. `stop` sends Ctrl-C to all panes and waits up to 15 seconds
for cleanup; if any remain alive it leaves them available for inspection.
Camera/lidar drivers remain active after stopping, and no bag is recorded.

GUI mode requires an active desktop login on the receiving PC. The YAML uses
`desktop_exec.py` to discover that user's Xwayland display and authority, verify
access with `xdpyinfo`, and open RViz there even when launched over SSH. It does
not use SSH X11 forwarding or open RViz on the Mac. It requires `tmuxp`, `tmux`, `rviz2`,
`robot_state_publisher`, the SDK Python dependencies and the patched FAST-LIO
already built in `ros2_ws`. Override `ROS_SETUP` or `ROS2_WS` for other locations.

## Separate robot power and home automation

The desktop **Start Astribot / Stop Astribot** buttons remain observation-only.
They do not run the commands below. Use `robot_power.sh` separately on the
receiving PC; it sources ROS Humble and the SDK environment automatically.

The same desktop installer also adds **Power On & Home Robot** (yellow power
symbol). Clicking it immediately runs `robot_power.sh start --execute` in a
terminal: power on, wait for confirmation, activate, wait for confirmation,
settle for five seconds, select PC control and confirm the web controller has
released its SDK ownership, then home the robot and open the grippers.
Each click saves output to `~/.local/state/astribot/robot-start-*.log`.
The terminal stays open after success
or failure; press Enter to close it. It does not launch the observation stack.
The existing robot-power lock prevents overlapping startup/shutdown sequences.

```bash
# Passive status and monitoring: no web client/control acquisition.
./astribot_scripts/robot_power.sh status
./astribot_scripts/robot_power.sh watch --duration 15

# Preview the sequence; these do not send motor commands or initialize the SDK.
./astribot_scripts/robot_power.sh start
./astribot_scripts/robot_power.sh stop

# Execute real robot operations when the workspace is clear.
./astribot_scripts/robot_power.sh start --execute
# ...run the observation stack / perform work...
# Stop the observation stack with its separate desktop Stop button, then:
./astribot_scripts/robot_power.sh stop --execute
```

Startup runs **power on → confirm power → activate → confirm drives → settle
5 seconds → WEB-to-PC handoff → confirm control released → SDK home → confirm
home**. Power/activation commands are skipped if already complete.
The additional settling period keeps monitoring fresh, healthy diagnostics;
`--activation-settle` adjusts its duration. The vendor web controller normally
claims SDK control when drives activate, so waiting alone does not release it.
The startup script uses the vendor `set_control_source` command to select `PC`,
then requires both passive PC-mode feedback and absence of SDK control owners
before launching the home helper. It refuses VR/joystick/drag modes and any
owner other than the vendor `/web_astribot_<pid>` node. The home helper still
checks for unowned control independently; it never forces a generic takeover.
Homing moves the torso, both arms, head and grippers, opening the grippers.
Keep people/objects clear and do not run it while a gripper is holding something.
The SDK's existing collision checks and control-rights rules remain in force;
noninteractive homing refuses an existing SDK control owner instead of seizing it.

Shutdown runs **deactivate → confirm inactive → power off → confirm off**.
It does not force power off if deactivation times out or diagnostics disappear.
Already-off is a successful no-op. The configured endpoint is
`192.168.0.10:5141`; `--host`, `--port` and `--timeout` override address/transition
timeout. Startup also uses the passive WebSocket at `192.168.0.10:5143/ws`
(`--control-port` overrides 5143) and requires `aiohttp`, already installed on
the receiving PC. It does not register a web UI client. A shared
`~/.local/state/astribot/robot-power.lock` prevents concurrent
robot-power/home sequences. Do not run manual UI controls simultaneously.

Commands use the verified web UI JSON protocol, but HTTP success only means
the command was dispatched. Completion uses fresh `/astribot/system_dsm_result`
messages, with at least three samples and two stable seconds. Power-on includes
the vendor's 10-second off-to-on cooldown. Faults, stale data and unconfirmed
transitions stop the sequence. Failures do not trigger automatic shutdown or
blind command retries. Neither script releases brakes or clears fault codes.

The home helper has a 45-second process limit and receives Ctrl-C on timeout so
it can request an SDK stop. If it cannot exit, it retains the robot-power lock;
inspect the robot and helper before another action. These checks are software
sequencing, not a hardware emergency-stop system.
The parent also requires a fresh completion marker written only after measured
home verification; an SDK process exiting with status 0 is insufficient.

Protocol/schema provenance: [integrations/astribot_web/README.md](integrations/astribot_web/README.md).
`robot_power.py` is also usable directly after sourcing the environment.

## Move an activated robot home

After powering on and activating the robot through the existing interface, source
the ROS/SDK environment and run:

```bash
python3 astribot_scripts/startup.py
```

This script calls the SDK's `move_to_home()` with its existing collision checks.
It retains the normal SDK control-rights prompt; motion requires control rights.
It does not automate power-on or robot activation, and it does not silently take
control from another controller. The home pose moves torso, both arms, head and
both grippers (opens them); it does not move the chassis.

The completion window is **10 seconds after connection/control acquisition**.
The unmodified SDK caps home-motion duration at about **5 seconds**, so requesting
10 seconds does not produce a 10-second trajectory. The native motion call blocks;
the deadline can only be checked once it returns, not enforced as an independent
watchdog while that call is running.

After the call returns, the script checks current joint positions against the
SDK home target every 0.1 seconds. Tolerances are 0.02 rad for torso/arms/head and
2 SDK opening units for grippers. These are feedback checks using the SDK's
reported state, not independent sensing or source-timestamp validation.
Successful verification prints:

```text
[STARTUP] Start up process is done. Home position confirmed.
```

Rejected motion, lost feedback/control or an expired deadline prints an error and
exits nonzero. After a motion request, failures and Ctrl-C attempt `stop_robot()`
only if control is still owned. A failed stop request is also logged. No successful
completion message is printed for a failed check. The SDK source is unchanged.

## Run on the receiving computer

Start the robot's normal driver/controller first. In **each terminal**:

```bash
cd /home/jiahe/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
source env.sh
```

The current `env.sh` sets `ROBOT_TYPE=S1`, `ROS_DOMAIN_ID=25`, Fast DDS, and a
`192.168.0.x` interface whitelist when that subnet is present. The computer and
robot must have compatible ROS discovery/network settings. Use the SDK's Python
environment (the bundled middleware targets Python 3.10).

**Camera (300), lidar (301), `get_lidar_and_img.py`, `get_joints_and_poses.py`, and `publish_robot_data.py` automatically answer the SDK prompt with Enter.**
Only during `Astribot()` construction, these custom scripts temporarily replace
`input()` with an empty response. Normal input is restored afterwards, including
when initialization fails. No SDK source changes are required. The SDK may still
print its prompt, but it does not wait for a keypress. Once initialization finishes,
the scripts subscribe to the sensor messages published by the robot's drivers.

The other scripts still use the SDK's default control-rights handling. When another
controller is running, press **Enter**, not `yes`, at their control-rights prompt
to continue without taking control. Without `--observe-only`, including in 300, 301 and `get_lidar_and_img.py`,
the unchanged SDK can still claim control automatically if no owner is found.
The automatic Enter response declines takeover when another owner is present;
it does not disable acquisition altogether. SDK background services still run. See
`astribot_sdk/core/astribot_api/astribot_interface.py::acquire_control_rights`.

Run the following in separate terminals as needed:

```bash
# Live color images: default is head_rgbd; select additional cameras explicitly.
python3 astribot_scripts/300-get_images.py --cameras head_rgbd left_wrist_rgbd right_wrist_rgbd
# For a terminal without a display, add --headless (counts, no JPEG decoding).

# Both complete PointCloud2 streams; prints scan/point counts and receive rate.
python3 astribot_scripts/301-get_lidar_scan.py
# Optional bounded session:
python3 astribot_scripts/301-get_lidar_scan.py --duration 60

# All parts: current and desired joint positions, velocities and torques.
python3 astribot_scripts/101-get_joint_states.py --rate 5

# Both arms: current and desired EE poses, with the same reference frame.
python3 astribot_scripts/102-get_cartesian_states.py --rate 5 --frame chassis
```

For joint and Cartesian data in a single terminal and SDK connection:

```bash
python3 astribot_scripts/get_joints_and_poses.py
python3 astribot_scripts/get_joints_and_poses.py --rate 5 --frame world
```

This combines the getters from examples 101 and 102. It prints current and desired
joint positions, velocities and torques for every part, plus current and desired
Cartesian poses of the chassis, torso, both arms (EE poses) and head. Gripper
openings remain in the joint tables and are excluded from the 7D pose tables.
Both current and desired poses use the selected `--frame` (default `chassis`).
Pose columns are `[x, y, z, qx, qy, qz, qw]`, with position in meters.
Refresh defaults to 1 Hz; stop with Ctrl-C. The script reads and prints only: it
does not publish topics, save files or issue motion commands. The SDK's existing
startup/control-rights behavior and state freshness limitations still apply.

## Publish and record sensors, IMUs and robot state together

Run from the repository root after sourcing ROS and `env.sh` as above:

```bash
python3 astribot_scripts/publish_robot_data.py
```

This replaces the separate image/lidar and joint/pose processes with **one SDK
connection**, no GUI and no motion commands. It activates camera/lidar drivers
without resetting them and leaves them running on exit. Add `--observe-only` to
bypass SDK control acquisition (always enabled by the tmux launcher); otherwise
the SDK control-rights behavior described above still applies. All five color cameras are selected by
default; only head depth has a configured decoder. Joint/pose polling defaults
to 50 Hz; sensors publish when messages arrive. To reduce image bandwidth:

```bash
python3 astribot_scripts/publish_robot_data.py --cameras head_rgbd --rate 30
```

| Topic | Type / contents |
| --- | --- |
| `/sdk/astribot_camera/<camera>/color_image` | `sensor_msgs/msg/Image`, BGR8 |
| `/sdk/astribot_camera/head_rgbd/depth_image` | `sensor_msgs/msg/Image`, 16UC1, default 640×576 little endian |
| `/sdk/astribot_lidar/{front,back}_scan` | Complete `sensor_msgs/msg/PointCloud2`, including original time fields |
| `/sdk/astribot_lidar/{front,back}_imu` | `sensor_msgs/msg/Imu` from `/livox/imu_front` and `/livox/imu_back` |
| `/sdk/astribot_chassis/imu` | `sensor_msgs/msg/Imu` from `/astribot_whole_body/imu_sensor/astribot_chassis_imu_site` |
| `/astribot_viz/joint_states` | Current `sensor_msgs/msg/JointState`: 20 model joints, position/velocity/effort |
| `/sdk/desired_joint_states` | Desired `sensor_msgs/msg/JointState`, same model ordering |
| `/sdk/cartesian/{current,desired}/<part>` | `geometry_msgs/msg/PoseStamped` in `astribot_viz_chassis` |
| `/sdk/chassis/{current,desired}_pose` | `geometry_msgs/msg/PoseStamped` in the separate `astribot_sdk_world` frame |
| `/astribot_viz/{current,desired}_poses` | Existing `visualization_msgs/msg/MarkerArray` for RViz |
| `/sdk/robot_state` | `std_msgs/msg/String` JSON: all 25 SDK values for current/desired position, velocity and torque |
| `/sdk/robot_description` | Transient-local `std_msgs/msg/String`: generated S1 URDF |

For the four-color-camera setup used by tmux, the color output encoding is
`rgb8` (`--color-encoding rgb8`), at 3 Hz per camera and 640×480 pixels
(`--image-rate 3 --image-size 640 480`). The combined publisher also defaults to
this rate and size. It caches only the newest compressed frame per camera,
decodes/resizes at each publication tick, and does not repeat stale images.
Source timestamps and frame IDs are preserved; resizing uses the exact requested
width/height, without cropping or padding. These are RViz Image displays; no
resized CameraInfo calibration is published. `--no-depth` suppresses depth
subscription and output. The standalone `get_lidar_and_img.py` retains `bgr8`,
source dimensions/rate and head depth by default; it accepts the same options.
Raw here means uncompressed `Image` messages decoded from the driver's existing
`/astribot_camera/<camera>/color_compress/compressed` streams. Those inputs must
have live publishers; the converter cannot restore a missing camera compressor.

Cartesian parts are `astribot_chassis`, `astribot_torso`, `astribot_arm_left`,
`astribot_arm_right`, and `astribot_head`. The JSON snapshot has `schema_version`,
`timestamp_basis`, `poll_started`, `poll_finished`, and
`states.<current|desired>.<position|velocity|torque>.<part>` arrays. It retains
gripper opening values in SDK units, without pretending they are angular model
joints. Chassis position is `[x, y, yaw]`; the 20 model joints use rad, rad/s and Nm.

Sensor headers, IMU values and covariances are preserved, including empty frame
IDs or unknown-orientation flags. No IMU frame, extrinsic calibration or orientation
is invented. Joint and pose stamps are **polling times**, and separate SDK getters
are not an atomic snapshot or proof of fresh measurements. The SDK world frame is
not aligned to KISS-ICP by this publisher. IMU counters in the terminal should
increase for all three streams; a discovered topic alone does not prove receipt.

In a second sourced terminal, record to a new bag directory:

```bash
bash astribot_scripts/record_robot_data.sh ./robot_data_run01
# Ctrl-C to finish, then inspect counts/types:
ros2 bag info ./robot_data_run01
```

The recorder includes every `/sdk/` topic, the three RViz state topics, the original
front/rear Livox clouds and IMUs, and the original chassis IMU. Raw Livox topics
keep existing KISS-ICP input names usable on replay; their `/sdk/` copies duplicate
those data intentionally. The QoS file captures the latched URDF even if recording
starts later. All camera streams are uncompressed here, so check disk space and
received counts. The publisher itself never starts recording automatically.

### Record RGB and depth images at 3 Hz per topic as MCAP

On the receiving PC, start the observation stack, then run:

```bash
cd /home/fourier/astribot_sdk_ros2
./astribot_scripts/record_raw_images.sh ./recordings/raw_rgbd_run01
# Ctrl-C finalizes the bag. Inspect the complete split recording with:
ros2 bag info ./recordings/raw_rgbd_run01
```

Alternatively, click the desktop **Record RGB-D MCAP** icon. It opens a terminal
and uses a new timestamped folder under `/home/fourier/astribot_sdk_ros2/recordings/`.
Press Ctrl-C in that terminal to stop and finalize the files. Repeated clicks
cannot start duplicate raw-image recorders. The icon does not power the robot
or launch the observation stack; start the publishers separately.

Omit the output argument to create a new `recordings/raw_rgbd_YYYYMMDD_HHMMSS/`
folder. The recorder subscribes directly to the driver feeds for `head_rgbd`,
`torso_rgbd`, `left_wrist_rgbd` and `right_wrist_rgbd`. Every third of a second
it saves the latest new frame from each stream under these bag topic names:

- `/sdk/astribot_camera/<camera>/color_image` — 640×480 `rgb8`.
- `/sdk/astribot_camera/<camera>/depth_image` — little-endian `16UC1` at native
  resolution (head 640×576, torso 640×480, wrists 640×360).

All eight topics use `sensor_msgs/msg/Image` and retain the original camera
timestamps and frame IDs. A missing input leaves a gap on that topic. The
terminal reports RGB and depth counts for each camera.

Storage is explicitly **MCAP**, targeting files below **4,000,000,000 bytes
(decimal 4 GB)**, producing numbered `.mcap` files plus one `metadata.yaml`
for the recording directory. rosbag2 rolls at **3,990,000,000 bytes**, reserving
10 MB for its final image/chunk and indexes so the configured image sizes
stay below 4 GB. If image dimensions/message sizes are increased, reassess this
headroom. MCAP chunk compression
is disabled, and ordinary indexes/summaries are retained. This is a separate
recorder process; neither desktop button starts it automatically. Stop recording
with Ctrl-C before shutting down the observation stack.

For live robot-model visualization alongside the combined publisher:

```bash
ros2 launch ./astribot_scripts/launch/astribot_rviz.launch.py publish_state:=false
```

For offline model/FK replay, stop live publishers and use an isolated ROS domain
in both replay terminals:

```bash
export ROS_DOMAIN_ID=125
ros2 launch ./astribot_scripts/launch/astribot_rviz.launch.py publish_state:=false use_sim_time:=true
# Second terminal, same domain:
ros2 bag play ./robot_data_run01 --clock
```

The launch rebuilds model TF from recorded joint states; TF need not be recorded.
Keep the SDK model/config/mesh files with your dataset: the URDF message contains
file references, not embedded meshes. Run the existing rear KISS-ICP setup against
`/livox/lidar_back` with `use_sim_time:=true` on replay to regenerate odometry and
the local map. It does not consume these IMUs; they are recorded for later use.
Adding `odometry:=true` to the RViz launch still requires the verified
cloud-to-chassis calibration described below. Recording additional streams does
not resolve that calibration, cross-PC clock offsets, camera intrinsics or depth
units; calibrated IMU fusion and RGB-D reconstruction need those separately.

## FAST-LIO2 odometry

The rear MID-360 / rear IMU integration, required timestamp-aware ROS 2 patch,
configuration, build commands and replay checks are documented in
[docs/fast_lio.md](docs/fast_lio.md). Start it with
`launch/astribot_fast_lio.launch.py` after building the patched package. Rear is
the default; use `sensor:=front` for the earlier front profile. Rear odometry and
TF are isolated under `/fast_lio/rear/`. The rear profile uses the robot-description
poses to restore the assumed front-frame rear cloud to native rear-lidar
coordinates before applying the nominal IMU mount. This is a model-based setup,
not measured per-robot calibration. The existing KISS adapter remains separate.

## Robot model and all Cartesian poses in RViz2

For nominal camera offsets relative to head, torso and wrist joint/link frames,
see [camera_mounts_reference.md](docs/camera_mounts_reference.md). It records official
S1 MJCF sources, composed wrist offsets and the camera-to-ROS optical-axis
conversion. The generated visualization URDF publishes the four active RGB mounts
as nominal optical frames; they are not measured camera calibration.
For the chassis and both LiDAR IMUs, see [imu_mounts_reference.md](docs/imu_mounts_reference.md),
including poses composed into the model's chassis frame and ROS-order quaternions.

On the computer with a display, source ROS and `env.sh` as above, then run from
the repository root:

```bash
ros2 launch ./astribot_scripts/launch/astribot_rviz.launch.py
# Optional refresh rate (default 10 Hz):
ros2 launch ./astribot_scripts/launch/astribot_rviz.launch.py rate:=20
```

No colcon build is needed. This starts `rviz_state.py`, `robot_state_publisher`
and RViz2 with `config/astribot_poses.rviz`. It requires the ROS Humble packages
`rviz2`, `robot_state_publisher`, and the existing SDK environment. The robot
must already be powered on and activated. Stop the launch with Ctrl-C.

The model follows **current** positions of 20 joints: torso (4), left arm (7),
right arm (7), and head (2). Labeled axes show **current and desired** Cartesian
poses for the chassis, torso, both arm end effectors, and head. Red/green/blue
arrows indicate each pose's local X/Y/Z axes; current axes are opaque and desired
axes are shorter and translucent. Toggle the two pose displays independently
in RViz's Displays panel. Gripper openings are not Cartesian poses, and the
bundled model has no movable gripper or chassis-wheel joints to animate.

| Topic | Standard ROS 2 type |
| --- | --- |
| `/astribot_viz/joint_states` | `sensor_msgs/msg/JointState` |
| `/astribot_viz/current_poses` | `visualization_msgs/msg/MarkerArray` |
| `/astribot_viz/desired_poses` | `visualization_msgs/msg/MarkerArray` |

The fixed frame is `astribot_viz_chassis`: this view follows the chassis, rather
than showing world navigation. All Cartesian getters explicitly use `chassis`.
The model uses the shipped S1 URDF and its configured torso mounting height;
mesh paths are resolved on the computer running the launch file. TF and robot
description use `/astribot_viz/` topics to avoid mixing with the driver's TF tree.
The SDK and original model files are unchanged.

For a **headless robot-connected PC and a separate RViz desktop**, keep a copy
of this repository on both computers. Source ROS/network settings on both;
they must share the ROS domain and be able to discover each other. Run:

```bash
# Robot-connected PC: SDK state only, no Qt or GUI.
python3 astribot_scripts/rviz_state.py

# Desktop, from its own repository copy: local meshes/model and RViz, no SDK connection.
ros2 launch ./astribot_scripts/launch/astribot_rviz.launch.py publish_state:=false
```

The two copies may live at different filesystem paths; the desktop resolves its
own meshes. Use only one state publisher. For a local launch without RViz,
`rviz:=false` starts only the SDK publisher and model/TF publisher.

This is a display, not a motion controller. Like the other custom monitors,
SDK initialization automatically answers Enter but can still acquire control
when no owner exists. It sends no joint or home-motion commands. Timestamps are
local polling times because these SDK getters expose no sensor timestamps;
separate reads are not synchronized and cached SDK data cannot be identified as
fresh. Invalid samples are skipped, and pose markers expire after publication
stops; the RobotModel may retain its last joint state. Model-to-SDK calibration
alignment and live RViz rendering still need verification on the robot.

### Rear KISS-ICP odometry and the robot base

The combined RViz file now includes the rear local map, base odometry history,
and an optional original-LiDAR odometry display. It follows the setup in
`~/astribot/kiss_iiiicp_utils`: input `/livox/lidar_back`, KISS output
`/kiss/rear/odometry`, map `/kiss/rear/local_map`, parent frame `kiss_rear_odom`,
and cloud frame `livox_frame`. Keep KISS's **`base_frame: ""` and
`publish_odom_tf: false`**; this adapter handles the conversion separately.
KISS-ICP must already be running. This launch does not start or reconfigure it.

`rear_lidar_odometry.py` publishes `/kiss/base/odometry` (`nav_msgs/msg/Odometry`)
with child frame `astribot_viz_chassis`, the same SDK chassis origin used by the
robot model. It broadcasts the following tree **only on `/astribot_viz/tf` and
`/astribot_viz/tf_static`**, leaving the driver's navigation TF untouched:

```text
kiss_rear_odom
  └─ astribot_viz_chassis
       ├─ livox_frame                 (calibrated static mount)
       └─ astribot_torso_base         (existing robot model)
            └─ torso / arms / head
```

**Calibration is still missing.** `config/rear_lidar_to_base.yaml` intentionally has
`base_from_cloud: null`. Fill it with `T_base_cloud`, the rigid transform that
maps the actual input cloud's coordinates to SDK chassis coordinates, in meters:
`p_base = R_base_cloud * p_cloud + t_base_cloud`. Its rotation must be orthonormal
with determinant +1. A missing matrix, reflection or malformed matrix stops
the odometry launch before SDK initialization. No identity transform is assumed.

The local SDK's 15 URDF/SDF files contain no LiDAR links or sensor tags, and its
YAML files contain no LiDAR mount. A deeper inspection of the official
`Astribot-Dev/astribot_descriptions` repository found explicit **nominal S1 mounts
in MJCF**, although its whole-body URDF/SDF omit these LiDAR frames. This corrects
the earlier, incomplete public-source search.

The following values are pinned to commit `32b67e270fecd49bd5444ca49824dcc171dfb3e6`:

| Sensor | Position relative to MJCF `chassis_base` (meters) | Roll/pitch/yaw (degrees, approximately) | Source |
| --- | --- | --- | --- |
| Front | `[0.17393, 0.16893, 0.073397]` | `[0, 0, -45]` | [lidar_mid360_front.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/lidar_mid360_front.xml) |
| Rear | `[-0.1795908, -0.1745908, 0.157397]` | `[0, 0, 135]` | [lidar_mid360_back.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/lidar_mid360_back.xml) |

[s1.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/s1.xml)
includes both sensor bodies directly inside `chassis_base`. Their MJCF
quaternions are `[w,x,y,z]`: front `[0.9238796,0,0,-0.3826834]`, rear
`[0.3826834,0,0,0.9238795]`. ROS uses `[x,y,z,w]`.
The `chassis_base` position `[0,0,0.095]` in that entry point is the simulation's
initial world placement, not another LiDAR mounting offset.

These model mounts are useful references, but do not establish that the MJCF
`chassis_base` equals the installed SDK `chassis` origin, or that live rear-cloud
points are expressed in the rear sensor frame. The repository README also warns
that coordinate conventions can differ between model formats. These values have
therefore not been inserted into the active odometry calibration.

Older local evidence exists in
`~/astribot/Astribot_S1/workspace/src/astribot_navigation/towergo_real/config/calibration.vendor.yaml`:

```text
Unverified FRONT-cloud candidate only; not an active configuration:
  [ 0.696665,  0.717397, 0, 0.17393 ]
  [-0.717397,  0.696665, 0, 0.16893 ]
  [ 0,         0,        1, 0.054   ]
  [ 0,         0,        0, 1       ]
```

That file is explicitly `verified: false`: its rotation is unconfirmed, and
its notes report a determinant -1 for the vendor's rear-sensor matrix. Historical
driver checks also found rear clouds already expressed in the front-LiDAR frame.
Neither the `/livox/lidar_back` topic name nor `livox_frame` proves the physical
sensor coordinate frame. Do not apply a rear mount twice. The remaining source
to inspect on the robot is `/etc/config/sensor_calib.json` together with the
active Livox driver's `MID360_config.json` and point transformation settings.

After supplying the correct matrix, run from the repository root:

```bash
# Same computer: SDK state + base conversion + model + RViz.
ros2 launch ./astribot_scripts/launch/astribot_rviz.launch.py odometry:=true

# Separate desktop: rviz_state.py already runs on the robot-connected PC.
ros2 launch ./astribot_scripts/launch/astribot_rviz.launch.py \
  odometry:=true publish_state:=false
```

The launcher selects fixed frame `kiss_rear_odom` with `odometry:=true`; the
default `odometry:=false` retains the previous chassis-relative model view.
To inspect the local map before calibration, open the RViz file directly and
enable **Original LiDAR odometry (reference)**; disable RobotModel/current/
desired/base-odometry displays until their transforms are available:

```bash
rviz2 -d ./astribot_scripts/config/astribot_poses.rviz --ros-args \
  -r /tf:=/astribot_viz/tf -r /tf_static:=/astribot_viz/tf_static
```

Conversion uses `T_odom_base = T_odom_cloud * inverse(T_base_cloud)`. It retains
the KISS map frame and input timestamp, including the initial base offset; it
does not reset the base to zero or rotate the existing map. Pose covariance
includes the lever arm's first-order position/orientation coupling, treating
calibration as exact; upstream KISS covariance is nominal, not measured accuracy.
KISS does not estimate twist, so output velocities remain zero with large
covariance (1e12 diagonal); **do not fuse them as measured velocity**.

The upstream scan-header/deskew-to-scan-end timing distinction remains unchanged
(about 100 ms in the earlier recording). SDK pose markers follow the latest
chassis TF for visualization; joints and Cartesian states are not synchronized
to the LiDAR exposure. This is not a sensor-fusion or navigation validation.
Replay uses `use_sim_time:=true publish_state:=false` and a bag clock; animated
joints/pose markers also require recorded `/astribot_viz/` state topics. A
LiDAR-only bag supplies base motion, not torso/arm/head joint observations.

For camera and lidar together, run **`get_lidar_and_img.py` instead of separate 300/301 processes**:

```bash
python3 astribot_scripts/get_lidar_and_img.py --cameras head_rgbd left_wrist_rgbd right_wrist_rgbd
```

`get_lidar_and_img.py` uses one SDK connection, automatically activates both camera and lidar drivers,
and receives their existing topics. It does not reset lidar before activation.
The camera driver publishes
`/astribot_camera/<camera>/color_compress/compressed` (`sensor_msgs/msg/CompressedImage`);
the lidar driver publishes `/livox/lidar_front` and `/livox/lidar_back`
(`sensor_msgs/msg/PointCloud2`). Without `--cameras`, `get_lidar_and_img.py` receives `head_rgbd` plus both
lidars. These input types are already standard ROS 2 messages, despite the
`astribot` names in the camera topic paths. `get_lidar_and_img.py` additionally publishes decoded
images and relays complete lidar messages on these separate output topics:

| Output topic | ROS 2 message type | Content |
| --- | --- | --- |
| `/sdk/astribot_camera/<camera>/color_image` | `sensor_msgs/msg/Image` | Full-resolution, uncompressed `bgr8` color pixels |
| `/sdk/astribot_camera/head_rgbd/depth_image` | `sensor_msgs/msg/Image` | Raw `16UC1` head depth, 640×576 by default |
| `/sdk/astribot_lidar/front_scan` | `sensor_msgs/msg/PointCloud2` | Complete front point cloud |
| `/sdk/astribot_lidar/back_scan` | `sensor_msgs/msg/PointCloud2` | Complete back point cloud |

All outputs preserve source timestamps and frame IDs. Point clouds retain all
original fields and data, including Livox-specific fields inside the standard
`PointCloud2` structure. Images retain source dimensions unless `--image-size`
is supplied; `--image-rate` limits publication and decoding to the newest frame
per camera at each tick. Without resizing, `head_stereo` remains the original
combined image. The script publishes no Astribot
custom message types. It uses Best Effort, volatile QoS with queue depth 10 for
these outputs; downstream subscribers should use sensor-data/Best Effort QoS.
Avoid another publisher on the same output topics.

`get_lidar_and_img.py` always runs without a GUI, including when no flags are supplied. It never
opens windows or calls OpenCV GUI functions; no Qt display setup is needed.
`--headless` is accepted for compatibility but is no longer required. The SDK
still uses OpenCV to decode JPEG pixels for standard Image publishing.
Uncompressed images require more bandwidth than the compressed input. `get_lidar_and_img.py` prints
image/scan/point counts once per second. Stop with Ctrl-C. Both drivers remain
running after exit, including if activation of one driver fails after the other
succeeded.

### Head RGB/depth in RViz2

With `head_rgbd` selected (the default), the combined script also subscribes to
`/astribot_camera/head_rgbd/depth_compress` as standard `CompressedImage`.
The remote sample has `format: raw`, 737,280 payload bytes and an empty frame ID.
The configured layout is 640×576 uint16, confirmed in this session; each frame's
byte count is checked before publishing. No `astribot_camera` message package is
needed. Depth is subscribed directly so it cannot replace the SDK color callback.

Depth byte order defaults to **little-endian**; use `--depth-big-endian` if the
source uses big-endian. Use `--head-depth-size WIDTH HEIGHT` if the driver resolution
changes. Pixel values are preserved without scaling; their physical unit has not
been independently verified. Do not assume calibrated meters from this image.
The original timestamp and frame ID (including an empty one) are preserved.

Run the script on the robot-connected computer and open RViz2 on a computer with
a display and matching ROS network settings. Add two **Image** displays:

| Display | Topic | Settings |
| --- | --- | --- |
| Head color | `/sdk/astribot_camera/head_rgbd/color_image` | Transport: raw; Reliability: Best Effort |
| Head depth | `/sdk/astribot_camera/head_rgbd/depth_image` | Transport: raw; Reliability: Best Effort; Normalize Range: enabled |

The output is numeric depth, not a JPEG or a colorized 8-bit depth picture.
RViz2's Image renderer supports `16UC1` and normalizes it for display
([Humble renderer source](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/image/ros_image_texture.cpp)).
Use Image displays for these 2D views. A calibrated Camera display or a depth-derived
3D cloud would additionally require real CameraInfo/TF and verified depth units;
this script does not fabricate calibration or align depth to color. RViz rendering
and live depth receipt have not been verified on the robot.

Joint and EE monitors refresh at **1 Hz by default**. `--rate` is the requested
display/polling rate, not a guaranteed sensor rate or real-time deadline. Stop with
Ctrl-C; camera windows and the lidar terminal also accept `q`.

Camera and lidar activation is **off by default in 300/301**. If the
drivers are not publishing, `--activate` requests activation. For lidar it resets
the driver first and deactivates it on exit. Camera activation uses the SDK's
camera service, and cameras remain running on exit. `--no-activate` is retained
as an explicit alias for lidar's default. No monitor writes images or rosbag files.

## Data and extension points

| Script | Data / place to add processing | Behavior |
| --- | --- | --- |
| `300-get_images.py` | `image_callback(topic_name, msg, width, height, array)` | Continuous color subscriptions; GUI keeps only the newest image per camera |
| `301-get_lidar_scan.py` | `LidarMonitor.lidar_callback(name, msg)` | Continuous front/back `PointCloud2`; all original fields are available in `msg` |
| `get_lidar_and_img.py` | `image_callback(...)` and `lidar_callback(name, msg)` | Automatic activation, raw Image publishing and complete PointCloud2 relay using one SDK connection |
| `101-get_joint_states.py` | `report(astribot)` and `get_current_joints_*()` | Continuous polling; desired values are commands, not measurements |
| `102-get_cartesian_states.py` | `report(astribot, frame)` | Continuous polling of both arms' SDK EE poses |
| `get_joints_and_poses.py` | `report(astribot, frame)` | Combined current/desired joint and Cartesian tables, one SDK connection |
| `publish_robot_data.py` | `read_state(robot, lifetime)` and shared `SensorPublisher` | Combined standard sensor/state topics, three IMUs and recordable full SDK snapshots |
| `100-get_robot_properties.py` | Names, DOFs, position/velocity/torque limits | One snapshot; corrected the original's reversed lower/upper limit assignment |
| `204-forward_kinematics.py` | `get_forward_kinematics(names, joints_position)` | One calculation using hard-coded example joint values, not live measurements |
| `205-inverse_kinematics.py` | `get_inverse_kinematics(names, cartesian_pose)` | One calculation using hard-coded target poses; no motion command |
| `209-get_closest_dist.py` | `get_self_closest_point()` | One current self-collision geometry query; not environmental obstacle sensing |

The camera topics are `/astribot_camera/<camera>/color_compress/compressed`.
Available selections are `head_rgbd`, `torso_rgbd`, `left_wrist_rgbd`,
`right_wrist_rgbd`, and `head_stereo`. Display arrays are BGR. In 300, headless mode
receives compressed bytes; `get_lidar_and_img.py` always decodes for standard Image publishing.
The combined script additionally wraps the configured head raw depth as `16UC1`;
other cameras remain color-only. Stereo splitting, depth/color registration and
camera calibration are not supplied here.

Lidar topics are `/livox/lidar_front` and `/livox/lidar_back`, with Best Effort QoS.
Use `msg.header.stamp` and `msg.header.frame_id` inside image/lidar callbacks for
source timing and coordinates. Counts measure received messages, not completeness
or network loss. Images are decoded on the SDK executor; heavy callback work can
delay other callbacks. Keep later processing bounded and outside the callback.

EE pose order is `[x, y, z, qx, qy, qz, qw]`, with translation in meters. Use
`astribot.arm_names` for left/right EE poses: `effector_names` identify grippers
and return opening values, not 7D poses. `--frame world` selects the SDK world
frame; its physical origin and tool offsets must be checked on the robot.
Revolute joint values use rad/rad/s and torques Nm; chassis and gripper entries
have different semantics, so preserve part names rather than treating all 25
S1 entries as angular joints.

The state getters return values without source timestamps. Separate getter calls
and separate processes are not a synchronized sensor snapshot; they also do not
prove freshness when the robot stops publishing. A future recorder/fusion pipeline
needs explicit source timestamps, freshness checks and alignment. Do not assign
polling time as though it were measurement time. Reuse one `Astribot` instance in
an integrated app; the SDK already owns an executor thread, so do not spin its
node in a second executor.

## Review of every example

The source review covered all **26 Python scripts** in `examples/`:

| Example | Decision and reason |
| --- | --- |
| `100-get_robot_properties.py` | Included: robot structure and limits |
| `101-get_joint_states.py` | Adapted: snapshot → continuous joint monitor |
| `102-get_cartesian_states.py` | Adapted: snapshot → continuous bilateral EE monitor, consistent frame |
| `103-move_to_joint_position.py` | Excluded: homes and moves joints |
| `104-set_joint_position.py` | Excluded: streams position commands |
| `105-set_joint_position_direct.py` | Excluded: streams direct position commands |
| `106-set_joint_velocity.py` | Excluded: streams gripper velocity commands |
| `107-move_to_cartesian_pose.py` | Excluded: homes and commands Cartesian motion |
| `108-set_cartesian_pose.py` | Excluded: streams Cartesian commands |
| `109-effector_open_close.py` | Excluded: sets gripper force and opens/closes grippers |
| `110-set_different_type_command.py` | Excluded: mixes joint and Cartesian commands |
| `201-head_follow.py` | Excluded: changes head tracking mode and moves arms |
| `202-chassis_joy_control_local.py` | Excluded: joystick drives chassis |
| `203-chassis_joy_control_global.py` | Excluded: joystick drives chassis in a global reference |
| `204-forward_kinematics.py` | Included: FK reference |
| `205-inverse_kinematics.py` | Included: IK reference |
| `206-move_joints_waypoints.py` | Excluded: executes joint waypoints |
| `207-move_cartesian_waypoints.py` | Excluded: executes Cartesian waypoints |
| `208-traj_replay.py` | Excluded: repeatedly executes stored trajectories |
| `209-get_closest_dist.py` | Included: closest-link geometry reference |
| `210-arm_gravity_compensation.py` | Excluded: sends torque commands and changes arm behavior |
| `211-whole_body_control_test.py` | Excluded: continuously commands whole-body control |
| `300-get_images.py` | Adapted: camera selection, optional activation, headless mode, main-thread GUI, cleanup |
| `301-get_lidar_scan.py` | Adapted: default driver activation disabled; no saving |
| `302-get_and_play_audio.py` | Excluded: activates audio and plays buffered microphone data through the speaker; also spins a node already owned by the SDK executor |
| `999-stop_robot.py` | Excluded: moves home, stops, restarts and moves again; not a stop-only utility |

`example_data/*.hdf5` are trajectory fixtures for the excluded replay demo and
were not copied. The camera rewrite removes the original undefined
`robot_shutdown()` call and its activation polling loop that had no sleep.

## Offline verification

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s astribot_scripts/tests -p 'test_*.py'
```

The former discovery command with `-s astribot_scripts` also works.

Monitor checks substitute ROS, SDK and GUI dependencies before importing scripts.
`test_sensor_messages.py` additionally uses installed NumPy and real ROS 2 messages
and serialization to validate pixel packing and header preservation offline.
`test_rviz_state.py` checks model mesh paths, joint mapping, pose axes/labels,
invalid data handling, ROS serialization and the mocked SDK polling loop.
`test_rear_lidar_odometry.py` checks mount inversion, rotated lever arms,
covariance propagation, frame validation and refusal of invalid calibration.
`test_robot_data.py` checks full joint/pose snapshots, IMU header/covariance
preservation, invalid samples, single-connection cleanup, observation-only control
acquisition bypass and recorder topic selection. `test_tmux_launcher.py` exercises
the real tmux server with inert command doubles: startup, inherited environment,
paths with spaces, rear/observer arguments, duplicate prevention and graceful
shutdown. It skips when tmux is unavailable and never connects to the robot.
The rear FAST-LIO and robot-model launchers also passed a localhost-only ROS
smoke check on domain 227, with no SDK connection: rear topic/parameter selection
and robot-description receipt were verified, then both launchers exited cleanly.
The base conversion and private TF tree were also checked with synthetic ROS
odometry and joints on an isolated local domain, without the SDK or robot.
An isolated local ROS smoke check also verified 20 moving-link transforms, the
fixed chassis mount and robot-description delivery using synthetic joints.
UDP networking was restricted in that check; it does not validate cross-PC DDS.
No tests connect to or command a robot. Hardware receipt, DDS compatibility,
publication rates, image rendering, pose calibration and simultaneous operation
still require live validation.
