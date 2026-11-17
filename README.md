# Astribot live observation examples

This folder contains selected/adapted scripts from `examples/`, combined
image/lidar and joint/pose monitors, and RViz2 visualization files for development
on the receiving computer. These observation
scripts do not save sensor data or send joint/motion commands. The separate
`startup.py` described below does command home motion.
The originals in `examples/` were left unchanged during this organization pass.

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

**Camera (300), lidar (301), combined streaming (`get_liadra_and_img.py`), and `get_joints_and_poses.py` automatically answer the SDK prompt with Enter.**
Only during `Astribot()` construction, these custom scripts temporarily replace
`input()` with an empty response. Normal input is restored afterwards, including
when initialization fails. No SDK source changes are required. The SDK may still
print its prompt, but it does not wait for a keypress. Once initialization finishes,
the scripts subscribe to the sensor messages published by the robot's drivers.

The other scripts still use the SDK's default control-rights handling. When another
controller is running, press **Enter**, not `yes`, at their control-rights prompt
to continue without taking control. In **all** scripts, including 300, 301 and `get_liadra_and_img.py`,
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

## Robot model and all Cartesian poses in RViz2

On the computer with a display, source ROS and `env.sh` as above, then run from
the repository root:

```bash
ros2 launch ./astribot_scripts/astribot_rviz.launch.py
# Optional refresh rate (default 10 Hz):
ros2 launch ./astribot_scripts/astribot_rviz.launch.py rate:=20
```

No colcon build is needed. This starts `rviz_state.py`, `robot_state_publisher`
and RViz2 with `astribot_poses.rviz`. It requires the ROS Humble packages
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
ros2 launch ./astribot_scripts/astribot_rviz.launch.py publish_state:=false
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

For camera and lidar together, run **`get_liadra_and_img.py` instead of separate 300/301 processes**:

```bash
python3 astribot_scripts/get_liadra_and_img.py --cameras head_rgbd left_wrist_rgbd right_wrist_rgbd
```

`get_liadra_and_img.py` uses one SDK connection, automatically activates both camera and lidar drivers,
and receives their existing topics. It does not reset lidar before activation.
The camera driver publishes
`/astribot_camera/<camera>/color_compress/compressed` (`sensor_msgs/msg/CompressedImage`);
the lidar driver publishes `/livox/lidar_front` and `/livox/lidar_back`
(`sensor_msgs/msg/PointCloud2`). Without `--cameras`, `get_liadra_and_img.py` receives `head_rgbd` plus both
lidars. These input types are already standard ROS 2 messages, despite the
`astribot` names in the camera topic paths. `get_liadra_and_img.py` additionally publishes decoded
images and relays complete lidar messages on these separate output topics:

| Output topic | ROS 2 message type | Content |
| --- | --- | --- |
| `/sdk/astribot_camera/<camera>/color_image` | `sensor_msgs/msg/Image` | Full-resolution, uncompressed `bgr8` color pixels |
| `/sdk/astribot_camera/head_rgbd/depth_image` | `sensor_msgs/msg/Image` | Raw `16UC1` head depth, 640×576 by default |
| `/sdk/astribot_lidar/front_scan` | `sensor_msgs/msg/PointCloud2` | Complete front point cloud |
| `/sdk/astribot_lidar/back_scan` | `sensor_msgs/msg/PointCloud2` | Complete back point cloud |

All outputs preserve source timestamps and frame IDs. Point clouds retain all
original fields and data, including Livox-specific fields inside the standard
`PointCloud2` structure. Images are not resized or geometrically transformed;
`head_stereo` remains the original combined image. The script publishes no Astribot
custom message types. It uses Best Effort, volatile QoS with queue depth 10 for
these outputs; downstream subscribers should use sensor-data/Best Effort QoS.
Avoid another publisher on the same output topics.

`get_liadra_and_img.py` always runs without a GUI, including when no flags are supplied. It never
opens windows or calls OpenCV GUI functions; no Qt display setup is needed.
`--headless` is accepted for compatibility but is no longer required. The SDK
still uses OpenCV to decode JPEG pixels for standard Image publishing.
Uncompressed images require more bandwidth than the compressed input. `get_liadra_and_img.py` prints
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
| `get_liadra_and_img.py` | `image_callback(...)` and `lidar_callback(name, msg)` | Automatic activation, raw Image publishing and complete PointCloud2 relay using one SDK connection |
| `101-get_joint_states.py` | `report(astribot)` and `get_current_joints_*()` | Continuous polling; desired values are commands, not measurements |
| `102-get_cartesian_states.py` | `report(astribot, frame)` | Continuous polling of both arms' SDK EE poses |
| `get_joints_and_poses.py` | `report(astribot, frame)` | Combined current/desired joint and Cartesian tables, one SDK connection |
| `100-get_robot_properties.py` | Names, DOFs, position/velocity/torque limits | One snapshot; corrected the original's reversed lower/upper limit assignment |
| `204-forward_kinematics.py` | `get_forward_kinematics(names, joints_position)` | One calculation using hard-coded example joint values, not live measurements |
| `205-inverse_kinematics.py` | `get_inverse_kinematics(names, cartesian_pose)` | One calculation using hard-coded target poses; no motion command |
| `209-get_closest_dist.py` | `get_self_closest_point()` | One current self-collision geometry query; not environmental obstacle sensing |

The camera topics are `/astribot_camera/<camera>/color_compress/compressed`.
Available selections are `head_rgbd`, `torso_rgbd`, `left_wrist_rgbd`,
`right_wrist_rgbd`, and `head_stereo`. Display arrays are BGR. In 300, headless mode
receives compressed bytes; `get_liadra_and_img.py` always decodes for standard Image publishing.
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
python3 -m unittest discover -s astribot_scripts -p 'test_*.py'
```

Monitor checks substitute ROS, SDK and GUI dependencies before importing scripts.
`test_sensor_messages.py` additionally uses installed NumPy and real ROS 2 messages
and serialization to validate pixel packing and header preservation offline.
`test_rviz_state.py` checks model mesh paths, joint mapping, pose axes/labels,
invalid data handling, ROS serialization and the mocked SDK polling loop.
An isolated local ROS smoke check also verified 20 moving-link transforms, the
fixed chassis mount and robot-description delivery using synthetic joints.
UDP networking was restricted in that check; it does not validate cross-PC DDS.
No tests connect to or command a robot. Hardware receipt, DDS compatibility,
publication rates, image rendering, pose calibration and simultaneous operation
still require live validation.
