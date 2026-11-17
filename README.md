# Astribot live observation examples

This folder contains selected/adapted scripts from `examples/`, combined
image/lidar and joint/pose monitors, and RViz2 visualization files for development
on the receiving computer. These observation
publishers do not save sensor data or send joint/motion commands. The optional
`record_robot_data.sh` saves a ROS bag. The separate
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

**Camera (300), lidar (301), `get_lidar_and_img.py`, `get_joints_and_poses.py`, and `publish_robot_data.py` automatically answer the SDK prompt with Enter.**
Only during `Astribot()` construction, these custom scripts temporarily replace
`input()` with an empty response. Normal input is restored afterwards, including
when initialization fails. No SDK source changes are required. The SDK may still
print its prompt, but it does not wait for a keypress. Once initialization finishes,
the scripts subscribe to the sensor messages published by the robot's drivers.

The other scripts still use the SDK's default control-rights handling. When another
controller is running, press **Enter**, not `yes`, at their control-rights prompt
to continue without taking control. In **all** scripts, including 300, 301 and `get_lidar_and_img.py`,
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
without resetting them and leaves them running on exit. The SDK control-rights
behavior described above still applies. All five color cameras are selected by
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

For live robot-model visualization alongside the combined publisher:

```bash
ros2 launch ./astribot_scripts/astribot_rviz.launch.py publish_state:=false
```

For offline model/FK replay, stop live publishers and use an isolated ROS domain
in both replay terminals:

```bash
export ROS_DOMAIN_ID=125
ros2 launch ./astribot_scripts/astribot_rviz.launch.py publish_state:=false use_sim_time:=true
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

## Robot model and all Cartesian poses in RViz2

For nominal camera offsets relative to head, torso and wrist joint/link frames,
see [camera_mounts_reference.md](camera_mounts_reference.md). It records official
S1 MJCF sources, composed wrist offsets and the camera-to-ROS optical-axis
conversion; these mounts are not automatically published as calibrated TF.
For the chassis and both LiDAR IMUs, see [imu_mounts_reference.md](imu_mounts_reference.md),
including poses composed into the model's chassis frame and ROS-order quaternions.

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

**Calibration is still missing.** `rear_lidar_to_base.yaml` intentionally has
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
ros2 launch ./astribot_scripts/astribot_rviz.launch.py odometry:=true

# Separate desktop: rviz_state.py already runs on the robot-connected PC.
ros2 launch ./astribot_scripts/astribot_rviz.launch.py \
  odometry:=true publish_state:=false
```

The launcher selects fixed frame `kiss_rear_odom` with `odometry:=true`; the
default `odometry:=false` retains the previous chassis-relative model view.
To inspect the local map before calibration, open the RViz file directly and
enable **Original LiDAR odometry (reference)**; disable RobotModel/current/
desired/base-odometry displays until their transforms are available:

```bash
rviz2 -d ./astribot_scripts/astribot_poses.rviz --ros-args \
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
`PointCloud2` structure. Images are not resized or geometrically transformed;
`head_stereo` remains the original combined image. The script publishes no Astribot
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
python3 -m unittest discover -s astribot_scripts -p 'test_*.py'
```

Monitor checks substitute ROS, SDK and GUI dependencies before importing scripts.
`test_sensor_messages.py` additionally uses installed NumPy and real ROS 2 messages
and serialization to validate pixel packing and header preservation offline.
`test_rviz_state.py` checks model mesh paths, joint mapping, pose axes/labels,
invalid data handling, ROS serialization and the mocked SDK polling loop.
`test_rear_lidar_odometry.py` checks mount inversion, rotated lever arms,
covariance propagation, frame validation and refusal of invalid calibration.
`test_robot_data.py` checks full joint/pose snapshots, IMU header/covariance
preservation, invalid samples, single-connection cleanup and recorder topic selection.
The base conversion and private TF tree were also checked with synthetic ROS
odometry and joints on an isolated local domain, without the SDK or robot.
An isolated local ROS smoke check also verified 20 moving-link transforms, the
fixed chassis mount and robot-description delivery using synthetic joints.
UDP networking was restricted in that check; it does not validate cross-PC DDS.
No tests connect to or command a robot. Hardware receipt, DDS compatibility,
publication rates, image rendering, pose calibration and simultaneous operation
still require live validation.
