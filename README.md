# Astribot live observation examples

This folder contains eight selected/adapted scripts from `examples/` plus a combined
image/lidar script for development on the receiving computer. Five monitors run
continuously; the other four scripts are supporting references. None saves sensor
data or sends joint/motion commands.
The originals in `examples/` were left unchanged during this organization pass.

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

**Camera (300), lidar (301), and combined streaming (303) automatically answer the SDK prompt with Enter.**
Only during `Astribot()` construction, these custom scripts temporarily replace
`input()` with an empty response. Normal input is restored afterwards, including
when initialization fails. No SDK source changes are required. The SDK may still
print its prompt, but it does not wait for a keypress. Once initialization finishes,
the scripts subscribe to the sensor messages published by the robot's drivers.

The other scripts still use the SDK's default control-rights handling. When another
controller is running, press **Enter**, not `yes`, at their control-rights prompt
to continue without taking control. In **all** scripts, including 300, 301 and 303,
the unchanged SDK can still claim control automatically if no owner is found.
The automatic Enter response declines takeover when another owner is present;
it does not disable acquisition altogether. SDK background services still run. See
`astribot_sdk/core/astribot_api/astribot_interface.py::acquire_control_rights`.

Run the following in separate terminals as needed:

```bash
# Live color images: default is head_rgbd; select additional cameras explicitly.
python3 custom/300-get_images.py --cameras head_rgbd left_wrist_rgbd right_wrist_rgbd
# For a terminal without a display, add --headless (counts, no JPEG decoding).

# Both complete PointCloud2 streams; prints scan/point counts and receive rate.
python3 custom/301-get_lidar_scan.py
# Optional bounded session:
python3 custom/301-get_lidar_scan.py --duration 60

# All parts: current and desired joint positions, velocities and torques.
python3 custom/101-get_joint_states.py --rate 5

# Both arms: current and desired EE poses, with the same reference frame.
python3 custom/102-get_cartesian_states.py --rate 5 --frame chassis
```

For camera and lidar together, run **303 instead of separate 300/301 processes**:

```bash
python3 custom/303-get_images_and_lidar.py --cameras head_rgbd left_wrist_rgbd right_wrist_rgbd
# Without a display:
python3 custom/303-get_images_and_lidar.py --cameras head_rgbd left_wrist_rgbd right_wrist_rgbd --headless
```

303 uses one SDK connection, automatically activates both camera and lidar drivers,
and receives their existing topics. It does not reset lidar before activation.
The camera driver publishes
`/astribot_camera/<camera>/color_compress/compressed` (`sensor_msgs/msg/CompressedImage`);
the lidar driver publishes `/livox/lidar_front` and `/livox/lidar_back`
(`sensor_msgs/msg/PointCloud2`). Without `--cameras`, 303 receives `head_rgbd` plus both
lidars. These input types are already standard ROS 2 messages, despite the
`astribot` names in the camera topic paths. 303 additionally publishes decoded
images and relays complete lidar messages on these separate output topics:

| Output topic | ROS 2 message type | Content |
| --- | --- | --- |
| `/sdk/astribot_camera/<camera>/color_image` | `sensor_msgs/msg/Image` | Full-resolution, uncompressed `bgr8` color pixels |
| `/sdk/astribot_lidar/front_scan` | `sensor_msgs/msg/PointCloud2` | Complete front point cloud |
| `/sdk/astribot_lidar/back_scan` | `sensor_msgs/msg/PointCloud2` | Complete back point cloud |

All outputs preserve source timestamps and frame IDs. Point clouds retain all
original fields and data, including Livox-specific fields inside the standard
`PointCloud2` structure. Images are not resized or geometrically transformed;
`head_stereo` remains the original combined image. The script publishes no Astribot
custom message types. It uses Best Effort, volatile QoS with queue depth 10 for
these outputs; downstream subscribers should use sensor-data/Best Effort QoS.
Avoid another publisher on the same output topics.

`--headless` disables only the GUI in 303: images are still decoded and published.
Uncompressed images require more bandwidth than the compressed input. 303 displays
selected color images and prints image/scan/point counts once
per second. Stop with Ctrl-C or `q` in an image window. Both drivers remain running
after exit, including if activation of one driver fails after the other succeeded.

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
| `303-get_images_and_lidar.py` | `image_callback(...)` and `lidar_callback(name, msg)` | Automatic activation, raw Image publishing and complete PointCloud2 relay using one SDK connection |
| `101-get_joint_states.py` | `report(astribot)` and `get_current_joints_*()` | Continuous polling; desired values are commands, not measurements |
| `102-get_cartesian_states.py` | `report(astribot, frame)` | Continuous polling of both arms' SDK EE poses |
| `100-get_robot_properties.py` | Names, DOFs, position/velocity/torque limits | One snapshot; corrected the original's reversed lower/upper limit assignment |
| `204-forward_kinematics.py` | `get_forward_kinematics(names, joints_position)` | One calculation using hard-coded example joint values, not live measurements |
| `205-inverse_kinematics.py` | `get_inverse_kinematics(names, cartesian_pose)` | One calculation using hard-coded target poses; no motion command |
| `209-get_closest_dist.py` | `get_self_closest_point()` | One current self-collision geometry query; not environmental obstacle sensing |

The camera topics are `/astribot_camera/<camera>/color_compress/compressed`.
Available selections are `head_rgbd`, `torso_rgbd`, `left_wrist_rgbd`,
`right_wrist_rgbd`, and `head_stereo`. Display arrays are BGR. In 300, headless mode
receives compressed bytes; 303 always decodes for standard Image publishing.
The custom monitors cover color only: the original does not
implement depth decoding either, and the SDK returns raw bytes for non-JPEG data.
Stereo splitting, depth decoding and camera calibration are not supplied here.

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
python3 -m unittest discover -s custom -p 'test_*.py'
```

Monitor checks substitute ROS, SDK and GUI dependencies before importing scripts.
`test_sensor_messages.py` additionally uses installed NumPy and real ROS 2 messages
and serialization to validate pixel packing and header preservation offline.
No tests connect to or command a robot. Hardware receipt, DDS compatibility,
publication rates, image rendering, pose calibration and simultaneous operation
still require live validation.
