# Nav2 on the current Astribot S1 / rear FAST-LIO stack

This integration targets **ROS 2 Humble**, matching the current receiving PC.
Nav2 plans and follows paths and produces chassis velocity commands. Astribot's
SDK and robot drivers remain responsible for motor/joint control. No `ros2_control`
wheel interface, differential-drive conversion, or SDK modification is needed.

The supplied profile provides **mapless local navigation** using a rolling 20 m
global costmap and a 6 m local costmap. It does not load a saved occupancy map or
run AMCL. Goals must stay within the current rolling planning window. Unobserved
space is treated as free, so this is a baseline for short local goals rather than
building-scale routes through unexplored space. FAST-LIO drift is not corrected
by a global localization system.

## Data and command paths

```text
/fast_lio/rear/odometry (map -> rear IMU, source measurement stamps)
  -> nav2_bridge.py -> /astribot_nav/odom (chassis pose + measured finite-difference twist)

/fast_lio/rear/cloud_registered_body (points already expressed in rear IMU)
  -> nav2_bridge.py -> /astribot_nav/points -> Nav2 obstacle costmaps

Nav2 DWB controller -> velocity smoother -> /astribot_nav/cmd_vel
  -> nav2_bridge.py freshness gate -> /astribot/nav_cmd_vel
  -> control_mobile_base.py -> Astribot.set_joints_position(chassis)
```

Nav2 recovery behaviors also publish to `/astribot_nav/cmd_vel` and pass through
the same gate. The controller and smoother support lateral motion, matching the
SDK chassis example. Default limits are 0.20 m/s planar speed and 0.40 rad/s yaw.
The existing SDK receiver retains its command watchdog and filtered control.

The navigation TF tree is published on **`/astribot_nav/tf`** and
**`/astribot_nav/tf_static`**:

```text
astribot_nav_odom -> astribot_nav_base_link -> astribot_nav_rear_imu -> astribot_nav_rear_lidar
```

The first valid chassis pose defines the navigation origin and initial yaw;
the Z axis remains aligned to LIO's gravity-aligned world. Full 3D transforms
are retained for obstacle projection; Nav2 controls X, Y and yaw. This local
profile uses `astribot_nav_odom` for both global and local planning, so no
`map -> odom` localization transform is required. A saved-map setup would need
an additional localization/map integration and different global costmap settings.

The existing `/astribot_viz/tf` display overlay, its `map` origin, sensor topics,
FAST-LIO profiles and desktop startup buttons are unchanged. Navigation does
**not** depend on the visualization's `/fast_lio/chassis/odometry` publisher or
its display-retimed TF.

## Install and start

On the Ubuntu receiving PC, install the Humble runtime if it is absent:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-tf2-ros python3-numpy python3-scipy python3-yaml
```

Start the existing sensor/FAST-LIO tmux stack as usual. Its sensor publisher
must use `--observe-only`. Power/activate the robot through the existing startup
procedure before enabling motion. Do not run another base or joint SDK controller
at the same time as this launch.

From the SDK root, inspect feedback and planning without starting an SDK receiver:

```bash
bash astribot_scripts/start_nav2.sh enable_control:=false
```

The wrapper sources ROS, `env.sh`, and the existing ROS workspace if available.
It does not start a second sensor publisher or FAST-LIO instance. In another
terminal, source ROS and `env.sh`, then check:

```bash
ros2 topic echo /astribot_nav/status
ros2 topic hz /astribot_nav/odom
ros2 topic hz /astribot_nav/points
ros2 lifecycle get /astribot_nav/controller_server
ros2 run tf2_ros tf2_echo astribot_nav_odom astribot_nav_base_link \
  --ros-args -r /tf:=/astribot_nav/tf -r /tf_static:=/astribot_nav/tf_static
```

Expect `feedback ready; preview output disabled`, odometry/clouds near the
FAST-LIO update rate (approximately 10 Hz for the current profile), and an
active controller. The command gate needs at least two odometry samples to
calculate measured velocity. Preview publishes no commands to the SDK topic.
Bag preview uses `enable_control:=false use_sim_time:=true` with a
matching bag clock; hardware output under simulation time is rejected.

Stop preview, then run the control-enabled launch:

```bash
bash astribot_scripts/start_nav2.sh

# Alternatively, also receive body joint commands through this one SDK owner:
bash astribot_scripts/start_nav2.sh with_joints:=true
```

Use one of those commands. Do not separately start `control_mobile_base.py` or
`control_joints.py`. If another SDK controller owns the robot, the launched
receiver exits rather than obtaining a takeover answer from closed stdin; stop
that controller first. Ctrl-C stops the launch and requests SDK stop through
the receiver. If the bridge or receiver process exits, the launch shuts down.

## Resolve the existing clock discrepancy

The existing live-validation notes recorded sensor stamps about **27 seconds
behind** the SDK PC clock; recorded bags showed a different offset. Neither
number is a calibration for the current run. The visualization code hides this
for display by retiming TF; that is not used here.

By default, `stamp_offset=0.0`. Odometry and clouds must have recent receipt
times **and** corrected measurement stamps less than 0.5 s old and no more than
0.05 s in the future. Their stamps must agree within 0.2 s. A new DDS arrival
does not make an old measurement acceptable. The bridge logs the measured stamp
age and blocks commands when this fails.

Synchronize the sensor/robot and receiving-PC clocks. If the sensor has a known
fixed clock offset, supply the independently verified value as **ROS time minus
sensor time**, in seconds:

```bash
# Replace OFFSET with the verified signed offset, not an observed processing delay.
bash astribot_scripts/start_nav2.sh enable_control:=false stamp_offset:=OFFSET
```

The same constant is applied to both odometry and clouds, preserving their
relative timing; original topics are untouched. Do not tune this value merely
until old/buffered data passes the gate. Restart with the same verified argument
when enabling control. A source timestamp reset, a feedback gap over one second,
or an implausible pose jump latches an odometry fault; check the cause and restart
the navigation launch. Restart navigation whenever FAST-LIO restarts, because its
world origin and existing goal coordinates are no longer valid.

## Send a local navigation goal

### Click a target in the existing combined RViz

The combined view's `config/astribot_fast_lio.rviz` now includes the standard
RViz **2D Goal Pose** and **Publish Point** tools, plus a **Click to navigate**
display group. No custom RViz plugin compilation is needed.

1. Start the existing observation/FAST-LIO view and run `bash astribot_scripts/start_nav2.sh`
   in another terminal. Use the verified `stamp_offset` argument if required.
2. If RViz was already open, choose **File → Open Config** and load
   `astribot_scripts/config/astribot_fast_lio.rviz` from this SDK checkout.
   Reload only the RViz configuration; restarting FAST-LIO resets the map origin.
3. Keep **Fixed Frame = map**. Select **2D Goal Pose** (shortcut **G**), click
   the destination on the ground plane, drag to set the final heading, then
   release. This immediately sends one goal to Nav2.
4. For X/Y only, select **Publish Point** (shortcut **U**) and click a visible
   point-cloud surface. The bridge uses its X/Y, discards the picked height,
   and preserves the robot's current heading. This tool needs a rendered object
   under the cursor; use 2D Goal Pose when clicking empty ground.

RViz displays an arrow and coordinate/status label at the target, and a cyan
planned path. The arrow turns green when Nav2 reports **Target reached** and red
on failure/cancellation. A new target replaces the active target once Nav2 accepts
it; these tools do not create a waypoint queue. Clicks made while a goal request
is awaiting a response are rejected; click again after its response. A rejected
replacement leaves the previous goal active.

Clicks use **FAST-LIO map coordinates**, not the robot's local X/Y. The goal
bridge applies the navigation origin's translation and rotation before sending
`/astribot_nav/navigate_to_pose`. It converts the planned path back into `map`
for display, so the existing RViz TF tree needs no additional parent or transform.
The target Z is projected onto the navigation plane. Motion still requires fresh
odometry/clouds, an active Nav2 server, and the SDK controller. Preview mode lets
you inspect paths but labels the target **Preview (motion disabled)** and sends
no physical velocity commands.

The tools use `/astribot_nav/goal_pose_map` (`PoseStamped`) and
`/astribot_nav/goal_point_map` (`PointStamped`). Read status or cancel a goal sent
by these tools from a sourced terminal:

```bash
ros2 topic echo /astribot_nav/goal_status
ros2 service call /astribot_nav/cancel_goal std_srvs/srv/Trigger '{}'
```

Cancellation is an asynchronous Nav2 request; the label changes to **Navigation
canceled** when Nav2 confirms it. Ctrl-C in the Nav2 launch terminal stops the
whole navigation launch and its SDK receiver. Goals outside the rolling planning
window may fail planning; start with nearby reachable targets.

### Send a goal from the terminal

With fresh feedback and the robot ready, this example requests a point 0.5 m
forward of the navigation **startup origin**, with its initial heading. It is not
a relative move from the robot's current pose:

```bash
ros2 action send_goal /astribot_nav/navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: astribot_nav_odom}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

For visualization, open a separate RViz using the navigation TF topics:

```bash
rviz2 -f astribot_nav_odom --ros-args \
  -r /tf:=/astribot_nav/tf -r /tf_static:=/astribot_nav/tf_static
```

Add Map displays for `/astribot_nav/global_costmap/costmap` and
`/astribot_nav/local_costmap/costmap`, a Path display for `/astribot_nav/plan`,
PointCloud2 for `/astribot_nav/points`, and a Polygon display for
`/astribot_nav/local_costmap/published_footprint`. The original combined RViz can
continue running independently.

## Geometry and tuning

`config/nav2_humble.yaml` is merged with the installed Humble Nav2 defaults at
launch, keeping that installation's BT plugin list and standard lifecycle setup.
The generated YAML is removed on launch shutdown. Use `params_file:=PATH` for
custom overrides.

The default sensor mounts are the current nominal model transforms used by the
rear-LIO setup. They are **not independently measured per-robot calibration**.
The raw rear cloud's existing driver-frame assumption remains a prerequisite.
For measured mounts, provide `mount_file:=PATH` pointing to JSON containing:

- `base_from_imu`: proper 4x4 matrix mapping rear IMU coordinates into the SDK chassis.
- `imu_from_lidar`: proper 4x4 matrix mapping physical rear lidar coordinates into its IMU.

The corrected body cloud is in IMU coordinates; the separate physical lidar TF
sets the raytracing origin. Do not feed raw `/livox/lidar_back` into this adapter
or substitute the IMU pose for chassis pose. Velocity is derived from successive
chassis measurements, accounting for the sensor offset. Covariances are marked
high/uncalibrated; this output is for Nav2 feedback, not a new precision EKF source.

The 0.45 m circular robot radius is an initial envelope based on the model's
0.30 m base radius plus margin. Verify it against the whole robot in its navigation
posture; extended arms or a payload require a larger/measured footprint. The
obstacle height band is 0.08–1.8 m relative to the initial chassis plane. Inspect
the live costmaps for ground/self returns, low obstacles and sensor blind spots
before treating the profile as tuned. Motion validation and physical stopping
distance measurements have not been performed by these offline tests.

## Offline checks and references

```bash
python3 -m unittest astribot_scripts.tests.test_nav2 \
  astribot_scripts.tests.test_command_receivers -v
bash -n astribot_scripts/start_nav2.sh
```

Tests cover chassis/IMU geometry, frame-relative measured velocity, timestamps,
command gating, preview isolation, malformed messages, launch assembly, parameter
merging, map-goal conversion, goal replacement/cancellation, path display, and the
existing SDK receiver. ROS messages/launch/action APIs are mocked here;
running Nav2 lifecycle nodes and commanding real hardware require the receiving PC.

Implementation references:

- [Nav2 Humble navigation launch](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py)
- [Nav2 Humble default parameters](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
- [Pinned FAST-LIO output implementation](https://github.com/Ericsii/FAST_LIO/blob/2fffc570a25d0df172720bac034fbdb6a13d2162/src/laserMapping.cpp)
- [Current rear FAST-LIO integration and clock observations](fast_lio.md)
