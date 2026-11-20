# FAST-LIO2 on Astribot S1 (ROS 2 Humble)

Prepared on 2026-11-18 for `fourier@192.168.100.102`; updated the same day to make
the rear lidar/IMU pair the default, using the robot-description poses as requested.
This uses the
`Ericsii/FAST_LIO` ROS 2 fork. The original request's “fast-liom” is interpreted
as FAST-LIO2, not FAST-LIVO2 (which also uses cameras).

## Installed layout and scope

- `/home/fourier/ros2_ws/src/FAST_LIO`: patched FAST-LIO source.
- `/home/fourier/ros2_ws/src/livox_ros_driver2`: upstream build dependency. Its
  node is **not** started; Astribot's existing driver remains the sensor source.
- `/home/fourier/ros2_ws/vendor/Livox-SDK2`: SDK source; installed into
  `/home/fourier/ros2_ws/livox_sdk2`, not `/usr/local`.
- `/home/fourier/astribot_sdk_ros2/astribot_scripts/config/fast_lio_rear.yaml`:
  default rear-lidar/rear-IMU profile, including nominal cloud-frame correction.
- `/home/fourier/astribot_sdk_ros2/astribot_scripts/config/fast_lio_front.yaml`:
  earlier front profile, selected with `sensor:=front`.
- `/home/fourier/astribot_sdk_ros2/astribot_scripts/launch/astribot_fast_lio.launch.py`:
  separate launch, without SDK connection, motion commands or driver activation.

KISS-ICP and the existing rear-cloud-to-chassis calibration are unchanged.
This is a **rear-pair experimental baseline with nominal model transforms**. It does
not establish calibration accuracy or enable navigation/control fusion. The
combined RViz now adds a nominal chassis visualization transform, described below.

Pinned sources used for this build:

| Repository | Commit |
| --- | --- |
| [Ericsii/FAST_LIO, ros2](https://github.com/Ericsii/FAST_LIO/tree/2fffc570a25d0df172720bac034fbdb6a13d2162) | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree/tree/e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4) | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |
| [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2/tree/21445540f0d100dc86a7e6df312dd70bbdb4afdf) | `21445540f0d100dc86a7e6df312dd70bbdb4afdf` |
| [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2/tree/c0796f04c143143899c87a773d9f6b7136453c0b) | `c0796f04c143143899c87a773d9f6b7136453c0b` |

## Why changing only YAML is insufficient

The inspected `robot_data_run01` bag has 1,017 clouds and 20,337 IMU messages
per sensor, approximately 10 Hz and 200 Hz respectively. Both front and rear
use `sensor_msgs/msg/PointCloud2`, not `livox_ros_driver2/msg/CustomMsg`.
Cloud fields are:

| Field | Type | Byte offset |
| --- | --- | --- |
| x / y / z | FLOAT32 | 0 / 4 / 8 |
| intensity | FLOAT32 | 12 |
| tag / line | UINT8 | 16 / 17 |
| timestamp | FLOAT64 | 18 |

The point stride is 26 bytes. `timestamp` is an **absolute nanosecond** value
(around `1.7949e18` in this bag), with about 99.83 ms per scan. It must be
converted to milliseconds relative to the cloud header **before** casting to
float. Absolute epoch nanoseconds in FLOAT64 already have approximately
256 ns quantization; the parser tolerates a 1 microsecond rounding discrepancy.

In the pinned fork, `lidar_type: 1` subscribes to Livox CustomMsg. Type 4's
MID-360 handler instead synthesizes times from azimuth and discards the measured
`timestamp` field. Neither is appropriate for this recorded cloud format.

The reproducible [source patch](../integrations/fast_lio/astribot_pointcloud.patch):

- Adds explicit `preprocess.lidar_type: 5` for this Astribot schema. It preserves
  coordinates, converts measured point times, sorts by time and applies the
  configured blind radius, line/tag checks and point stride.
- Handles unaligned fields, either byte order and organized row padding; rejects
  wrong/missing fields, truncated data and timestamps outside the expected scan
  interval. It never substitutes azimuth timing when timestamps are absent.
- Drops invalid scans with a throttled warning and keeps cloud/time queues
  together if cloud timestamps go backwards. Restart FAST-LIO for each replay;
  this is not a full filter reset for looping bags.
- Uses sensor-compatible Best Effort IMU QoS with a 200-message queue; this
  accepts both Reliable and Best Effort publishers.
- Adds `preprocess.cloud_to_lidar_R/T` to restore native lidar coordinates
  before range filtering and deskew. Defaults are identity/zero for the front
  profile. Invalid, nonfinite or reflected transforms are rejected.
- Includes nine native parser tests covering timing, filtering, layout,
  malformed input, transform validation and transformed zero-return removal.
  Existing sensor modes remain available.

**Type 5 requires this patch and a rebuild.** An unpatched fork may silently
fall back to its generic parser, losing point times. The earlier front-only
patch does not implement cloud-frame correction; rebuild with the updated patch
before using the rear profile.

## Extrinsics, units and frame meaning

The [nominal mount reference](imu_mounts_reference.md) gives
`T_lidar_imu`, mapping IMU coordinates into the physical lidar frame:

```text
R_lidar_imu = identity
t_lidar_imu = [0.011, 0.02329, -0.04412] meters
```

FAST-LIO uses the inverse direction: `p_imu = extrinsic_R * p_lidar + extrinsic_T`.
Both profiles use this physical lidar-to-its-own-IMU transform:

```yaml
mapping:
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  extrinsic_est_en: false
```

This is consistent with the fork's nominal MID-360 profile. The rear profile
first restores native rear-lidar coordinates using the model-based correction
described below. It assumes the rear cloud is published in the front-lidar frame
and the rear IMU retains native axes. The front profile assumes native front
cloud and IMU axes. These are nominal model configurations, not per-robot
calibration.

The default pairs `/livox/lidar_back` with `/livox/imu_back`; `sensor:=front`
pairs the corresponding front streams. The recorded IMU acceleration
norm is around 1, consistent with Livox values in g; the pinned FAST-LIO IMU
processor normalizes acceleration by the initial mean norm to its gravity
constant. This integration leaves the messages unchanged. It does not consume
the IMU orientation quaternion. Start with the robot stationary for gravity and
gyro-bias initialization. Noise settings are upstream starting values, not
measured noise calibration.

The two sensors' header stamps overlap. `time_sync_en` stays false and
`time_offset_lidar_to_imu` stays 0.0; bag-arrival versus sensor-clock differences
are not evidence for a lidar-to-IMU offset. Recalibrate time offset if motion
tests show a synchronization problem.

With the world-frame patch and supplied profiles, the fork publishes
`map -> body`, where **body is the selected IMU
origin**, not the lidar origin or SDK chassis. The launcher isolates outputs:

| Output | Topic |
| --- | --- |
| Rear IMU pose odometry | `/fast_lio/rear/odometry` |
| Path | `/fast_lio/rear/path` |
| Registered world cloud | `/fast_lio/rear/cloud_registered` |
| Cloud in rear IMU frame | `/fast_lio/rear/cloud_registered_body` |
| TF / static TF | `/fast_lio/rear/tf`, `/fast_lio/rear/tf_static` |

Selecting `sensor:=front` uses `/fast_lio/front/` instead. Both profiles select
`common.world_frame: map`; the patch applies it consistently to world clouds,
odometry, TF and every path pose. The upstream default remains `camera_init`
for configurations that omit the parameter. Sensor TF is isolated on private topics.
The combined rear visualization creates a separate display-time pose TF from
odometry on `/astribot_viz/tf`; these original sensor TF topics remain unchanged.

The fork does not fill a measured twist in Odometry; do not fuse its default
zero twist as a velocity measurement. Its pose covariance is an estimator
quantity, and the pinned implementation fills it after publishing, so it lags
one output (the first published covariance is zero). No covariance or chassis
fusion changes are included in this input-format patch.

To obtain chassis pose later, use `T_world_chassis = T_world_imu * T_imu_chassis`
with a verified transform to the SDK chassis. The KISS adapter assumes its
input pose is a **cloud** pose and must not be connected to this IMU pose as-is.

## Run on the remote PC

### One RViz on the receiving PC

Load `astribot_scripts/config/astribot.tmux.yaml` with tmuxp. Mapping runs with
`sensor:=rear rviz:=false`; only
`astribot_lio_rviz.launch.py` starts RViz. Its configuration is
`config/astribot_fast_lio.rviz`, with fixed frame `map`:

- Robot chassis odometry: `/fast_lio/chassis/odometry` (up to 1000 historical
  axes, spaced by 0.05 m translation or 0.1 rad rotation; child frame
  `astribot_viz_chassis`, covariance display disabled).
- Chassis trajectory: `/fast_lio/chassis/path`, drawn as a white line containing
  the current mapping session's full published path. The mapper keeps this
  history when only the visualization is reloaded.
- Map view: 60 seconds of `/fast_lio/rear/cloud_registered`, accumulated in the
  fixed world frame. This is a bounded scan map, not the mapper's saved full map;
  upstream unbounded map accumulation/PCD saving stays disabled.
- The articulated model, current/desired Cartesian pose markers, and raw `rgb8`
  images from head, torso, left wrist and right wrist, resized to 640×480 at
  3 Hz per camera from the compressed source streams. The combined view's RGB-D
  adapter separately samples head/torso depth and provides colored cloud displays
  after depth calibration is configured; see [RGB-D point clouds](rgbd_pointcloud.md).
  Stereo outputs remain excluded by the tmux sensor command.
- **Robot joint axes:** chassis base; torso joints 1–3; chest/torso joint 4;
  head pan and tilt; left and right wrist joint 7. These are native Axes displays
  tied to the actual chassis/URDF link frames, grouped for independent toggling.

The tree uses one visualization TF namespace:

```text
map --latest chassis pose at display time--> astribot_viz_chassis
                            ├─ body (rear IMU, fixed nominal mount)
                            └─ astribot_torso_base (SDK +0.097 m weld)
                                └─ torso / arms / head (joint states)
```

The grid and Orbit view target also use `map`. The viewer therefore stays fixed
in the environment instead of following the robot chassis. `map` is the local
world initialized by FAST-LIO at startup; restarting mapping resets its origin.
This frame-name/view change does not add loop closure or eliminate odometry drift.

After rebuilding and restarting through the desktop shortcut, a 12-second live
check received 98 registered clouds and matched IMU/chassis odometry samples.
Both odometry topics, both paths (including each pose), registered clouds and
sensor TF used `map`. All four nominal optical frames resolved to `map`, the
four images were `rgb8`, and RViz reported Global Status OK with Orbit target
`map`. No motion commands were issued during validation. Results:
[`validation_map_frames_20261118.json`](../integrations/fast_lio/validation_map_frames_20261118.json).

`common/fast_lio_viz.py` computes the mount instead of equating the IMU and
chassis origins. The pinned MJCF `chassis_base` corresponds nominally to the
SDK URDF `astribot_torso_base`: their base mesh origins and four torso joint
transforms match (including the first joint at `[0,0,0.12]` and RPY
`[-1.5708,1.5708,0]`). Source:
[`kinematic_visual.xml`](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/kinematic_visual.xml)
and the bundled `astribot_whole_body_with_head.urdf`. Thus:

```text
T_sdk_chassis_imu = T_sdk_chassis_torso_base * T_model_chassis_rear * T_rear_imu
T_imu_sdk_chassis = inverse(T_sdk_chassis_imu)
T_world_sdk_chassis = T_world_imu * T_imu_sdk_chassis
```

The rear IMU position in SDK chassis coordinates is nominally
`[-0.203837491, -0.183281143, 0.210277]` m, yaw approximately 135 degrees.
The extra 0.097 m is the SDK weld, not the MJCF initial world placement.
The chassis conversion applies the lever arm and orientation, rather than simply
renaming the IMU frame. It preserves the map origin and measurement timestamps,
and propagates pose covariance through the offset. Upstream does not provide a
measured twist; chassis twist is left zero with a large (1e12 diagonal) covariance
and must not be interpreted as measured velocity. The chassis path converts every
pose in the upstream path with the same mount, preserving all pose timestamps.
This explicit nominal correspondence is suitable for the requested model overlay;
it does not establish physical calibration for navigation fusion.
The KISS calibration remains untouched. Pose markers follow the latest chassis
TF; SDK polling and LiDAR measurement times are not synchronized.

Live validation found sensor odometry stamps lagging SDK joint polling by about
27 seconds, so directly joining their measurement-time TF trees cannot transform
the articulated links. `fast_lio_display_tf.py` broadcasts the latest chassis pose at
20 Hz using the visualization clock, and stops broadcasting after two seconds
without a new odometry message. Only `/astribot_viz/tf` is retimed; converted
chassis odometry/path, source IMU odometry, registered clouds and
`/fast_lio/rear/tf` retain measurement timestamps.
This is an explicit latest-pose display overlay, not timestamp correction for
mapping or synchronized navigation fusion.

The YAML wraps only the visualization launch with `desktop_exec.py`. On this
receiving PC it selects the logged-in user's Xwayland desktop and validates X
access, so one RViz window opens on that PC when tmuxp is invoked over SSH.
No X11 forwarding or `xhost` access-control changes are used.

### Standalone mapping

The sensor driver must already be running. This launcher does not activate it.
In a fresh Bash terminal on `fourier@192.168.100.102`:

```bash
source /opt/ros/humble/setup.bash
source /home/fourier/ros2_ws/install/local_setup.bash
cd /home/fourier/astribot_sdk_ros2
source env.sh
ros2 launch ./astribot_scripts/launch/astribot_fast_lio.launch.py
# To select the earlier front profile instead:
# ros2 launch ./astribot_scripts/launch/astribot_fast_lio.launch.py sensor:=front
```

Use `rviz:=true` on a graphical ROS machine to open the upstream RViz display,
with the same output and TF remaps. The GUI was not exercised in the headless
replay check. Mapping runs without RViz by default. PCD accumulation and full-map
publishing are disabled to avoid unbounded PCD buffering; registered scan clouds
remain enabled.

For offline replay, use **two fresh terminals on this PC**, both with:

```bash
source /opt/ros/humble/setup.bash
source /home/fourier/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=126 ROS_LOCALHOST_ONLY=1
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

Terminal 1:

```bash
ros2 launch /home/fourier/astribot_sdk_ros2/astribot_scripts/launch/astribot_fast_lio.launch.py \
  use_sim_time:=true rviz:=false
```

Terminal 2, after the mapping node is ready:

```bash
ros2 bag play /home/fourier/astribot_sdk_ros2/robot_data_run01 --clock 100 \
  --topics /livox/lidar_back /livox/imu_back
```

Restart both processes to replay from the beginning. Do not use `--loop` across
a continuing filter state. Use a fresh terminal with the normal SDK environment
when returning to live data; the isolated replay settings are deliberate.

## Earlier front-pair validation on 2026-11-18

The remote Humble build succeeded for both `livox_ros_driver2` and `fast_lio`.
All seven new C++ parser tests passed (CTest reports eight results including its
wrapper), and all 38 existing `astribot_scripts` unit tests passed on that PC.
The launch arguments and runtime library dependencies were also checked. The
Mac's system Python lacks ROS/NumPy, so the ROS-dependent regression tests were
run on the remote PC rather than treated as passing locally.

The complete 101.68-second `robot_data_run01` recording was replayed at 1x in
ROS domain 126 with localhost-only discovery. Results:

| Check | Result |
| --- | --- |
| Odometry messages received | 1,012 |
| Odometry timestamp span | 101.100 seconds |
| Successive timestamp intervals | 97.47–102.57 ms, all increasing |
| Pose/quaternion values | All finite; unit quaternion norm within 1e-5 |
| Largest successive position step | 0.01192 m |
| Estimated accumulated path length | 3.232 m |
| Timestamp/parser rejection warnings | None |

There was one expected “No point, skip this scan” warning during IMU
initialization. The independent Best Effort monitoring subscriber received
1,016 clouds and 19,678 IMU messages; these counters are not proof of lossless
delivery to FAST-LIO. The source bag contains 1,017 / 20,337 respectively.
The node was deliberately stopped after replay; launch's shutdown log records
SIGINT termination. No mapping, playback or robot-control process was left
running by this check.

This is a functionality/sanity check without ground truth, not a measured
accuracy or drift benchmark. Noise tuning, physical extrinsic validation,
live motion validation and chassis-frame integration remain separate work.
The sensors' header clock is about **37 seconds ahead of bag receipt time**;
both lidar and its IMU share that clock discrepancy. Do not enter 37 seconds
as `time_offset_lidar_to_imu`. Cross-PC clock alignment matters when combining
these poses with other timestamped robot/camera data.

Machine-readable results are in
[`validation_20261118.json`](../integrations/fast_lio/validation_20261118.json).
Raw logs and the trajectory CSV are on the remote PC under
`/home/fourier/ros2_ws/validation/fast_lio_front_20261118/`. The one-off replay
checker is `/home/fourier/ros2_ws/validate_fast_lio_replay.py`.

## Rear model-based frame correction

As requested, the rear configuration is derived from the poses already
documented in `astribot_scripts/README.md` and `docs/imu_mounts_reference.md`,
without using vendor/support lidar calibration files. The source model is pinned
to `Astribot-Dev/astribot_descriptions` commit
`32b67e270fecd49bd5444ca49824dcc171dfb3e6`.

The existing notes say the rear topic is expressed in the front-lidar frame.
That is the explicit frame assumption for this nominal configuration; the shared
`livox_frame` label alone does not prove it. With `B = chassis_base`, `F = front
lidar`, `L = rear lidar`, and `I = rear IMU`, the two stages are:

```text
T_L_F = inverse(T_B_L) * T_B_F           # published cloud -> native rear lidar
T_I_L = inverse(T_L_I)                   # native rear lidar -> rear IMU
p_imu = T_I_L * T_L_F * p_published
```

The model-derived correction is approximately:

```yaml
preprocess:
  cloud_to_lidar_R: [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
  cloud_to_lidar_T: [-0.007071085, -0.492882842, -0.084]
```

The actual YAML retains the normalized source quaternions' full numeric
precision. The physical rear-lidar-to-IMU transform remains identity rotation
with translation `[-0.011, -0.02329, 0.04412]` m. Do not also compose the cloud
correction into `mapping.extrinsic_R/T`, which would apply it twice.

Preprocessing restores native coordinates **before** the 0.5 m blind-radius
filter. This matters because missing returns shifted away from zero by a driver
transform otherwise look like points near the robot. Timestamp, intensity,
line/tag filtering and measured point-time handling are preserved.

All model poses, assumptions and composed transforms are recorded in
[`rear_nominal_extrinsics.json`](../integrations/fast_lio/rear_nominal_extrinsics.json).
The configuration tests check the full published-cloud-to-rear-IMU chain against
those source poses. Nominal model poses may differ from the driver's actual
transform; replay success does not establish physical calibration accuracy.

## Rear-pair validation on 2026-11-18

The updated package built successfully. All **9 C++ parser tests** and **42
script tests** passed, including four checks for rear frame composition and ROS
parameter-array types. The running node's parameters were read back to confirm
the configured cloud correction and rear sensor pair.

The complete `robot_data_run01` bag was replayed at 1x in localhost-only ROS
domain 127. The monitor observed all **1,017 rear clouds** and **20,337 rear IMU
messages**, and received **1,013 odometry updates** spanning **101.200 seconds**.
All poses were finite, timestamps increased, quaternion norms were within 1e-5
of unity, and output intervals were **99.00–101.47 ms**. The maximum successive
position step was **0.018865 m**, with **3.275 m** estimated accumulated travel.
These are sanity metrics, not accuracy measurements against ground truth.

No parser rejection or “No Effective Points” warnings occurred. One empty-cloud
warning occurred during IMU initialization. The mapping node and bag player were
stopped after validation; this does not leave a live odometry process running.
An initial startup check caught mixed integer/float values in the generated
rotation array; the YAML was corrected and a regression test added before the
successful full replay.

Machine-readable results:
[`validation_rear_nominal_20261118.json`](../integrations/fast_lio/validation_rear_nominal_20261118.json).
Raw logs, effective parameters and trajectory CSV are on the remote PC under
`/home/fourier/ros2_ws/validation/fast_lio_rear_nominal_20261118/`. The replay
checker is `/home/fourier/ros2_ws/validate_fast_lio_rear_replay.py`.

## Rebuild the current installation

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/fourier/ros2_ws/livox_sdk2:${CMAKE_PREFIX_PATH:-}
export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4
cd /home/fourier/ros2_ws
colcon build --packages-select livox_ros_driver2 fast_lio --symlink-install \
  --executor sequential --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH=/home/fourier/ros2_ws/livox_sdk2/lib
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --test-result-base build/fast_lio --verbose
```

For a new workspace, clone the pinned repositories above, check out the exact
commits, and initialize FAST-LIO's submodule. In `livox_ros_driver2`, copy
`package_ROS2.xml` to `package.xml`. Build Livox-SDK2 with CMake and install to
the workspace-local prefix used above. Apply the patch from the FAST-LIO root:

```bash
git apply --check /path/to/astribot_scripts/integrations/fast_lio/astribot_pointcloud.patch
git apply /path/to/astribot_scripts/integrations/fast_lio/astribot_pointcloud.patch
git apply --check /path/to/astribot_scripts/integrations/fast_lio/world_frame.patch
git apply /path/to/astribot_scripts/integrations/fast_lio/world_frame.patch
```

Do not apply it twice. This PC already had PCL, Eigen and ROS interface build
tools. Only `ros-humble-pcl-ros` was added via apt; no existing apt packages were
upgraded. A fresh machine also needs the fork/driver dependencies and
`ament_cmake_gtest` for tests.

## Internet access used during setup

The Mac's existing `~/Desktop/internet-sharing/provider.py` provided HTTP/HTTPS
access through an SSH reverse tunnel. No shell startup files, desktop proxy
settings or routes were changed. Equivalent manual setup:

```bash
# Mac terminal 1; keep running while downloading/building.
python3 ~/Desktop/internet-sharing/provider.py 127.0.0.1/32 17892 --allow 127.0.0.1/32
# Mac terminal 2; authenticate normally, keep the tunnel running.
ssh -N -o ExitOnForwardFailure=yes -R 127.0.0.1:17892:127.0.0.1:17892 fourier@192.168.100.102
# Remote shell for downloads only:
export http_proxy=http://127.0.0.1:17892 https_proxy=http://127.0.0.1:17892
curl -I https://github.com
```

Stop both Mac processes after downloads; unset remote proxy variables before
reusing that shell without the tunnel. Both listeners are restricted to localhost.
