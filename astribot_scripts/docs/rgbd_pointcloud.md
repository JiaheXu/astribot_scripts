# Head and torso RGB-D clouds in the SLAM view

The combined `astribot_lio_rviz.launch.py` starts `rgbd_pointcloud.py` alongside
the existing FAST-LIO display. RViz has an enabled **Head and torso RGB-D clouds**
group with RGB8 coloring. Only head and torso clouds are processed. The existing
wrist RGB image displays remain available.

**Current calibration status:** the checked-in configuration contains the actual
640×480 RGB intrinsics from the November 19 Kalibr calibration in
`../apriltag_utils/config/cameras.yaml`. Native depth dimensions and raw transport
come from `record_rgbd_mcap.py`. Depth intrinsics, distortion, depth units, and
depth-to-color extrinsics have not been verified. Their YAML values are `null`.
The node reports the missing calibration and publishes sampled depth images,
but intentionally does not create a cloud for an uncalibrated camera. These
values must be completed before colored clouds will appear. Matching image
dimensions alone do not prove RGB/depth registration.

## Inputs and outputs

| Camera | RGB input (640×480, up to 3 Hz) | Native depth input |
| --- | --- | --- |
| Head | `/sdk/astribot_camera/head_rgbd/color_image` | `/astribot_camera/head_rgbd/depth_compress`, raw uint16 640×576 |
| Torso | `/sdk/astribot_camera/torso_rgbd/color_image` | `/astribot_camera/torso_rgbd/depth_compress`, raw uint16 640×480 |

For each `<camera>` (`head_rgbd` or `torso_rgbd`), the node publishes:

| Topic | Meaning |
| --- | --- |
| `/sdk/astribot_camera/<camera>/depth_image` | Sampled native depth with the original header and numeric values |
| `/sdk/astribot_camera/<camera>/points` | XYZ in meters in the configured RGB optical frame, packed PCL RGB, original depth capture timestamp |
| `/astribot_viz/<camera>/points` | Cloud transformed once into `map` for the combined RViz overlay |

All use best-effort, volatile QoS. The node adds no SDK connection or robot
control. The sensor pane keeps `--no-depth`: this node owns the two sampled
depth outputs. Do not run another publisher on the same output topics.
Standard `sensor_msgs/Image` depth inputs are also supported by setting
`depth_transport: image` and `depth_topic`. The node does not republish depth if
that input is already its sampled output topic. `16UC1` uses the configured
meters-per-unit scale; `32FC1` follows the ROS convention of meters.

## Complete calibration on the receiving PC

Edit `astribot_scripts/config/rgbd_pointcloud.yaml` for each camera:

1. Obtain the depth intrinsics and distortion for the exact emitted depth raster
   from the camera driver/SDK or a matching `CameraInfo`. Set `depth.camera_matrix`
   to its row-major 3×3 K and `depth.distortion_coefficients` to the measured
   coefficients. Supported models are `plumb_bob` (5 values) and
   `rational_polynomial` (8). For a verified rectified image, use its rectified K
   and zero distortion.
2. Set `depth_unit_m` to the verified meters per uint16 unit (for example,
   `0.001` **only if the driver confirms millimeters**).
3. Set `color_from_depth` to the measured 4×4 rigid transform that maps a depth
   optical point into the RGB optical frame: `p_color = R * p_depth + t`, with
   translation in meters. The projector applies depth undistortion, this
   transform, and RGB distortion before sampling color.
4. If the driver already aligns depth to RGB, use the calibration of that
   **aligned output**, its actual dimensions, and identity `color_from_depth`.
   Confirm whether it is aligned to raw or rectified color. The supplied RGB
   intrinsics describe the raw, resized 640×480 stream. Do not stretch the native
   head depth image to RGB size and assume registration.
5. Set `depth_input_frame` to any nonempty frame ID actually carried by the
   depth driver. Empty source headers are accepted; unexpected nonempty frames
   are rejected. The color frame must be empty or match `optical_frame`.

Camera-to-robot mounts come from `camera_frames.yaml` and are nominal. They
limit overlay accuracy even after RGB-D registration is calibrated.

## Launch and verification

The normal desktop/tmux combined launch includes this node by default. To add
it to an **older running view that does not already contain it**, without
restarting FAST-LIO, run from the SDK root:

```bash
source /opt/ros/humble/setup.bash
source ./env.sh
python3 -m astribot_scripts.rgbd_pointcloud --ros-args \
  -r /tf:=/astribot_viz/tf -r /tf_static:=/astribot_viz/tf_static
```

Then use RViz **File → Open Config** to load
`astribot_scripts/config/astribot_fast_lio.rviz`. Keep Fixed Frame `map` and
enable the head/torso cloud group. This reload does not restart mapping.
After configuration changes, restart the RGB-D node. Avoid duplicate nodes.

Combined launch arguments:

- `rgbd_clouds:=false` disables the RGB-D node and its sampled depth publishing.
- `rgbd_config:=/absolute/path/config.yaml` selects another calibration file.
- `rgbd_tf_time:=latest` is the default display mode.
- `rgbd_tf_time:=measurement` uses capture-time TF; it requires synchronized
  clocks and TF available at the image timestamps.

Standalone ROS parameters include `rate` (3 Hz), `stride` (2 depth pixels),
`min_depth` (0.2 m), `max_depth` (5 m), `sync_slop` (0.06 s),
`max_pair_age` (1 s), `max_tf_age` (1 s), and `tf_time_mode`.
Images are paired using their original capture timestamps, with bounded queues.
Zero timestamps, unmatched frames, invalid depths and points outside RGB are
discarded. Each pair is consumed once. A nearest-depth test handles points
projecting onto the same RGB pixel, reducing color bleeding at occlusions.

The default `latest` mode accommodates the existing sensor/display clock offset:
it freezes each cloud using recent camera TF into `map` and stamps the display
cloud with display time. The camera cloud retains its capture timestamp. This
is an approximate visualization overlay; FAST-LIO continues using its existing
LiDAR/IMU inputs. RGB-D clouds are not fused into the SLAM estimate or saved map.
RViz expires the overlay after one second so stale geometry does not accumulate.
The desktop's existing readiness check covers the original RGB/SLAM streams;
verify these new clouds separately:

```bash
ros2 topic hz /sdk/astribot_camera/head_rgbd/points
ros2 topic hz /sdk/astribot_camera/torso_rgbd/points
ros2 topic hz /astribot_viz/head_rgbd/points
ros2 topic hz /astribot_viz/torso_rgbd/points
```

The node reports received, paired and published counts every ten seconds. If
camera clouds arrive but map clouds do not, check `/astribot_viz/tf` and
`/astribot_viz/tf_static` for fresh `map` to optical transforms. If no pairs form,
inspect source timestamps before changing `sync_slop`. Missing calibration is
reported per camera at startup.

Offline verification (NumPy, OpenCV, SciPy and PyYAML; no robot/ROS required):

```bash
python3 -m unittest astribot_scripts.tests.test_rgbd_pointcloud -v
```
