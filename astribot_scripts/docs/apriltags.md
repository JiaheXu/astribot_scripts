# AprilTag / ArUco center axes in the combined RViz

The existing `apriltag_utils` package detects tags and estimates their poses from
the four calibrated 640×480 RGB streams. The SDK's `apriltag_rviz.py` subscribes
to its `vision_msgs/Detection2DArray` outputs and draws a coordinate triad at
each tag's **physical center**. It uses the 3D pose, not the bounding-box center.

- Red = tag +X, green = tag +Y, blue = tag +Z; each axis is 0.10 m long by default.
- The label shows camera name, family, and tag ID.
- The native AprilTag convention is X right, Y down, Z into an upright tag.
- Each camera has its own marker namespace. Seeing the same ID from two cameras
  gives two independent pose estimates; they are not fused.
- Tags disappear immediately on an empty/changed detection result. Markers expire
  after one second if detection stops, and missing/stale camera TF clears that
  camera's previous axes.

`apriltag_utils/config/tags.yaml` selects **AprilTag tag36h11, ID 1, side 60 mm**.
Measure the outer black square, excluding the white margin. Only this tag ID is
published; ArUco detection is inactive. If the installed package has an older
configuration, use `apriltag_tags:=PATH` and restart the detector and axes adapter.

## Start with the existing system

The package was found at `../apriltag_utils` beside this SDK checkout. The launch
prefers a built/sourced ROS package, then falls back to that sibling source folder
if the active Python can import `vision_msgs`, `cv_bridge`, OpenCV, and the
selected detector (`pupil_apriltags` for AprilTag, `cv2.aruco` for ArUco).
It reuses the package's `config/cameras.yaml` and `config/tags.yaml`; no camera
intrinsics or tag sizes are copied or invented in the SDK.

Build/install the detector using its own README if needed. On the ROS Humble PC,
source ROS, this SDK's `env.sh`, and the workspace containing `apriltag_utils`:

```bash
source /opt/ros/humble/setup.bash
source ./env.sh
source "$HOME/ros2_ws/install/local_setup.bash"
```

The existing combined launch now defaults to `apriltags:=auto`: when the package
and dependencies are available, it starts the detector and marker adapter along
with RViz. If they are absent, the normal visualization starts and logs why tag
axes are unavailable. Use `apriltags:=true` to require detection, or
`apriltags:=false` to disable it explicitly. No SDK connection or motion command
is added by this feature.

If the existing sensor/FAST-LIO/RViz stack is **already running**, start only
the detector and axes in a separate sourced terminal, from the SDK root:

```bash
ros2 launch ./astribot_scripts/launch/astribot_apriltag.launch.py
```

Do not run this standalone launch if the combined view already started the same
detector/adapter. When a detector is already running separately, reuse it:

```bash
ros2 launch ./astribot_scripts/launch/astribot_apriltag.launch.py \
  apriltag_start_detector:=false
```

Then reload `astribot_scripts/config/astribot_fast_lio.rviz` using RViz
**File → Open Config**. Keep **Fixed Frame = map** and enable
**AprilTag detections → Tag centers and RGB axes**. The display reads
`/astribot_viz/apriltag_axes`. Reloading this RViz config does not restart mapping.

For a fresh combined view, the equivalent launch is:

```bash
ros2 launch ./astribot_scripts/launch/astribot_lio_rviz.launch.py apriltags:=true
```

Both launches accept:

| Argument | Default / purpose |
| --- | --- |
| `apriltag_root` | Sibling `../apriltag_utils`; source fallback when no installed package is found |
| `apriltag_cameras` | Detector package's calibrated `config/cameras.yaml` |
| `apriltag_tags` | Detector package's `config/tags.yaml`, default `tag36h11` |
| `apriltag_axis_length` | `0.1`, meters |
| `apriltag_start_detector` | `true`; `false` runs only the axes adapter |
| `apriltag_tf_time` | `latest`; optional `measurement` for synchronized image-time TF |
| `use_sim_time` | `false`; use `true` for a matching bag-clock visualization |

If running from source, use the detector's ROS-compatible environment described
in its README. The source fallback uses the same interpreter as `ros2 launch`;
an installed detector uses its installed executable/interpreter. A missing native
detector dependency is not resolved merely by adding a source path.

## Frames and timing

The calibrated detector frames already match this SDK's generated optical frames:
`astribot_viz_head_rgbd_optical`, `astribot_viz_torso_rgbd_optical`, and the two
wrist optical frames. The detector allows empty image frame IDs but rejects
nonempty ones that disagree with its configured optical frame.

The adapter listens on `/astribot_viz/tf` and `/astribot_viz/tf_static`, matching
the existing combined RViz tree. Default `latest` mode composes
`T_map_camera × T_camera_tag` using recent display TF, then freezes each result
in `map` until the next detection. Old axes therefore do not move with the camera
between images. This accommodates the system's known sensor-clock discrepancy;
it is an approximate display overlay, **not synchronized tag localization**.
The detector's image timestamps and pose messages remain unchanged. Nominal
camera-to-robot mounts also limit map-frame accuracy.

Use `apriltag_tf_time:=measurement` only with synchronized clocks and a TF tree
covering the image timestamps. It requests the exact image-time transform and
drops detections when that transform is unavailable. Neither mode publishes tag
TF or creates a competing robot TF tree. The RGB triad is a MarkerArray visual;
the native detector's optional `publish_tf` feature is not needed.

## Troubleshooting and tests

```bash
ros2 topic echo /sdk/astribot_camera/head_rgbd/tag_detections
ros2 topic echo /astribot_viz/apriltag_axes
ros2 run tf2_ros tf2_echo map astribot_viz_head_rgbd_optical \
  --ros-args -r /tf:=/astribot_viz/tf -r /tf_static:=/astribot_viz/tf_static
python3 -m unittest astribot_scripts.tests.test_apriltag_rviz -v
```

Empty detection arrays mean no configured tag passed detection/pose checks.
Check the family, ID list, physical size, image resolution, border visibility,
and detector warnings. Valid detections but no axes usually indicate missing
optical-frame TF or an unloaded RViz display. The current sensor launcher already
publishes the four required RGB streams at 640×480 and up to 3 Hz.

Offline tests cover axis origins/orientations, camera transform composition,
multi-camera ID isolation, deletion/expiry, invalid poses, latest/measurement TF,
and installed/source launch wiring. Live camera/RViz rendering still needs the
ROS receiving PC.
