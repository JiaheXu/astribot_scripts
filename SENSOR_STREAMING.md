# Independent LiDAR and camera processes

Source the same ROS/SDK environment in both terminals on the receiving computer.
Both processes must use the same `ROS_DOMAIN_ID` as the robot.

Run the existing LiDAR example in the first terminal:

```bash
python3 examples/301-get_lidar_scan.py
```

Example 301 starts the LiDAR driver and records the existing `PointCloud2` topics
`/livox/lidar_front` and `/livox/lidar_back` into a ROS 2 bag. The original
messages and header timestamps are preserved, so the result can be replayed
directly by a lidar-odometry pipeline:

```bash
python3 examples/301-get_lidar_scan.py --duration 60
ros2 bag info recordings/lidar_<timestamp>
ros2 bag play recordings/lidar_<timestamp>
```

Use `--output /path/to/bag` to select the bag URI, or `--no-activate` to attach
to a driver that is already running. The example prints per-stream scan and
point counts while it records. Press `q` (or Ctrl-C) to close the bag cleanly.

Run the independent camera converter in the second terminal:

```bash
python3 camera_raw_node.py
```

It listens to existing `/astribot_camera/<camera>/color_compress/compressed`
topics. It does not start, stop, or configure camera drivers and does not import
the Astribot SDK. The camera driver must already be publishing JPEG messages.

| Camera input | Raw output under `/sdk/astribot_camera/` |
| --- | --- |
| `head_rgbd` | `head_rgbd/color_image` |
| `torso_rgbd` | `torso_rgbd/color_image` |
| `left_wrist_rgbd` | `left_wrist_rgbd/color_image` |
| `right_wrist_rgbd` | `right_wrist_rgbd/color_image` |
| `head_stereo` | `head_stereo/left_image`, `head_stereo/right_image` |

Each output is a raw `sensor_msgs/Image` in `bgr8`, resized to 320×240 with its
original timestamp and frame ID. The stereo JPEG is split into left/right views
before resizing. Resizing does not reduce incoming network bandwidth.

To subscribe only to the head and wrists:

```bash
python3 camera_raw_node.py --cameras head_rgbd left_wrist_rgbd right_wrist_rgbd
```

The converter keeps one pending JPEG and a separate decoder worker for each
camera. Its five-second reports distinguish compressed input rate from converted
frame rate. For stereo, one converted source frame produces two images; JSON
`output_counts` records each view separately. Zero received frames means there
was no input to decode, not necessarily a decoding error.

For a timed diagnostic:

```bash
python3 camera_raw_node.py --duration 60 --report /tmp/camera_raw_report.json
```

If you need additional `/sdk/astribot_lidar/{front,back}_scan` relay topics, run
`python3 lidar_node.py --activate` in place of example 301. Avoid running multiple
copies of a camera converter or the combined `main.py`/`lidar_head_stream.py`
alongside it, since they publish overlapping image topics.

`main.py` and the camera examples are unchanged. Example 301 is the LiDAR bag
recorder described above.
