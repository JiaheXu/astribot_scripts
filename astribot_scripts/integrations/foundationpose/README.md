# Head and torso cameras for FoundationPose

The camera PC at `192.168.100.102` reads ROS domain 25. Its camera link publishes four `sensor_msgs/msg/Image` topics on domain 26, preserving their headers and sampling the newest image from each stream up to 3 Hz. On `192.168.200.102`, the Isaac ROS Humble container uses a Fast DDS unicast peer to discover the camera PC across the router.

| Domain 26 topic | Image |
| --- | --- |
| `/sdk/astribot_camera/head_rgbd/color_image` | 640×480 RGB |
| `/sdk/astribot_camera/head_rgbd/depth_image` | Native 640×576 depth |
| `/sdk/astribot_camera/torso_rgbd/color_image` | 640×480 RGB |
| `/sdk/astribot_camera/torso_rgbd/depth_image` | Native 640×480 depth |

The source prefers the existing SDK Image publishers. When one is silent it reads that stream's driver topic (`color_compress/compressed` or `depth_compress`). It waits for the camera drivers to publish. The source service restarts discovery when the robot LAN address changes.

## Operate the deployed listeners

On the camera PC, use `systemctl --user status astribot-camera-link`. Start or stop forwarding with `systemctl --user start astribot-camera-link` or `systemctl --user stop astribot-camera-link`.

On the GPU PC, inspect the running pipeline and frame counts with:

```bash
tail -f ~/workspaces/isaac_ros_humble/logs/astribot-foundationpose.log
```

The deployment runs on demand. To start it after a container restart, run on the GPU host:

```bash
docker exec -d isaac_ros_pose_humble bash -c '
  exec /workspaces/isaac_ros-dev/astribot_camera_inputs/astribot_scripts/integrations/foundationpose/run_gpu.sh \
    >>/workspaces/isaac_ros-dev/logs/astribot-foundationpose.log 2>&1'
```

To stop just this launch on the GPU host:

```bash
docker exec isaac_ros_pose_humble pkill -INT -f '^/usr/bin/python3 .*/ros2 launch .*/foundationpose/astribot\.launch\.py( |$)'
```

To inspect the topics, run `docker exec -it isaac_ros_pose_humble bash` on the GPU host, then:

```bash
source /opt/isaac_ros_ws/install/setup.bash
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/astribot_camera_inputs/astribot_scripts/integrations/foundationpose/gpu_peer.xml
ros2 node info /foundationpose/head_rgbd/foundationpose --no-daemon --spin-time 4
ros2 topic hz /sdk/astribot_camera/torso_rgbd/depth_image
```

## Inputs for pose estimation

Each camera has its own FoundationPose node under `/foundationpose/head_rgbd` or `/foundationpose/torso_rgbd`. The input adapter publishes rectified `rgb/image_rect_color` and `rgb/camera_info` in each namespace. It pairs RGB and depth capture timestamps within 60 ms. With measured calibration, it publishes `depth_registered/image_rect` as `32FC1` in meters on the rectified RGB grid with the matching RGB timestamp.

The deployed calibration is in `~/workspaces/isaac_ros_humble/astribot_camera_inputs/astribot_scripts/config/rgbd_pointcloud.yaml` on the GPU host. Both cameras have RGB intrinsics. Their depth intrinsics, distortion, unit scale and depth-to-color transforms are missing. Both native depth topics remain available to subscribers. Registered depth stays paused per camera until its calibration is supplied. After editing the file, restart the GPU launch. See the [calibration instructions](../../docs/rgbd_pointcloud.md).

For poses, publish a `mono8` mask of the target object to each camera's `segmentation` topic using its rectified RGB grid, optical frame and capture timestamp. The launch accepts `mesh:=...` and `texture:=...` paths inside the container and defaults to the installed mustard bottle model. Estimates appear on each camera's `output` topic as `vision_msgs/msg/Detection3DArray`.

## Deployment check

On November 20, 2026, the robot Ethernet adapter reported no carrier and the camera publisher had exited after a heartbeat timeout. A replay of the November 19 camera recording used a separate source domain (126), through the same camera-link executable to the deployed GPU inputs on domain 26. The GPU adapter received 29 head RGB and 32 head depth images, and 32 torso RGB and 32 torso depth images. It matched 18 head pairs and 28 torso pairs. Both FoundationPose nodes were running with separate RGB, CameraInfo, registered-depth and mask subscriptions. The replay is stopped. Live camera frames and poses still need verification with the robot connected.
