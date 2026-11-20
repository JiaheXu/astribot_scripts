# Astribot S1 nominal IMU mounts

Source: official `Astribot-Dev/astribot_descriptions`, commit
`32b67e270fecd49bd5444ca49824dcc171dfb3e6`. The S1 MJCF defines three IMU
locations, each with a gyro and accelerometer. These are model mounts, not
verified per-robot calibration or a statement that the hardware has no other IMUs.

## Poses relative to the chassis joint group

All three are fixed relative to the model's `chassis_base`. Positions below are
in meters in that body's axes; roll/pitch/yaw are in degrees. LiDAR IMU poses
have been composed through their respective LiDAR mounts, including rotation
of the local IMU offset.

| IMU site | Position in `chassis_base` | Roll, pitch, yaw |
| --- | --- | --- |
| `astribot_chassis_imu_site` | `[0.16117, 0.082, 0.01541]` | `[0, 0, 0]` |
| `lidar_imu_site` (front) | `[0.198176691, 0.177620345, 0.029277]` | `[0, 0, -45]` approximately |
| `lidar_back_imu_site` (rear) | `[-0.203837491, -0.183281143, 0.113277]` | `[0, 0, 135]` approximately |

In [s1_kinematic.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/s1_kinematic.xml),
the `chassis_base` body carries joints `astribot_chassis_x`,
`astribot_chassis_y`, and `astribot_chassis_zrot`. In the dynamic
[s1.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/s1.xml),
it instead carries the free joint `chassis_free`. These mounts do not depend
on torso, arm or head joint angles. The entry points place the chassis at initial
world Z values of 0.097 and 0.095 respectively; neither value is added to a pose
expressed relative to `chassis_base`.

## Direct parent mounts and source definitions

- **Chassis IMU:** [kinematic.xml, line 5](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/kinematic.xml#L5)
  places the site directly in `chassis_base` at `[0.16117,0.082,0.01541]`.
  No orientation is specified; it is identity (the site defaults do not override
  orientation). `kinematic_visual.xml` has the identical site.
- **Front LiDAR IMU:** [lidar_mid360_front.xml, line 5](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/lidar_mid360_front.xml#L5)
  places `lidar_imu_site` inside `lidar_front` at `[0.011,0.02329,-0.04412]`,
  with identity orientation relative to that LiDAR body.
- **Rear LiDAR IMU:** [lidar_mid360_back.xml, line 5](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/lidar_mid360_back.xml#L5)
  places `lidar_back_imu_site` inside `lidar_back` at the same local offset
  `[0.011,0.02329,-0.04412]`, also with identity local orientation.

The sensor declarations bind gyro/accelerometer pairs to those sites:

| Definition | Gyro | Accelerometer |
| --- | --- | --- |
| [imu_chassis.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/imu_chassis.xml) | `astribot_chassis_base_imu_gyro` | `astribot_chassis_base_imu_acc` |
| [imu_front.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/imu_front.xml) | `lidar_imu_gyro` | `lidar_imu_acc` |
| [imu_back.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/imu_back.xml) | `lidar_back_imu_gyro` | `lidar_back_imu_acc` |

## Composition and quaternion conventions

The transform maps IMU coordinates into the reference body's coordinates:

```text
T_chassis_imu = T_chassis_lidar @ T_lidar_imu
t_chassis_imu = t_chassis_lidar + R_chassis_lidar @ [0.011, 0.02329, -0.04412]
R_chassis_imu = R_chassis_lidar
```

Normalized ROS-order quaternions `[x,y,z,w]` relative to `chassis_base`:

```text
chassis: [0, 0, 0, 1]
front:   [0, 0, -0.382683380879, 0.923879553838]
rear:    [0, 0,  0.923879539193, 0.382683416234]
```

These are normalized/reordered from the MJCF body quaternions; unlike a
MuJoCo camera-to-ROS-optical conversion, no camera-axis flip applies to IMUs.
The computed mounts were checked by transforming back to the original local
IMU offsets. Both model entry points reference the same LiDAR/IMU definitions.

The inspected local SDK URDF/SDF/YAML and official whole-body URDF/SDF do not
declare these IMU frames. Before using the model values with live ROS IMU data,
verify the actual message coordinate frame and any driver-applied rotation, and
establish how MJCF `chassis_base` relates to SDK `chassis`. A transformed LiDAR
point cloud does not prove its IMU measurements underwent the same transform.
No runtime TF, calibration or SDK files were changed by this inspection.
