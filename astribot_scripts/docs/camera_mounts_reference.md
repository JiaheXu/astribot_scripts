# Astribot S1 nominal camera mounts

Source: official `Astribot-Dev/astribot_descriptions`, commit
`32b67e270fecd49bd5444ca49824dcc171dfb3e6`. These are simulation model values,
not verified factory calibration for the connected robot. The inspected local
SDK URDF/SDF/YAML files do not contain camera mounts.

## Positions relative to moving link frames

Positions are `[x, y, z]` in meters, expressed in the named parent link's axes.
The associated joint is at `[0,0,0]` in that link in the MJCF. These are offsets
from the moving link/joint origin, not from its upstream parent or from world.

| Camera | Reference link | Associated joint | Position |
| --- | --- | --- | --- |
| `head_rgbd` | `astribot_head_link_2` | `astribot_head_joint_2` | `[0.0603, -0.2139, -0.0277]` |
| `head_stereo_left` | `astribot_head_link_2` | `astribot_head_joint_2` | `[0.035, 0, 0]` |
| `head_stereo_right` | `astribot_head_link_2` | `astribot_head_joint_2` | `[-0.035, 0, 0]` |
| `torso_rgbd` | `astribot_torso_link_4` | `astribot_torso_joint_4` | `[0, 0, 0.1]` |
| `left_wrist_rgbd` | `astribot_arm_left_link_7` | `astribot_arm_left_joint_7` | `[0.01142099616, -0.09111799978, 0.04426011885]` |
| `right_wrist_rgbd` | `astribot_arm_right_link_7` | `astribot_arm_right_joint_7` | `[0.00772, -0.09408, 0.04319]` |

Source links:

- [kinematic.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/kinematic.xml#L438): torso/head attachment hierarchy, head RGB-D, and wrist camera definitions (left at line 490, right at line 569).
- [head_stereo_left.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/head_stereo_left.xml).
- [head_stereo_right.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/head_stereo_right.xml).
- [torso_rgbd.xml](https://github.com/Astribot-Dev/astribot_descriptions/blob/32b67e270fecd49bd5444ca49824dcc171dfb3e6/astribot_s1/model/mjcf/robot/sensors/torso_rgbd.xml).

For each wrist the MJCF inserts `arm_left_tool_cam` / `arm_right_tool_cam` at
`[0,-0.15,0]`, with identity rotation, beneath arm link 7. The positions above
include that offset. The camera-local positions before composition are:

```text
left:  [0.011420996159648984, 0.058882000224149605, 0.044260118849329295]
right: [0.00772,              0.05592,              0.04319]
```

The two stereo includes are direct children of `astribot_head_link_2`.
`astribot_head_camera_base_link` is a separate, self-closing sibling body;
its `[0.09,0,0.08]` offset must not be added to the stereo camera positions.

## Source orientations

These quaternions are **MJCF `[w,x,y,z]`**, relative to the reference links above,
and describe MuJoCo camera axes. Normalize before use, especially the rounded
`head_rgbd` quaternion (norm approximately 0.99932915).

```text
head_rgbd:          [0.7025, -0.0295, -0.71, -0.0135]
head_stereo_left:   [0.7071068, 0, 0.7071068, 0]
head_stereo_right:  [0.7071068, 0, 0.7071068, 0]
torso_rgbd:         [1, 0, 0, 0]
left_wrist_rgbd:    [0.703108425353722, -0.7109134990656752, -0.009037420129074155, -0.012604129490710232]
right_wrist_rgbd:   [-0.7130607009975457, 0.7006910271198059, 0.013493708973317153, 0.019860539389038687]
```

[MuJoCo cameras](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera)
look along -Z, with +X right and +Y up.
[ROS optical frames](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
use +Z forward, +X right and +Y down. To express the same modeled camera with
ROS optical axes, compose on the right:

```text
R_link_optical = R_link_mujoco_camera @ diag(1, -1, -1)
t_link_optical = t_link_mujoco_camera
```

Then encode the resulting normalized quaternion as ROS `[x,y,z,w]`. Merely
reordering the source quaternion is insufficient for an optical-frame transform.
This conversion changes axes only; it does not establish the extrinsics of a
physical RGB/depth sensor or calibrate separate color/depth optical centers.

For a world/base pose, compose the current link forward kinematics with this
fixed camera mount. Verify that the installed SDK and MJCF use the same link
frames before doing so; matching link names alone are insufficient. No runtime
image headers or CameraInfo are changed by the visualization integration.

## Generated visualization frames

`config/camera_frames.yaml` stores the four active RGB mounts. The generated
visualization URDF adds fixed links named `astribot_viz_<camera>_optical` for
`head_rgbd`, `torso_rgbd`, `left_wrist_rgbd` and `right_wrist_rgbd`. The combined
RViz configuration shows them in the **Camera optical frames** group. Stereo
frames are omitted from the active visualization.

The incoming head-link-2, torso-link-4 and wrist-link-7 joint rotations and
translations were checked against the SDK URDF before composing these mounts.
The vendor URDF remains unchanged. These frames follow the articulated model
in `map`; they describe nominal model geometry, not measured RGB extrinsics.
