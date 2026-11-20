"""Start independent Isaac ROS FoundationPose graphs for the head and torso."""
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    assets = Path(os.environ['ISAAC_ROS_WS']) / 'isaac_ros_assets'
    sample = assets / 'isaac_ros_foundationpose/Mustard'
    models = assets / 'models/foundationpose'
    sdk_root = Path(__file__).resolve().parents[3]
    desc = LaunchDescription([
        DeclareLaunchArgument('calibration', default_value=str(sdk_root / 'astribot_scripts/config/rgbd_pointcloud.yaml')),
        DeclareLaunchArgument('mesh', default_value=str(sample / 'textured_simple.obj')),
        DeclareLaunchArgument('texture', default_value=str(sample / 'texture_map.png')),
        ExecuteProcess(cmd=[sys.executable, '-m', 'astribot_scripts.integrations.foundationpose.inputs',
                            '--ros-args', '-p', ['calibration:=', LaunchConfiguration('calibration')]],
                       cwd=str(sdk_root), output='screen', on_exit=Shutdown(reason='Camera input adapter exited')),
    ])
    for camera in ('head_rgbd', 'torso_rgbd'):
        namespace = '/foundationpose/' + camera
        component = ComposableNode(
            name='foundationpose', namespace=namespace,
            package='isaac_ros_foundationpose',
            plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
            parameters=[{
                'mesh_file_path': LaunchConfiguration('mesh'), 'texture_path': LaunchConfiguration('texture'),
                'refine_engine_file_path': str(models / 'refine_trt_engine.plan'),
                'score_engine_file_path': str(models / 'score_trt_engine.plan'),
                'refine_model_file_path': str(models / 'refine_model.onnx'),
                'score_model_file_path': str(models / 'score_model.onnx'),
                'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                'refine_input_binding_names': ['input1', 'input2'],
                'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
                'refine_output_binding_names': ['output1', 'output2'],
                'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                'score_input_binding_names': ['input1', 'input2'],
                'score_output_tensor_names': ['output_tensor'], 'score_output_binding_names': ['output1'],
                'tf_frame_name': camera + '_fp_object',
            }],
            remappings=[('pose_estimation/image', 'rgb/image_rect_color'),
                        ('pose_estimation/depth_image', 'depth_registered/image_rect'),
                        ('pose_estimation/camera_info', 'rgb/camera_info'),
                        ('pose_estimation/segmentation', 'segmentation'),
                        ('pose_estimation/output', 'output')])
        desc.add_action(ComposableNodeContainer(
            name='container', namespace=namespace, package='rclcpp_components',
            executable='component_container_mt', composable_node_descriptions=[component],
            output='screen', on_exit=Shutdown(reason=f'{camera} container exited')))
    return desc
